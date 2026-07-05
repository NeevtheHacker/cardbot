// src/main.cpp
#include "main.h"
#include <cmath>
#include "lemlib/api.hpp" // IWYU pragma: keep
using namespace pros;

// ==================== CONFIG (EDIT THESE) ====================
//Auton Path 
ASSET(AutonRunRightRedLGCluster_txt);
ASSET(backtomatchload_txt);
ASSET(Intomatchloaderfinal_txt);
ASSET(ToLonggoalRightBlue_txt);
ASSET(testPath_txt);

//Drive Motor Ports
constexpr int8_t L_FRONT_PORT       = -20;
constexpr int8_t L_BACK_PORT        = -19;
constexpr int8_t L_BACK_STACK_PORT  = 18;
constexpr int8_t R_FRONT_PORT       = 10;
constexpr int8_t R_BACK_PORT       = 9;
constexpr int8_t R_BACK_STACK_PORT = -8;

// Odometry ports
constexpr int8_t IMU_PORT = 16;
constexpr int8_t VERTICAL_ROTATION_SENSOR_PORT = 3;
constexpr int8_t HORIZONTAL_ROTATION_SENSOR_PORT = 2;

// InOutMechanism ports
// Clockwise Direction
constexpr int8_t OUTER_TOWER_MIDDLE_InOutMechanism_PORT      = -12;
constexpr int8_t INNER_TOWER_MIDDLE_TOP_InOutMechanism_PORT  = -11;
constexpr int8_t INNER_TOWER_LOWER_InOutMechanism_PORT       = 13;

// Outtake ports - todo
constexpr int8_t TOP_OUTTAKE_PORT = -14;

// Pneumatics (3-wire/ADI) port
constexpr char PISTON_PORT = 'H';  // Change to your 3-wire port (A..H)
constexpr char PISTON_PORT_DESCORE = 'F';  // Change to your 3-wire port (A..H)

// Drive style
constexpr bool kArcadeDrive = true;

// Lemlib constants
constexpr int8_t TRACK_WIDTH  = 12.25; // inches
constexpr int8_t WHEEL_BASE   = 8; // inches

// Shaping
constexpr int    kDeadband = 5;
constexpr double kExpo      = 1.6;
constexpr int    kMaxCmd    = 127;

// Loop rate
constexpr int kDriveLoopMs  = 10;
constexpr int kInOutMechanismLoopMs = 10;

// InOutMechanism powers
constexpr int kInOutMechanismCmd = 127;
constexpr int kHoldCmd   = 18;

// Controller button mapping
constexpr auto BTN_BRAKE_HOLD         = E_CONTROLLER_DIGITAL_A;
constexpr auto BTN_BRAKE_COAST        = E_CONTROLLER_DIGITAL_UP;
constexpr auto BTN_INTOPSTORAGE       = E_CONTROLLER_DIGITAL_R1;
constexpr auto BTN_INLOWSTORAGE       = E_CONTROLLER_DIGITAL_R2;
constexpr auto BTN_TOPOUTTAKE         = E_CONTROLLER_DIGITAL_L1;
constexpr auto BTN_OUTMIDGOAL         = E_CONTROLLER_DIGITAL_L2;
constexpr auto BTN_OUTLOWGOAL         = E_CONTROLLER_DIGITAL_Y;
constexpr auto BTN_midToTopStorage    = E_CONTROLLER_DIGITAL_LEFT;
constexpr auto BTN_InOutMechanism_OFF = E_CONTROLLER_DIGITAL_X;
constexpr auto BTN_DriveReverse = E_CONTROLLER_DIGITAL_B;




// Pneumatic toggle button
constexpr auto BTN_PISTON_TOGGLE = E_CONTROLLER_DIGITAL_RIGHT;
constexpr auto BTN_PISTON_TOGGLE_DESCORE = E_CONTROLLER_DIGITAL_DOWN;

// ==================== DEVICES ====================
Controller master(E_CONTROLLER_MASTER);

// Drive train motor objects
MotorGroup leftDrive({L_FRONT_PORT, L_BACK_PORT, L_BACK_STACK_PORT}, v5::MotorGears::green, v5::MotorUnits::rotations);
MotorGroup rightDrive({R_FRONT_PORT, R_BACK_PORT, R_BACK_STACK_PORT}, v5::MotorGears::green, v5::MotorUnits::rotations);
bool driveReversed = false;

// InOutMechanism motors
Motor outerTowerMiddleMotor(OUTER_TOWER_MIDDLE_InOutMechanism_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
Motor innerTowerMiddleTopMotor(INNER_TOWER_MIDDLE_TOP_InOutMechanism_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
Motor innerTowerLowerMotor(INNER_TOWER_LOWER_InOutMechanism_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

// Outtake Motor
Motor topOutTakeMotor(TOP_OUTTAKE_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

// ==================== PNEUMATICS ====================
pros::adi::Pneumatics piston(PISTON_PORT, false); // false = retracted at startup
static bool piston_extended = false;
pros::adi::Pneumatics piston_descore(PISTON_PORT_DESCORE, false); // false = retracted at startup
static bool piston_extended_descore = false;

// ==================== HELPERS ====================
static inline int clamp127(int v) { return std::max(-127, std::min(127, v)); }

static inline int applyDeadband(int v, int db = kDeadband) {
  return (std::abs(v) < db) ? 0 : v;
}

static inline int expoCmd(int v, double expo = kExpo) {
  const double s = (double)v / 127.0;
  const double shaped = std::copysign(std::pow(std::abs(s), expo), s);
  return clamp127((int)std::round(shaped * std::min(127, kMaxCmd)));
}

static inline void driveTank(int left, int right) {
  leftDrive.move(clamp127(left));
  rightDrive.move(clamp127(right));
}

static inline void driveArcade(int throttle, int turn) {
  int l = clamp127(throttle + turn);
  int r = clamp127(throttle - turn);
  driveTank(l, r);
}

pros::Mutex lcd_mutex;
void safe_lcd_set(int line, std::string text) {
    lcd_mutex.take();
    pros::lcd::set_text(line, text);
    lcd_mutex.give();
}



// InOutMechanism - This class handles all operations for intake and outtake from the robot

class InOutMechanism {
public:
  enum class Mode { Off, 
                    InTopStorage,
                    InTopStorageOff,
                    InLowStorage,
                    InLowStorageOff,
                    OutMiddleGoal,
                    OutMiddleGoalOff,
                    OutLowGoal,
                    OutLowGoalOff,
                    OutTopGoal,
                    OutTopGoalOff,
                    MidToTopStorage,
                    MidToTopStorageOff
                  };

  InOutMechanism(Motor& L1,
                 Motor& L2,
                 Motor& R1,
                 Motor& T1) : m_outerTowerMiddleMotor(L1),
                              m_innerTowerMiddleTopMotor(L2),
                              m_innerTowerLowerMotor(R1),
                              m_topGoalMotor(T1) {}

  void set_mode(Mode m) { mode_ = m; }
  Mode mode() const { return mode_; }

  // Call from a background task
  void update() {
    // TODO - ramp speeds instead of instant full voltage
    // TODO - Should we have the motors on 'hold' as 
    // opposed to on 'coast' to ensure that objects
    // remain in place?

    switch (mode_) {
      case Mode::Off:
        m_outerTowerMiddleMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        m_innerTowerLowerMotor.move(0);
        m_topGoalMotor.move(0);
        break;
      case Mode::InTopStorage:
        m_outerTowerMiddleMotor.move(127);
        m_innerTowerLowerMotor.move(10);
        m_innerTowerMiddleTopMotor.move(-127);
        break;
      case Mode::InTopStorageOff:
        m_outerTowerMiddleMotor.move(0);
        m_innerTowerLowerMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        break;
      case Mode::InLowStorage:
        m_topGoalMotor.move(10);
        m_outerTowerMiddleMotor.move(127);
        m_innerTowerLowerMotor.move(0);
        m_innerTowerMiddleTopMotor.move(127);
        break;
      case Mode::InLowStorageOff:
        m_topGoalMotor.move(0);
        m_outerTowerMiddleMotor.move(0);
        m_innerTowerLowerMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        break;
      case Mode::OutMiddleGoal:
        m_outerTowerMiddleMotor.move(127);
        m_innerTowerMiddleTopMotor.move(-127);
        m_innerTowerLowerMotor.move(-127);
        break;
      case Mode::OutMiddleGoalOff:
        m_outerTowerMiddleMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        m_innerTowerLowerMotor.move(0);
        break;
      case Mode::OutLowGoal:
        m_outerTowerMiddleMotor.move(-127);
        m_innerTowerMiddleTopMotor.move(-127);
        m_innerTowerLowerMotor.move(-85);
        break;
        case Mode::OutLowGoalOff:
        m_outerTowerMiddleMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        m_innerTowerLowerMotor.move(0);
        break;
      case Mode::OutTopGoal:
        m_topGoalMotor.move(-127);
        m_innerTowerMiddleTopMotor.move(-127);
        break;
      case Mode::OutTopGoalOff:
        m_topGoalMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        break;
      case Mode::MidToTopStorage:
        // m_outerTowerMiddleMotor.move(127);
        // m_innerTowerMiddleTopMotor.move(-127);
        m_innerTowerLowerMotor.move(127);
        break;
      case Mode::MidToTopStorageOff:
        m_outerTowerMiddleMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        m_innerTowerLowerMotor.move(0);
        break;
      default:
        m_outerTowerMiddleMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        m_innerTowerLowerMotor.move(0);
        m_topGoalMotor.move(0);
        break;
    }
  }

private:
  Motor& m_outerTowerMiddleMotor;
  Motor& m_innerTowerMiddleTopMotor;
  Motor& m_innerTowerLowerMotor;
  Motor& m_topGoalMotor;
  Mode mode_ = Mode::Off;
};

InOutMechanism inOutMech(outerTowerMiddleMotor, 
                         innerTowerMiddleTopMotor,
                         innerTowerLowerMotor,
                         topOutTakeMotor);

void InOutMechanismTaskFn(void*) {
  
  // Print the state for debugging
  //printf("InoutMechanismTask state: %d\n", pros::Task::current().get_state());
  outerTowerMiddleMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  innerTowerMiddleTopMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  innerTowerLowerMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  topOutTakeMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  while (true) { 
    inOutMech.update(); 
    delay(kInOutMechanismLoopMs);
  }
}
Task InOutMechanismTask(InOutMechanismTaskFn, nullptr, "InOutMechanism_task");
// Lemlib setup

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftDrive, // left motor group
                              &rightDrive, // right motor group
                              TRACK_WIDTH,
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              300, // drivetrain rpm is 300 (double check)
                              2 // horizontal drift is 2 (for now)
);

// odometry settings

pros::Imu imu(IMU_PORT);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(HORIZONTAL_ROTATION_SENSOR_PORT);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 4);

// vertical tracking wheel encoder
pros::Rotation vertical_encoder(VERTICAL_ROTATION_SENSOR_PORT);
MotorGroup leftDriveEncoder({L_FRONT_PORT}, v5::MotorGears::green, v5::MotorUnits::rotations);
MotorGroup rightDriveEncoder({R_FRONT_PORT}, v5::MotorGears::green, v5::MotorUnits::rotations);

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0);
lemlib::TrackingWheel vertical_tracking_wheel_driveRight(&rightDriveEncoder, lemlib::Omniwheel::NEW_4, 6.5, 300);
lemlib::TrackingWheel vertical_tracking_wheel_driveLeft(&leftDriveEncoder, lemlib::Omniwheel::NEW_4, -6.5, 300);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor

/*
lemlib::OdomSensors sensors(&vertical_tracking_wheel_driveLeft, // vertical tracking wheel 1
                            &vertical_tracking_wheel_driveRight, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
*/
);

// angular PID controller
// NN - 
/*
lemlib::ControllerSettings angular_controller(5.5, // proportional gain (kP)5.5
                                              0, // integral gain (kI)
                                              43.79, // derivative gain (kD)51.73
                                              0, // anti windup3
                                              .75, // small error range, in degrees 1
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::ControllerSettings lateral_controller(4.8, // proportional gain (kP)16.1
                                              0, // integral gain (kI)
                                              4, // derivative gain (kD)72
                                              3, // anti windup
                                              1, // small error range, in inches
                                              500, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              35 // maximum acceleration (slew)
);
*/
/// Neev 
lemlib::ControllerSettings angular_controller(5.5, // proportional gain (kP)5.5
                                              0, // integral gain (kI)0
                                              57.63, // derivative gain (kD)51.73-57.63
                                              3, // anti windup3
                                              1, // small error range1
                                              100, // small error range timeout, in milliseconds100
                                              3, // large error range, in degrees3
                                              500, // large error range timeout, in milliseconds500
                                              0 // maximum acceleration (slew)0
);
lemlib::ControllerSettings lateral_controller(7.441, // proportional gain (kP)4.8
                                              0, // integral gain (kI)
                                              58.33, // derivative gain (kD)4;72.541
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);


// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

// ==================== PROS LIFECYCLE ====================
void initialize() {
  lcd::initialize();
  chassis.calibrate(); // calibrate sensors

  leftDrive.set_brake_mode(E_MOTOR_BRAKE_COAST);
  rightDrive.set_brake_mode(E_MOTOR_BRAKE_COAST);

  // Zero encoders (optional)
  leftDrive.tare_position_all();
  rightDrive.tare_position_all();
  outerTowerMiddleMotor.tare_position();
  innerTowerMiddleTopMotor.tare_position();
  innerTowerLowerMotor.tare_position();

  // Pneumatic default: retracted
  piston.set_value(false);
  piston_extended = false;
  piston_descore.set_value(false);
  piston_extended_descore = false;
  // to debug for autonomous
  pros::Task screenTask([&]() {
    while (true) {
    pros::lcd::print(0, "x: %f", chassis.getPose().x);
    pros::lcd::print(1, "y: %f", chassis.getPose().y);
    pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
    pros::lcd::print(3, "InOutMode: %d", (int)inOutMech.mode());
    pros::delay(50);
    }

  });
}

void disabled() {}
void competition_initialize() {}

void purePursuitSkills() {
  chassis.setPose(-48.356, -18.208, 180);
  inOutMech.set_mode(InOutMechanism::Mode::InLowStorage);
  chassis.follow(AutonRunRightRedLGCluster_txt, 15, 30000);
  // inOutMech.set_mode(InOutMechanism::Mode::InLowStorageOff);
  // inOutMech.set_mode(InOutMechanism::Mode::OutLowGoal);
  // delay(2000);
  // inOutMech.set_mode(InOutMechanism::Mode::OutLowGoalOff);
  // delay(2000);
  // chassis.follow(backtomatchload_txt,15,3000,false);
  // inOutMech.set_mode(InOutMechanism::Mode::InTopStorage);
  // chassis.follow(Intomatchloaderfinal_txt,10,1000);
  // delay(2000);
  // inOutMech.set_mode(InOutMechanism::Mode::InTopStorageOff);
  // chassis.follow(ToLonggoalRightBlue_txt,10,2000,false);
  // inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
  // delay(3000);
  // inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
}

void rightAuton () {
  chassis.setPose(0, 0, 0);
  // get balls near middle goal
  inOutMech.set_mode(InOutMechanism::Mode::InLowStorage);
  chassis.turnToHeading(-74,1000);
  // chassis.moveToPose(-35, 9, -74, 4000,{.maxSpeed = 40});
  chassis.moveToPose(-29, 8, -74, 4000,{.maxSpeed = 50}, false);
  delay(100);
  chassis.turnToHeading(-140, 1000);
  inOutMech.set_mode(InOutMechanism::Mode::InLowStorageOff);
  delay(100);
  chassis.moveToPoint(-34.5, 1, 4000, {.maxSpeed = 80}, false);
  // chassis.moveToPoint(34.5, .2, 2000, false);
  pros::delay(100);
  inOutMech.set_mode(InOutMechanism::Mode::OutLowGoal);
  pros::delay(1000);
  inOutMech.set_mode(InOutMechanism::Mode::OutLowGoalOff);
  // chassis.moveToPoint(-10, 23, 4000,{.forwards=false,.maxSpeed = 80});

  // Copied from left !
  // Go towards space between long goal and match loader 

  // chassis.moveToPose(-10, 31, -140, 5000,{.forwards=false,.maxSpeed=50});
  chassis.moveToPose(-10, 31, -140, 5000, {.forwards=false,.maxSpeed=80, .minSpeed = 64});
  
  // Get objects from match loader
  chassis.turnToHeading(-270, 4000);
  piston.set_value(true);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorage);
  chassis.moveToPoint(7.3, 31.4, 2000,{.maxSpeed = 50}, false);
  delay(1300);
  chassis.moveToPose(-19, 33, -270, 2000, {.forwards=false, .maxSpeed  = 80}, false);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorageOff);
  piston.set_value(false);
  // outtake
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
  delay(2000);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
  delay(1000);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
  inOutMech.set_mode(InOutMechanism::Mode::Off);
}

void leftAuton() {
   chassis.setPose(0, 0, 0);
  // get balls near middle goal
  inOutMech.set_mode(InOutMechanism::Mode::InLowStorage);
  chassis.turnToHeading(74,1000);
  // chassis.moveToPose(26.7,5.5,74,4000,{.maxSpeed = 40});
  chassis.moveToPose(29,8,74,4000,{.maxSpeed = 50}, false);
  delay(100);
  chassis.turnToHeading(140, 2000);
  inOutMech.set_mode(InOutMechanism::Mode::InLowStorageOff);
  piston.set_value(true);
  chassis.moveToPoint(34.5, .2, 2000,{.maxSpeed = 50}, false);
  delay(100);
  inOutMech.set_mode(InOutMechanism::Mode::OutMiddleGoal);
  pros::delay(2000);
  inOutMech.set_mode(InOutMechanism::Mode::OutMiddleGoalOff);
  // Ram the balls in
  // chassis.moveToPoint(37, -1.75,2000,{.maxSpeed = 80}, false);
  chassis.moveToPose(10, 31, 140, 5000,{.forwards=false,.maxSpeed=65});
  // Get objects from match loader
  chassis.turnToHeading(270, 4000);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorage);
  chassis.moveToPoint(-7.3, 32, 2000,{.maxSpeed = 50}, false);
  delay(1000);
  chassis.moveToPose(19, 33, 270, 5000,{.forwards=false, .maxSpeed = 50}, false);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorageOff);
  piston.set_value(false);
  // outtake to goal
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
  delay(2000);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
  delay(500);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
  delay(1000);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
  inOutMech.set_mode(InOutMechanism::Mode::Off);
}

// Run this for sure 20 points on skills
void skills20 () {
  // starting positions is the middle of left back wheel aligned
  // with the boundary of the parking zone color and the black connector
  // Go forward
  chassis.setPose(0, 0, 0);
  chassis.arcade(30,0);
  // chassis.waitUntil(10);
  delay(1000);
  // Go backward
  chassis.arcade(-60,0);
  delay(3000);
  chassis.arcade(0,0);
}
// ==================== Drive To Point (Heading Locked - Simplified) ====================
/*
 * Drives to a point while maintaining a fixed heading.
 *
 * targetX, targetY  -> field coordinates (inches)
 * desiredAngleDeg   -> heading to maintain (degrees)
 * kP                -> forward proportional gain
 * maxSpeed          -> max motor command (0-127)
 * coeff             -> forward scaling multiplier (0-1)
 * timeoutMs         -> max runtime in ms
 */
// void IMUStraightAdvanced(double targetDistance,
//                          double targetHeading,
//                          int maxVoltage,
//                          int timeoutMs)
// {
//     // ===== TUNING =====
//     const double kP_dist = 905.0;     // mV per inch (tune)900
//     const double kP_ang  = 94.0;      // mV per degree 80
//     const double kD_ang  = 290.0;     // damping 300
//     const double kP_x    = 700.0;     // mV per inch of lateral drift
//     const double kS      = 200.0;     // static friction compensation (mV)

//     const int loopMs = 10;
//     const double dt = loopMs / 1000.0;

//     int startTime = pros::millis();

//     vertical_tracking_wheel.reset();
//     horizontal_tracking_wheel.reset();

//     double lastHeadingError = 0;

//     while (true)
//     {
//         if (pros::millis() - startTime > timeoutMs)
//             break;

//         // ===== DISTANCE =====
//         double dist = vertical_tracking_wheel.getDistanceTraveled();
//         double distError = targetDistance - dist;

//         if (fabs(distError) < 0.2)
//             break;

//         double forwardVoltage = kP_dist * distError;

//         // Clamp forward voltage
//         if (forwardVoltage > maxVoltage) forwardVoltage = maxVoltage;
//         if (forwardVoltage < -maxVoltage) forwardVoltage = -maxVoltage;

//         // ===== HEADING ERROR (MANUAL NORMALIZATION) =====
//         double currentHeading = imu.get_rotation();
//         double headingError = targetHeading - currentHeading;

//         while (headingError > 180)  headingError -= 360;
//         while (headingError < -180) headingError += 360;

//         double headingDerivative = (headingError - lastHeadingError) / dt;

//         double imuTurn =
//             (kP_ang * headingError) +
//             (kD_ang * headingDerivative);

//         // Static friction compensation
//         if (headingError > 0)
//             imuTurn += kS;
//         else if (headingError < 0)
//             imuTurn -= kS;

//         lastHeadingError = headingError;

//         // // ===== LATERAL DRIFT CORRECTION =====
//         // double xDrift = horizontal_tracking_wheel.getDistanceTraveled();
//         // double xTurn = -kP_x * xDrift;

//         // ===== COMBINE =====
//         // double totalTurn = imuTurn + xTurn;
//         // Combine IMU + lateral
//         double totalTurn = imuTurn;

//         // Scale turn authority based on forward command magnitude
//         double speedRatio = fabs(forwardVoltage) / maxVoltage;

//         // prevent zeroing out turn completely
//         if (speedRatio < 0.2)
//             speedRatio = 0.2;

//         totalTurn *= speedRatio;
        
//         double leftVoltage  = forwardVoltage + totalTurn;
//         double rightVoltage = forwardVoltage - totalTurn;

//         // Clamp final outputs
//         if (leftVoltage > 12000) leftVoltage = 12000;
//         if (leftVoltage < -12000) leftVoltage = -12000;
//         if (rightVoltage > 12000) rightVoltage = 12000;
//         if (rightVoltage < -12000) rightVoltage = -12000;

//         leftDrive.move_voltage(leftVoltage);
//         rightDrive.move_voltage(rightVoltage);

//         pros::delay(loopMs);
//     }

//     leftDrive.move_voltage(0);
//     rightDrive.move_voltage(0);
// }
// // ==================== IMUStraight (Vertical Encoder Version) ====================
// /*
//  * Move forward a set distance using vertical tracking wheel
//  * and IMU heading correction.
//  *
//  * Parameters:
//  *  targetDistance - distance to travel (in inches)
//  *  targetAngle    - heading in degrees to maintain
//  *  maxSpeed       - maximum forward speed (0-127)
//  *  coeff          - heading correction coefficient
//  *  timeoutMs      - max runtime in ms
//  */
// // void IMUStraight(double targetDistance, double targetAngle, int maxSpeed, double coeff, int timeoutMs) {

// //     int startTime = pros::millis();
// //     int maxPower = 127;
// //     // Reset tracking wheel distance
// //     chassis.setPose(0,0,0);   // or verticalEncoder.reset_position();
    
// //     while (true) {

// //         // Distance traveled from vertical encoder
// //         double distanceTraveled = vertical_tracking_wheel.getDistanceTraveled();

// //         // Stop condition
// //         if (fabs(distanceTraveled) >= fabs(targetDistance) ||
// //             (pros::millis() - startTime) > timeoutMs)
// //             break;

// //         // Get IMU heading
// //         double currentAngle = imu.get_rotation();

// //         // Calculate heading error
// //         double angleError = targetAngle - currentAngle;
// //         if (angleError > 180) angleError -= 360;
// //         if (angleError < -180) angleError += 360;
// //         std::cout<<angleError<<std::endl;

// //         int correction = clamp127((int)(angleError * coeff));
        
// //         int leftPower  = clamp127(maxSpeed + correction);
// //         int rightPower = clamp127(maxSpeed - correction);
// //         // int leftVoltage = (forwardSpeed + correction);
// //         // int rightVoltage = (forwardSpeed - correction);

// //         leftDrive.move(leftPower);
// //         rightDrive.move(rightPower);
// //         // leftDrive.move_voltage(clamp127(leftPower)*1000);
// //         // rightDrive.move_voltage(clamp127(rightPower)*1000);

// //         pros::delay(10);
// //     }

// //     leftDrive.move(0);
// //     rightDrive.move(0);
// // }

void skillsRight () {
  chassis.setPose(0, 0, 0);
  chassis.moveToPose(0,33,0, 5000,{.maxSpeed=50});
  chassis.turnToHeading(90, 1000);
  piston.set_value(true);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorage);
  chassis.moveToPoint(10.3, chassis.getPose().y, 5000,{.maxSpeed=50});
  delay(2000);
  chassis.moveToPoint(0, chassis.getPose().y, 5000,{.forwards=false,.maxSpeed=50});
  piston.set_value(false);
  // This code to go in front of goal through middle
  chassis.turnToHeading(225, 5000);
  delay(500);
  chassis.moveToPose(-21.5, 9.5, 225, 5000,{.maxSpeed=50});
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorageOff);
  delay(500);
  chassis.turnToHeading(270, 2000);
  delay(500);
  // 95.6, 10.5
  chassis.moveToPose(-96.5, 10.5, 270, 5000,{.maxSpeed=50});
  delay(500);
  // 94.5, 10.05, -180.06
  chassis.turnToHeading(180, 1000);
  delay(500);
  chassis.moveToPoint(-94.75, 36, 5000,{.forwards=false,.maxSpeed=50});
  delay(500);
  chassis.turnToHeading(270, 1000);
  delay(500);
  chassis.moveToPose(-79.7, 35.5, 270, 3000, {.forwards=false,.maxSpeed=50},false);
  // out take 
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
  delay(3000);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
  delay(500);
  // Move towards match loader and get objects
  
  piston.set_value(true);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorage);
  chassis.moveToPoint(-108, 35.5, 5000,{.maxSpeed=50},false);
  delay(3000);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorageOff);

  // Now move back to goal and score
  chassis.moveToPoint(-80.5, 35.5, 5000,{.forwards=false,.maxSpeed=50},false);
  piston.set_value(false);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
  delay(2000);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
  delay(500);
  chassis.moveToPose(-94.75, 35.5, 270, 3000, {.maxSpeed=50},false);
  chassis.turnToHeading(180, 1000);
  chassis.moveToPose(-92.89, -67.21, 180, 10000,{.maxSpeed=50});
  // get objects from match loader
  chassis.turnToHeading(270, 1000);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorage);
  piston.set_value(true);
  chassis.moveToPoint(-105.89, -67.21, 2000,{.maxSpeed=50});
  delay(2000);
  // come towards goal
  chassis.moveToPoint(-79.89, -67.21,2000,{.maxSpeed=50});
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
  delay(3000);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
  delay(500);

  // come back to position between loader and long goal
  chassis.moveToPose(-92.89, -67.21, 180, 1000,{.maxSpeed=50});
  // // come back to parking
  // copy code from last skills code / skills 20 code - should be exact
}
// skills code starting bot to the left

void skillsLeft () {
  
  chassis.setPose(0, 0, 0);
  chassis.moveToPose(0,33,0,2000,{.maxSpeed=50});
  chassis.turnToHeading(-90, 1000);
  piston.set_value(true);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorage);
  chassis.moveToPoint(-10.3, chassis.getPose().y, 5000,{.maxSpeed=50});
  delay(3000);
  chassis.moveToPoint(0, chassis.getPose().y, 5000,{.forwards=false,.maxSpeed=50});
  piston.set_value(false);
  // This code to go in front of goal through middle
  chassis.turnToHeading(-225, 5000);
  delay(500);
  chassis.moveToPose(21.5,9.5,-225,5000,{.maxSpeed=50});
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorageOff);
  delay(500);
  chassis.turnToHeading(-270, 2000);
  delay(500);
  // 95.6, 10.5
  chassis.moveToPose(96.5, 10.5, -270, 5000,{.maxSpeed=50});
  delay(500);
  // 94.5, 10.05, -180.06
  chassis.turnToHeading(-180, 1000);
  delay(500);
  chassis.moveToPoint(94.75, 36, 5000,{.forwards=false,.maxSpeed=50});
  delay(500);
  chassis.turnToHeading(-270, 1000);
  delay(500);
  chassis.moveToPose(79.7, 35.5, -270, 3000, {.forwards=false,.maxSpeed=50},false);
  // out take 
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
  delay(3000);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
  delay(500);
  // Move towards match loader and get objects
  
  piston.set_value(true);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorage);
  chassis.moveToPoint(108, 35.5, 5000,{.maxSpeed=50},false);
  delay(3000);
  inOutMech.set_mode(InOutMechanism::Mode::InTopStorageOff);

  // Now move back to goal and score
  chassis.moveToPoint(80.5, 35.5, 5000,{.forwards=false,.maxSpeed=50},false);
  piston.set_value(false);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
  delay(2000);
  inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
  delay(500);
  // // come back to parking
  // chassis.moveToPoint(94.75, 36, 5000, {.maxSpeed=50});
  // chassis.turnToHeading(-180, 1000);
  // chassis.moveToPoint(94.75, 10.5, 5000,{.maxSpeed=50});
  // chassis.turnToHeading(-90, 1000);
  // chassis.moveToPose(-10.5, 10.5, -90, 10000,{.maxSpeed=50});
}
// --------- ============================AUTONOMOUS==================== ---------
void autonomous() {
  // skills();
  // skillsLeft();
  // leftAuton();
  // rightAuton();
  // skills20();
  // AutonAuton();
}
  

// --------- Driver Control ---------
void opcontrol() {
  // inOutMech.set_mode(InOutMechanism::Mode::Off);

  // leftAuton();
  // rightAuton();
  // skillsLeft();
  skillsRight();
  // skillsLeft();
  
  while (true) {
    
    if (master.get_digital_new_press(BTN_BRAKE_HOLD)) {
      leftDrive.set_brake_mode_all(MotorBrake::hold);
      rightDrive.set_brake_mode_all(MotorBrake::hold);
      //lcd::set_text(4, "Drive Brake: HOLD");
    }
    if (master.get_digital_new_press(BTN_BRAKE_COAST)) {
      leftDrive.set_brake_mode_all(MotorBrake::coast);
      rightDrive.set_brake_mode_all(MotorBrake::coast);
      //lcd::set_text(4, "Drive Brake: COAST");
    }
      //Drive reverse quick toggle
    if (master.get_digital_new_press(BTN_DriveReverse)) {
    driveReversed = !driveReversed;
    //lcd::set_text(4, driveReversed ? "Drive: REVERSED" : "Drive: NORMAL");
    }

    // ----- Pneumatics: toggle extend/retract -----
    if (master.get_digital_new_press(BTN_PISTON_TOGGLE)) {
      piston_extended = !piston_extended;
      piston.set_value(piston_extended);  // true = extend, false = retract
    }
    // ----- Pneumatics: toggle extend/retract -----
    if (master.get_digital_new_press(BTN_PISTON_TOGGLE_DESCORE)) {
      piston_extended_descore = !piston_extended_descore;
      piston_descore.set_value(piston_extended_descore);  // true = extend, false = retract
    }

// InOutMechanism Mode selection
    if (master.get_digital(BTN_INTOPSTORAGE)) {
      inOutMech.set_mode(InOutMechanism::Mode::InTopStorage);
      //lcd::set_text(4, "InOutMechanism Mode: InTopStorage");
    }
    if (master.get_digital_new_release(BTN_INTOPSTORAGE)) {
      inOutMech.set_mode(InOutMechanism::Mode::InTopStorageOff);
      //lcd::set_text(4, "InTopStorage: Off");
    }
    if (master.get_digital(BTN_INLOWSTORAGE)) {
      inOutMech.set_mode(InOutMechanism::Mode::InLowStorage);
      //lcd::set_text(4, "InOutMechanism Mode: InLowStorage");
    }
    if (master.get_digital_new_release(BTN_INLOWSTORAGE)) {
      // Stop all top storage motors
      inOutMech.set_mode(InOutMechanism::Mode::InLowStorageOff);
      //lcd::set_text(4, "InLowStorage: off");
    }

    if (master.get_digital(BTN_OUTMIDGOAL)) {
      inOutMech.set_mode(InOutMechanism::Mode::OutMiddleGoal);
      //lcd::set_text(4, "InOutMechanism Mode: OutMiddleGoal");
    }

    if (master.get_digital_new_release(BTN_OUTMIDGOAL)) {
      inOutMech.set_mode(InOutMechanism::Mode::OutMiddleGoalOff);
      //lcd::set_text(4, "OutMiddleGoal: off");
    }
    if (master.get_digital(BTN_OUTLOWGOAL)) {
      inOutMech.set_mode(InOutMechanism::Mode::OutLowGoal);
      //lcd::set_text(4, "InOutMechanism Mode: OutLowGoal");
    }
    if (master.get_digital_new_release(BTN_OUTLOWGOAL)) {
      inOutMech.set_mode(InOutMechanism::Mode::OutLowGoalOff);
      //lcd::set_text(4, "OutLowGoal off");
    }
    if (master.get_digital(BTN_midToTopStorage)) {
      inOutMech.set_mode(InOutMechanism::Mode::MidToTopStorage);
      //lcd::set_text(4, "InOutMechanism Mode: MidToTopStorage");
    }
    if (master.get_digital_new_release(BTN_midToTopStorage)) {
      inOutMech.set_mode(InOutMechanism::Mode::MidToTopStorageOff);
      //lcd::set_text(4, "MidToTopStorage off");
    }
    if (master.get_digital(BTN_TOPOUTTAKE)) {
      inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
      //lcd::set_text(4, "InOutMechanism Mode: OutTopGoal");
    }
    
    if (master.get_digital_new_release(BTN_TOPOUTTAKE)) {
      inOutMech.set_mode(InOutMechanism::Mode::OutTopGoalOff);
      //lcd::set_text(4, "OutTopGoal: Off");
    }

    if (master.get_digital_new_press(BTN_InOutMechanism_OFF)) {
      inOutMech.set_mode(InOutMechanism::Mode::Off);
      //lcd::set_text(4, "InOutMechanism Mode: Off");
    }

    // Drive input
    int leftY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    int leftX = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
    int rightY = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
    int rightX = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
    if (driveReversed) {
      // chassis.moveToPose(0,0,0,10000,{.forwards=false});//odom test
      // leftY = -leftY;   // invert forward/back
      if (leftX > 30) leftX = 30; //slow func
      if (leftX < -30) leftX = -30;
      if (leftY > 60) leftY = 60;
      if (leftY < -60) leftY = -60;
    }

    
/*
    LY = expoCmd(applyDeadband(LY));
    RY = expoCmd(applyDeadband(RY));
    LX = expoCmd(applyDeadband(LX));

    if (kArcadeDrive) driveArcade(LY, LX);
    else              driveTank(LY, RY);
*/
  // Lemlib options
  // move the robot
    // chassis.arcade(leftY, leftX); // Single Stick Arcade
    // chassis.arcade(leftY, rightX); // Double Stick Arcade
    // chassis.arcade(leftY, rightX, .9); // Double Stick Arcade
    chassis.arcade(leftY, leftX, false,.9); // prioritize steering slightly main.3
    // chassis.curvature(leftY, leftX); // Single stick curvature
    // chassis.curvature(leftY,rightX);
    // printf("Opcontrol state before delay: %d\n", pros::Task::current().get_state());
    delay(kDriveLoopMs);
    // printf("Opcontrol state after delay: %d\n", pros::Task::current().get_state());
  }
}