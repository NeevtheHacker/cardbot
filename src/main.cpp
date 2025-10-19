// src/main.cpp
#include "main.h"
#include <cmath>
using namespace pros;

// ==================== CONFIG (EDIT THESE) ====================
constexpr int8_t L_FRONT_PORT = -20;
constexpr int8_t L_BACK_PORT  = -19;
constexpr int8_t R_FRONT_PORT = 10;
constexpr int8_t R_BACK_PORT  = 9;

// Odometry ports
constexpr int8_t IMU_PORT = 16;
constexpr int8_t VERTICAL_ROTATION_SENSOR_PORT = 8;
// constexpr int8_t HORIZONTAL_ROTATION_SENSOR_PORT = 16;

// Intake ports
//Clockwise Direction
constexpr int8_t OUTER_TOWER_MIDDLE_INTAKE_PORT = -12;
constexpr int8_t INNER_TOWER_MIDDLE_TOP_INTAKE_PORT = -11;
constexpr int8_t INNER_TOWER_LOWER_INTAKE_PORT = 13;

// Outtake ports - todo
constexpr int8_t OUTTAKE_PORT = 14;

// Drive style
constexpr bool kArcadeDrive = true;

// Shaping
constexpr int    kDeadband = 5;
constexpr double kExpo     = 1.6;
constexpr int    kMaxCmd   = 127;

// Loop rate
constexpr int kDriveLoopMs  = 10;
constexpr int kIntakeLoopMs = 10;

// Intake powers
constexpr int kIntakeCmd = 127;
constexpr int kHoldCmd   = 18;

// Buttons
constexpr auto BTN_BRAKE_HOLD   = E_CONTROLLER_DIGITAL_A;
constexpr auto BTN_BRAKE_COAST  = E_CONTROLLER_DIGITAL_B;
constexpr auto BTN_INTOPSTORAGE = E_CONTROLLER_DIGITAL_R1;
constexpr auto BTN_INLOWSTORAGE = E_CONTROLLER_DIGITAL_R2;
constexpr auto BTN_OUTMIDGOAL   = E_CONTROLLER_DIGITAL_L1;
constexpr auto BTN_OUTLOWGOAL   = E_CONTROLLER_DIGITAL_L2;
constexpr auto BTN_INTAKE_OFF   = E_CONTROLLER_DIGITAL_X;
constexpr auto BTN_OUTTAKE      = E_CONTROLLER_DIGITAL_Y;


// ==================== DEVICES ====================
Controller master(E_CONTROLLER_MASTER);
// Drive train motor objects
MotorGroup leftDrive({L_FRONT_PORT, L_BACK_PORT}, v5::MotorGears::green, v5::MotorUnits::rotations);
MotorGroup rightDrive({R_FRONT_PORT, R_BACK_PORT}, v5::MotorGears::green, v5::MotorUnits::rotations);

// Intake motors
Motor outerTowerMiddleMotor(OUTER_TOWER_MIDDLE_INTAKE_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
Motor innerTowerMiddleTopMotor(INNER_TOWER_MIDDLE_TOP_INTAKE_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
Motor innerTowerLowerMotor(INNER_TOWER_LOWER_INTAKE_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

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

// ==================== INTAKE SUBSYSTEM ====================
class Intake {
public:
  enum class Mode { Off, InTopStorage, InLowStorage, OutMiddleGoal, OutLowGoal};

  Intake(Motor& L1, Motor& L2, Motor& R1) : m_outerTowerMiddleMotor(L1),
                                            m_innerTowerMiddleTopMotor(L2), 
                                            m_innerTowerLowerMotor(R1) {}

  void set_mode(Mode m) {
    mode_ = m;
  }

  Mode mode() const { return mode_; }

  // Call from a background task
  void update() {
    // TODO - need to ramp up speed rather than just go with full voltage
    switch (mode_) {
      case Mode::Off: 
        m_outerTowerMiddleMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        m_innerTowerLowerMotor.move(0);
        break;
      case Mode::InTopStorage:
        m_outerTowerMiddleMotor.move(127);
        m_innerTowerLowerMotor.move(0);
        m_innerTowerMiddleTopMotor.move(-127);
        break;
      case Mode::InLowStorage:
        m_outerTowerMiddleMotor.move(127);
        m_innerTowerLowerMotor.move(0);
        m_innerTowerMiddleTopMotor.move(127);  
        break;
      case Mode::OutMiddleGoal:
      case Mode::OutLowGoal:
      default:
        m_outerTowerMiddleMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        m_innerTowerLowerMotor.move(0);
        break;
    }
  }

private:
  Motor& m_outerTowerMiddleMotor;
  Motor& m_innerTowerMiddleTopMotor;
  Motor& m_innerTowerLowerMotor;
  Mode mode_  = Mode::Off;
};

Intake intake(outerTowerMiddleMotor, innerTowerMiddleTopMotor, innerTowerLowerMotor);

void intakeTaskFn(void*) {
  outerTowerMiddleMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  innerTowerMiddleTopMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  innerTowerLowerMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  while (true) { intake.update(); delay(kIntakeLoopMs); }
}
Task intakeTask(intakeTaskFn, nullptr, "intake_task");

// ==================== PROS LIFECYCLE ====================
void initialize() {
  lcd::initialize();
  lcd::set_text(1, "Drive+Intake Ready");

  leftDrive.set_brake_mode(E_MOTOR_BRAKE_COAST);
  rightDrive.set_brake_mode(E_MOTOR_BRAKE_COAST);

  // Zero encoders (optional)
  leftDrive.tare_position_all();
  rightDrive.tare_position_all();
  outerTowerMiddleMotor.tare_position();
  innerTowerMiddleTopMotor.tare_position();
  innerTowerLowerMotor.tare_position();
}

void disabled() {}
void competition_initialize() {}

// --------- Autonomous (example) ---------
void autonomous() {
  // Intake in, drive forward, then hold
  intake.set_mode(Intake::Mode::InLowStorage);
  driveTank(80, 80);
  delay(1000);

  driveTank(0, 0);
  intake.set_mode(Intake::Mode::Off);
  delay(200);

  // small turn example
  driveTank(60, -60);
  delay(300);
  driveTank(0, 0);
}

// --------- Driver Control ---------
void opcontrol() {
  /* 
  intakeL1Toggle = false;
  intakeL2Toggle = false;
  intakeR1Toggle = false;
  */

  while (true) {
    // Brake mode quick toggle
    if (master.get_digital_new_press(BTN_BRAKE_HOLD)) {
      leftDrive.set_brake_mode_all(MotorBrake::hold);
      rightDrive.set_brake_mode_all(MotorBrake::hold);
      lcd::set_text(2, "Drive Brake: HOLD");
    }
    if (master.get_digital_new_press(BTN_BRAKE_COAST)) {
      leftDrive.set_brake_mode_all(MotorBrake::coast);
      rightDrive.set_brake_mode_all(MotorBrake::coast);
      lcd::set_text(2, "Drive Brake: COAST");
    }
    
/*
    // Intake Motors Test
    if (master.get_digital_new_press(BTN_INTAKE)) {
      intakeL1Toggle = !intakeL1Toggle;
      if (intakeL1Toggle) {
        innerTowerLowerMotor.move(127);
      } else {
        innerTowerLowerMotor.move(0);
      }
    }

    if (master.get_digital_new_press(BTN_OUTTAKE)) {
      intakeL2Toggle = !intakeL2Toggle;
      if (intakeL2Toggle) {
        innerTowerMiddleTopMotor.move(127);
      } else {
        innerTowerMiddleTopMotor.move(0);
      }
    }
    
    if (master.get_digital_new_press(BTN_HOLD)) {
      intakeR1Toggle = !intakeR1Toggle;
      if (intakeR1Toggle) {
        outerTowerMiddleMotor.move(-127);
      } else {
        outerTowerMiddleMotor.move(0);
      }
    }
    // Optional quick telemetry
    lcd::set_text(2, "innerTowerLowerMotor:" + std::string(intakeL1Toggle ? "ON " : "OFF") +
                     "innerTowerMiddleTopMotor:" + std::string(intakeL2Toggle ? "ON " : "OFF") +
                     "outerTowerMiddleMotor:" + std::string(intakeR1Toggle ? "ON" : "OFF"));
*/



constexpr auto BTN_INTAKE_OFF   = E_CONTROLLER_DIGITAL_X;
constexpr auto BTN_OUTTAKE      = E_CONTROLLER_DIGITAL_Y;


    // Intake Mode selection
    if (master.get_digital_new_press(BTN_INTOPSTORAGE)) {
      intake.set_mode(Intake::Mode::InTopStorage);
      lcd::set_text(2, "Intake Mode: InTopStorage");
    }

    if (master.get_digital_new_press(BTN_INLOWSTORAGE)) {
      intake.set_mode(Intake::Mode::InLowStorage);
      lcd::set_text(2, "Intake Mode: InLowStorage");
    }
    if (master.get_digital_new_press(BTN_OUTMIDGOAL)) {
      intake.set_mode(Intake::Mode::OutMiddleGoal);
      lcd::set_text(2, "Intake Mode: OutMiddleGoal");
    }

    if (master.get_digital_new_press(BTN_OUTLOWGOAL)) {
      intake.set_mode(Intake::Mode::OutLowGoal);
      lcd::set_text(2, "Intake Mode: OutLowGoal");
    }

    if (master.get_digital_new_press(BTN_INTAKE_OFF)) {
      intake.set_mode(Intake::Mode::Off);
      lcd::set_text(2, "Intake Mode: Off");
    }
    // No code defined yet for outtake

    // Drive input
    int LY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    int RY = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
    int LX = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);

    LY = expoCmd(applyDeadband(LY));
    RY = expoCmd(applyDeadband(RY));
    LX = expoCmd(applyDeadband(LX));

    if (kArcadeDrive) driveArcade(LY, LX);
    else              driveTank(LY, RY);

    delay(kDriveLoopMs);
  }
}