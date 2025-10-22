// src/main.cpp
#include "main.h"
#include <cmath>
using namespace pros;

// ==================== CONFIG (EDIT THESE) ====================

// The ports follow the notion that pressing forward button on the brain, if 
// the motor moves clockwise then its positive.clockwise direction is
// considered positive direction for that motor.

constexpr int8_t L_FRONT_PORT = -20;
constexpr int8_t L_BACK_PORT  = -19;
constexpr int8_t R_FRONT_PORT = 7;
constexpr int8_t R_BACK_PORT  = 9;

// Odometry ports
constexpr int8_t IMU_PORT = 16;
constexpr int8_t VERTICAL_ROTATION_SENSOR_PORT = 8;
// constexpr int8_t HORIZONTAL_ROTATION_SENSOR_PORT = 16;

// InOutMechanism ports

// Intake ports
constexpr int8_t OUTER_TOWER_MIDDLE_InOutMechanism_PORT      = -12;
constexpr int8_t INNER_TOWER_MIDDLE_TOP_InOutMechanism_PORT  = -11;
constexpr int8_t INNER_TOWER_LOWER_InOutMechanism_PORT       = 13;

// Outtake ports - todo
constexpr int8_t TOP_OUTTAKE_PORT = 14;

// Pneumatics (3-wire/ADI) port
constexpr char PISTON_PORT = 'H';  // Change to your 3-wire port (A..H)

// Drive style
constexpr bool kArcadeDrive = true;

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
constexpr auto BTN_BRAKE_HOLD   = E_CONTROLLER_DIGITAL_A;
constexpr auto BTN_BRAKE_COAST  = E_CONTROLLER_DIGITAL_B;
constexpr auto BTN_INTOPSTORAGE = E_CONTROLLER_DIGITAL_R1;
constexpr auto BTN_INLOWSTORAGE = E_CONTROLLER_DIGITAL_R2;
constexpr auto BTN_OUTMIDGOAL   = E_CONTROLLER_DIGITAL_L1;
constexpr auto BTN_OUTLOWGOAL   = E_CONTROLLER_DIGITAL_L2;
constexpr auto BTN_InOutMechanism_OFF   = E_CONTROLLER_DIGITAL_X;
constexpr auto BTN_TOPOUTTAKE   = E_CONTROLLER_DIGITAL_Y;

// Pneumatic toggle button
constexpr auto BTN_PISTON_TOGGLE = E_CONTROLLER_DIGITAL_RIGHT;

// ==================== DEVICES ====================
Controller master(E_CONTROLLER_MASTER);

// Drive train motor objects
MotorGroup leftDrive({L_FRONT_PORT, L_BACK_PORT}, v5::MotorGears::green, v5::MotorUnits::rotations);
MotorGroup rightDrive({R_FRONT_PORT, R_BACK_PORT}, v5::MotorGears::green, v5::MotorUnits::rotations);

// InOutMechanism motors
Motor outerTowerMiddleMotor(OUTER_TOWER_MIDDLE_InOutMechanism_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
Motor innerTowerMiddleTopMotor(INNER_TOWER_MIDDLE_TOP_InOutMechanism_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
Motor innerTowerLowerMotor(INNER_TOWER_LOWER_InOutMechanism_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
// Outtake Motor
Motor topOutTakeMotor(TOP_OUTTAKE_PORT, pros::v5::MotorGears::blue, pros::v5::MotorUnits::rotations);

// ==================== PNEUMATICS ====================
pros::adi::Pneumatics piston(PISTON_PORT, false); // false = retracted at startup
static bool piston_extended = false;

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

static inline void arcadeToTank(int throttle, int turn, int &l, int &r) {
  l = clamp127(throttle + turn);
  r = clamp127(throttle - turn);
}

// ===== Smooth drive tuning =====
constexpr int kAccelSlewPerLoop = 6;   // units per 10ms (≈0.6 s 0→127)
constexpr int kDecelSlewPerLoop = 10;  // allow slightly faster braking
constexpr double kJoyEMA = 0.20;       // joystick low-pass (0..1), higher = snappier
constexpr double kTurnAtSpeedScale = 0.6; // reduce turn when moving fast (0..1)

// Simple joystick exponential moving average (EMA) filter
struct EMA {
  double a;  // smoothing factor (0..1), higher = snappier 
  double y{0.0};
  explicit EMA(double alpha): a(alpha) {}
  int step(int x) { y = a * x + (1.0 - a) * y; return (int)std::round(y); }
};

// Per-side slew limiter
struct Slew {
  int up, down;   // max step per loop when increasing/decreasing
  int y{0};
  Slew(int up_step, int down_step): up(up_step), down(down_step) {}
  int step(int target) {
    int diff = target - y;
    int lim  = (diff > 0) ? up : down;
    if (std::abs(diff) > lim) y += (diff > 0 ? lim : -lim);
    else y = target;
    return y;
  }
};

EMA emaY(kJoyEMA), emaX(kJoyEMA);
Slew leftSlew(kAccelSlewPerLoop, kDecelSlewPerLoop);
Slew rightSlew(kAccelSlewPerLoop, kDecelSlewPerLoop);


// InOutMechanism - This class handles all operations for intake and outtake from the robot
class InOutMechanism {
public:
  enum class Mode { Off, InTopStorage, InLowStorage, OutMiddleGoal, OutLowGoal, OutTopGoal};

  InOutMechanism(Motor& L1, Motor& L2, Motor& R1, Motor& T1)
  : m_outerTowerMiddleMotor(L1), m_innerTowerMiddleTopMotor(L2), m_innerTowerLowerMotor(R1),m_topGoalMotor(T1) {}

  void set_mode(Mode m) { mode_ = m; }
  Mode mode() const { return mode_; }

  // Call from a background task
  void update() {
    // TODO - ramp speeds instead of instant full voltage
    // TODO - Should we have the motors on 'hold' as opposed to on 'coast' to ensure that objects remain in place?
    switch (mode_) {
      case Mode::Off:
        m_outerTowerMiddleMotor.move(0);
        m_innerTowerMiddleTopMotor.move(0);
        m_innerTowerLowerMotor.move(0);
        m_topGoalMotor.move(0);
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
      case Mode::OutTopGoal:
        m_topGoalMotor.move(127);
        break;
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
  Motor& m_topGoalMotor;
  Mode mode_ = Mode::Off;
};

InOutMechanism inOutMech(outerTowerMiddleMotor, innerTowerMiddleTopMotor, innerTowerLowerMotor, topOutTakeMotor);

void InOutMechanismTaskFn(void*) {
  outerTowerMiddleMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  innerTowerMiddleTopMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  innerTowerLowerMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  topOutTakeMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  while (true) { inOutMech.update(); delay(kInOutMechanismLoopMs); }
}
Task InOutMechanismTask(InOutMechanismTaskFn, nullptr, "InOutMechanism_task");

// ==================== PROS LIFECYCLE ====================
void initialize() {
  lcd::initialize();
  lcd::set_text(1, "Drive+Intake+Outtake+Piston Ready");

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
}

void disabled() {}
void competition_initialize() {}

// --------- Autonomous (example) ---------
void autonomous() {
  // InOutMechanism in, drive forward, then hold
  inOutMech.set_mode(InOutMechanism::Mode::InLowStorage);
  driveTank(80, 80);
  delay(1000);

  driveTank(0, 0);
  inOutMech.set_mode(InOutMechanism::Mode::Off);
  delay(200);

  // small turn example
  driveTank(60, -60);
  delay(300);
  driveTank(0, 0);
}

// --------- Driver Control ---------
void opcontrol() {
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

    // ----- Pneumatics: toggle extend/retract -----
    if (master.get_digital_new_press(BTN_PISTON_TOGGLE)) {
      piston_extended = !piston_extended;
      piston.set_value(piston_extended);  // true = extend, false = retract
      lcd::set_text(3, piston_extended ? "Piston: EXTENDED" : "Piston: RETRACTED");
    }

    // InOutMechanism Mode selection
    if (master.get_digital_new_press(BTN_INTOPSTORAGE)) {
      inOutMech.set_mode(InOutMechanism::Mode::InTopStorage);
      lcd::set_text(2, "InOutMechanism Mode: InTopStorage");
    }
    if (master.get_digital_new_press(BTN_INLOWSTORAGE)) {
      inOutMech.set_mode(InOutMechanism::Mode::InLowStorage);
      lcd::set_text(2, "InOutMechanism Mode: InLowStorage");
    }
    if (master.get_digital_new_press(BTN_OUTMIDGOAL)) {
      inOutMech.set_mode(InOutMechanism::Mode::OutMiddleGoal);
      lcd::set_text(2, "InOutMechanism Mode: OutMiddleGoal");
    }
    if (master.get_digital_new_press(BTN_OUTLOWGOAL)) {
      inOutMech.set_mode(InOutMechanism::Mode::OutLowGoal);
      lcd::set_text(2, "InOutMechanism Mode: OutLowGoal");
    }
    if (master.get_digital_new_press(BTN_InOutMechanism_OFF)) {
      inOutMech.set_mode(InOutMechanism::Mode::Off);
      lcd::set_text(2, "InOutMechanism Mode: Off");
    }
    // No code defined yet for outtake
    if (master.get_digital_new_press(BTN_TOPOUTTAKE)) {
      inOutMech.set_mode(InOutMechanism::Mode::OutTopGoal);
      lcd::set_text(2, "InOutMechanism Mode: OutTopGoal");
    }
/*
    // Drive input
    int LY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    int RY = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
    int LX = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);

    LY = expoCmd(applyDeadband(LY));
    RY = expoCmd(applyDeadband(RY));
    LX = expoCmd(applyDeadband(LX));

    if (kArcadeDrive) driveArcade(LY, LX);
    else              driveTank(LY, RY);
*/
    // ema + slew limit version:

    // -------- Smooth Drive Input --------
    int rawLY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    int rawLX = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);

    // Deadband → expo → EMA (order matters; EMA last for smoothness)
    int LY = expoCmd(applyDeadband(rawLY));
    int LX = expoCmd(applyDeadband(rawLX));
    LY = emaY.step(LY);
    LX = emaX.step(LX);

    // Curvature: reduce turn as forward speed rises (keeps high-speed stable)
    double speedFrac = std::min(1.0, std::abs(LY) / 127.0);
    int scaledTurn = (int)std::round(LX * (1.0 - kTurnAtSpeedScale * speedFrac));

    // Mix to tank and apply per-side slew
    int l_tgt, r_tgt; arcadeToTank(LY, scaledTurn, l_tgt, r_tgt);
    int l_cmd = leftSlew.step(l_tgt);
    int r_cmd = rightSlew.step(r_tgt);

    driveTank(l_cmd, r_cmd);
    delay(kDriveLoopMs);
  }
}