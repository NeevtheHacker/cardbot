// src/main.cpp
#include "main.h"
#include <cmath>
using namespace pros;

// ==================== CONFIG (EDIT THESE) ====================
constexpr int8_t L_FRONT_PORT = -20;
constexpr int8_t L_BACK_PORT  = -19;
constexpr int8_t R_FRONT_PORT = 10;
constexpr int8_t R_BACK_PORT  = 9;

constexpr int8_t INTAKE_L_PORT = 11;
constexpr int8_t INTAKE_R_PORT = 2;

constexpr bool INTAKE_L_REV = false; // set so BOTH pull IN on Intake
constexpr bool INTAKE_R_REV = true;

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
constexpr auto BTN_BRAKE_HOLD  = E_CONTROLLER_DIGITAL_A;
constexpr auto BTN_BRAKE_COAST = E_CONTROLLER_DIGITAL_B;
constexpr auto BTN_INTAKE      = E_CONTROLLER_DIGITAL_R1;
constexpr auto BTN_OUTTAKE     = E_CONTROLLER_DIGITAL_R2;
constexpr auto BTN_HOLD        = E_CONTROLLER_DIGITAL_L1;

// ==================== DEVICES ====================
Controller master(E_CONTROLLER_MASTER);
// Drive train motor objects
MotorGroup leftDrive({L_FRONT_PORT, L_BACK_PORT}, v5::MotorGears::green, v5::MotorUnits::rotations);
MotorGroup rightDrive({R_FRONT_PORT, R_BACK_PORT}, v5::MotorGears::green, v5::MotorUnits::rotations);

// Intake motors
Motor leftIntakeMotor(INTAKE_L_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);
Motor rightIntakeMotor(INTAKE_R_PORT, pros::v5::MotorGears::green, pros::v5::MotorUnits::rotations);

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
  enum class Mode { Off, Hold, In, Out };

  Intake(Motor& L, Motor& R) : mL(L), mR(R) {}

  void set_mode(Mode m) {
    mode_ = m;
    switch (mode_) {
      case Mode::Off:  target_ = 0;            break;
      case Mode::Hold: target_ = kHoldCmd;     break;
      case Mode::In:   target_ =  kIntakeCmd;  break;
      case Mode::Out:  target_ = -kIntakeCmd;  break;
    }
  }

  Mode mode() const { return mode_; }

  // Call from a background task
  void update() {
    // ramp toward target
    const int step = 6;
    int d = target_ - output_;
    if (std::abs(d) <= step) output_ = target_;
    else output_ += (d > 0 ? step : -step);

    mL.move(output_);
    mR.move(output_);
  }

private:
  Motor& mL;
  Motor& mR;
  Mode mode_  = Mode::Off;
  int  target_ = 0;
  int  output_ = 0;
};

Intake intake(leftIntakeMotor, rightIntakeMotor);

void intakeTaskFn(void*) {
  leftIntakeMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
  rightIntakeMotor.set_brake_mode(E_MOTOR_BRAKE_COAST);
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
  leftIntakeMotor.tare_position();
  rightIntakeMotor.tare_position();
}

void disabled() {}
void competition_initialize() {}

// --------- Autonomous (example) ---------
void autonomous() {
  // Intake in, drive forward, then hold
  intake.set_mode(Intake::Mode::In);
  driveTank(80, 80);
  delay(1000);

  driveTank(0, 0);
  intake.set_mode(Intake::Mode::Hold);
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

    // Intake modes
    if (master.get_digital(BTN_INTAKE)) {
      intake.set_mode(Intake::Mode::In);
      lcd::set_text(2, "Intake Mode: In");
    } else if (master.get_digital(BTN_OUTTAKE)) {
      intake.set_mode(Intake::Mode::Out);
      lcd::set_text(2, "Intake Mode: Out");
    } else if (master.get_digital(BTN_HOLD)) {
      intake.set_mode(Intake::Mode::Hold);
      lcd::set_text(2, "intake Mode: HOLD");
    } else {
      intake.set_mode(Intake::Mode::Off);
      lcd::set_text(2, "Intake Mode: OFF");
    }
      

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