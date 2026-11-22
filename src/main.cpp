#include "main.h" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <new>

// ----------- INIT PORTS FOR ROBOT MODULES ----------- //
// drivetrain port constants (negative = reversed)
constexpr int LDT_MOTOR_F = -2; // left front drive motor
constexpr int LDT_MOTOR_B = 4;  // left back/middle drive motor
constexpr int LDT_MOTOR_U = -6; // left upper drive motor
constexpr int RDT_MOTOR_F = 3;  // right front drive motor
constexpr int RDT_MOTOR_B = 5;  // right back/middle drive motor
constexpr int RDT_MOTOR_U = -7; // right upper drive motor

// collector & cascade lift port constants
constexpr int FIRST_COLLECTOR_PORT = 16;  // front bottom collector
constexpr int SECOND_COLLECTOR_PORT = 12; // back collector
constexpr int THIRD_COLLECTOR_PORT = -11; // front top collector (reversed)

// CASCADE LIFT MOTORS CHECK LIFT PORTS, MOTOR COLOR, AND IF THEY ARE REVERSED OR NOT. (CHECK)
constexpr int LIFT_MOTOR_L_PORT = -8;     // left cascade lift motor (reversed)
constexpr int LIFT_MOTOR_R_PORT = 9;      // right cascade lift motor

// sensor port constants
constexpr int IMU_PORT = 10;           // inertial sensor
constexpr int HORIZONTAL_ENC_PORT = 20; // horizontal tracking wheel
constexpr int VERTICAL_ENC_PORT = -9;   // vertical tracking wheel (reversed)

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({LDT_MOTOR_F, LDT_MOTOR_B, LDT_MOTOR_U}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({RDT_MOTOR_F, RDT_MOTOR_B, RDT_MOTOR_U}, pros::MotorGearset::blue);

// collectors
pros::Motor FirstCollector(FIRST_COLLECTOR_PORT, pros::MotorGearset::blue); // front bottom collector
pros::Motor SecondCollector(SECOND_COLLECTOR_PORT, pros::MotorGearset::blue);  // back collector
pros::Motor ThirdCollector(THIRD_COLLECTOR_PORT, pros::MotorGearset::blue);  // front top collector

// cascade lift 
pros::MotorGroup cascadeLiftMotors({LIFT_MOTOR_L_PORT, LIFT_MOTOR_R_PORT}, pros::MotorGearset::green);


// sensors
pros::Imu imu(IMU_PORT);

// pneumatics (N/A)
//pros::adi::Pneumatics wing('E', false);
//pros::adi::Pneumatics feeder('G', false);
//pros::adi::Pneumatics stopper('H', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.5, lemlib::Omniwheel::NEW_275, 450, 2);

// controllers
lemlib::ControllerSettings linearController(11, // proportional gain (kP)
//TUNE THIS kP (1-mat / 24" tape test)
                                              0, // integral gain (kI)
                                              44, // derivative gain (kD)
//2. TUNE THIS kD (1-mat / 24" tape test)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(3, // proportional gain (kP) 3
//3. TUNE THIS kP (90-deg turn test)
                                              0, // integral gain (kI)
                                           10, // derivative gain (kD) 10
//4. TUNE THIS kP (90-deg turn test)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// tracking wheels
pros::Rotation horizontalEnc(HORIZONTAL_ENC_PORT);
pros::Rotation verticalEnc(VERTICAL_ENC_PORT);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0.75);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, -2.75);

// odom sensors
lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);

// drive curves
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);

// chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


// -------------------- QUICK CONFIGURATION -------------------- //
constexpr int side = 1; // 1 = right, -1 = left
//CHECK LIFT DEGREES PER INCH VALUE FOR YOUR ROBOT. THIS IS THE NUMBER OF MOTOR DEGREES PER 1 INCH OF LIFT TRAVEL. IT IS USED TO CONVERT INCHES TO MOTOR DEGREES FOR THE CASCADE LIFT. (CHECK)
constexpr double LIFT_DEG_PER_INCH = 45.0; // distance scale factor: motor degrees per 1 inch of lift travel
constexpr int TUNING_MODE = 2; // 0 = normal auton, 1 = 90-deg angular PID test, 2 = 1-mat (24 in) linear PID tape test

// -------------------- HELPER FUNCTIONS -------------------- //
/* Moves the cascade lift to a target height in inches during autonomous */
void cascadeLift(double targetInches, int velocityRPM = 200) {
    double targetDegrees = targetInches * LIFT_DEG_PER_INCH;
    cascadeLiftMotors.move_absolute(targetDegrees, velocityRPM);
}

/* Interactive controller PID tuner (fixed for LemLib 0.5.6: updates global chassis PID in-place) */
void auto_tune_pid(bool tuneLinear) {
    lemlib::ControllerSettings& activeSettings = tuneLinear ? linearController : angularController;
    while (true) {
        // rebuild the active PID inside the global chassis with updated kP and kD
        if (tuneLinear) {
            new (&chassis.lateralPID) lemlib::PID(
                linearController.kP, linearController.kI, linearController.kD, linearController.windupRange, false
            );
        } else {
            new (&chassis.angularPID) lemlib::PID(
                angularController.kP, angularController.kI, angularController.kD, angularController.windupRange, true
            );
        }
        std::printf("Testing %s PID (kP: %.2f, kD: %.2f)\n", tuneLinear ? "LINEAR" : "ANGULAR", activeSettings.kP, activeSettings.kD);
        chassis.setPose(0, 0, 0);
        if (tuneLinear) {
            chassis.moveToPoint(0, 24, 4000); // 1 VEX mat = 24 inches
        } else {
            chassis.turnToHeading(90, 4000);  // 90 degree turn test
        }
        chassis.waitUntilDone();
        chassis.cancelAllMotions();
        lemlib::Pose endPose = chassis.getPose();
        std::printf("Done! Final Pose -> X: %.2f, Y: %.2f, Theta: %.2f\n", endPose.x, endPose.y, endPose.theta);
        std::printf("D-Pad UP/DOWN: kP +/- 0.5 | X/B: kD +/- 1.0 | A: re-test | Y: exit tuner\n");
        // adjust kP and kD live from the controller
        while (true) {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                activeSettings.kP += 0.5f;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                activeSettings.kP -= 0.5f;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                activeSettings.kD += 1.0f;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                activeSettings.kD -= 1.0f;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
                break; // run next test with new gains
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                return; // exit back to driver control
            }
            pros::lcd::print(5, "%s kP:%.1f kD:%.1f", tuneLinear ? "LIN" : "ANG", activeSettings.kP, activeSettings.kD);
            pros::delay(20);
        }
    }
}

// -------------------- PROS Callbacks -------------------- //
/* Init code upon program being run */
bool logDebug = false;
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    // hold cascade lift in place against gravity and zero its encoders
    cascadeLiftMotors.set_brake_mode_all(pros::MotorBrake::hold);
    cascadeLiftMotors.tare_position_all();

    // IMPORTANT: make tasks static so they don't get destroyed when initialize() returns
    static pros::Task screenTask([] {
        while (true) {
            lemlib::Pose chass = chassis.getPose();
            pros::lcd::print(0, "X: %f", chass.x);
            pros::lcd::print(1, "Y: %f", chass.y); 
            pros::lcd::print(2, "Theta: %f", chass.theta); 

            if (logDebug) {
                std::printf(
                    "Chassis:\nX: %f\nY: %f\nTheta: %f\n",
                    chass.x,
                    chass.y,
                    chass.theta
                );
            }

            lemlib::telemetrySink()->info("Chassis pose: {}", chass);

            pros::delay(50);
        }
    });
}

/* Match end robot state */
void disabled() {

}

/* Init code that only runs in competition mode */
void competition_initialize() {}

/* Code that runs during autonomous period */
void autonomous() {
chassis.setPose(0, 0, 0);

    if (TUNING_MODE == 1) {
        // angular PID test: 90 degree clockwise turn (tune this FIRST before linear PID)
        chassis.turnToHeading(90, 4000);
        chassis.waitUntilDone();
    } else if (TUNING_MODE == 2) {
        // linear PID test: drive forward 1 VEX mat (24 inches) between two pieces of tape
        chassis.moveToPoint(0, 24, 4000);
        chassis.waitUntilDone();
    }
}

// -------------------- Driver Control -------------------- //
bool driveDirection = true; // default direction, brain side
int modifier = driveDirection ? 1 : -1;

/* Code that runs during driver control; The manual controls for the robot */
void opcontrol() {
    while (true) {
        int leftY = modifier*controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            driveDirection = !driveDirection;
            modifier = driveDirection ? 1 : -1;
        }
    // collector controls
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            FirstCollector.move(127);
            SecondCollector.move(127);
            ThirdCollector.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            FirstCollector.move(-127);
            SecondCollector.move(-127);
            ThirdCollector.move(-127);
        } else {
            FirstCollector.move(0);
            SecondCollector.move(0);
            ThirdCollector.move(0); 
        }
        // cascade lift controls
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            cascadeLiftMotors.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            cascadeLiftMotors.move(-127);
        } else {
            cascadeLiftMotors.brake();
        }
        // auto pid tuner triggers (hold X + A for angular tuner, or Y + A for 1-mat linear tuner)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            logDebug = false;
            auto_tune_pid(false);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
                   controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            logDebug = false;
            auto_tune_pid(true);
        }
        pros::delay(25);
    }
}
