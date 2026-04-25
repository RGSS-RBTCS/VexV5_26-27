#include "main.h" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

// ----------- INIT PORTS FOR ROBOT MODULES ----------- //
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-2, 4, -6}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({3, 5, -7}, pros::MotorGearset::blue);

// collectors
pros::Motor FirstCollector(16, pros::MotorGearset::blue);   // front bottom collector
pros::Motor SecondCollector(12, pros::MotorGearset::blue);  // back collector
pros::Motor ThirdCollector(-11, pros::MotorGearset::blue);   // front top collector

// sensors
pros::Imu imu(10);

// pneumatics
pros::adi::Pneumatics wing('E', false);
pros::adi::Pneumatics feeder('G', false);
pros::adi::Pneumatics stopper('H', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12.5, lemlib::Omniwheel::NEW_275, 450, 2);
// controllers
lemlib::ControllerSettings linearController(11, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              44, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(3, // proportional gain (kP) 3
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD) 10
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// tracking wheels
pros::Rotation horizontalEnc(20);
pros::Rotation verticalEnc(-9);
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

// -------------------- PROS Callbacks -------------------- //
/* Init code upon program being run */
bool logDebug = false;
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    // IMPORTANT: make tasks static so they don't get destroyed when initialize() returns
    static pros::Task screenTask([] {
        while (true) {
            lemlib::Pose chass = chassis.getPose();
            pros::lcd::print(0, "X: %f", chass.x);
            pros::lcd::print(1, "Y: %f", chass.y); 
            pros::lcd::print(2, "Theta: %f", chass.theta); 

            if (logDebug) {
                std::printf(
                    "Chassis:\nX: %f\nY: %f\nTheta: %f\n\nAuto Intake: %s\n\nCollectors:\nFirst: %d\nSecond: %d\nThird: %d\n",
                    chass.x,
                    chass.y,
                    chass.theta,
                    FirstCollector.get_faults(),
                    SecondCollector.get_faults(),
                    ThirdCollector.get_faults()
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

void auto_tune_pid(lemlib::ControllerSettings movementController, bool linear, int margin, int OSCMargin) {
    logDebug = false;
    while (true) {
        std::printf("Testing (%f, %f)\n", angularController.kP, angularController.kD);
        chassis.setPose(0, 0, 0);
        if (linear) {
            chassis.moveToPoint(0, 24, 4999);
        } else {
            chassis.turnToHeading(180, 4999);
        }

        chassis.waitUntilDone();
        chassis.cancelAllMotions();
        std::printf("Change kP and kD accordingly. Left up and down for kP, X and B for kD, and A to finish.\nCurrent values: (%f, %f)\n",movementController.kP, movementController.kD);
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
                angularController.kP += 1;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                angularController.kP -= 1;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                angularController.kD += 1;
            }
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                angularController.kD -= 1;
            }
            pros::delay(20);
        }

        lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);
        
        std::printf("Press A on the controller to continue...\n");
        while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            pros::delay(20);
        }
        std::printf("----------------------------------\n");
    }
}

/* Code that runs during autonomous period */
void autonomous() {

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

        // AUTO PID TUNING ********************************************************

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            logDebug = false;
            auto_tune_pid(angularController, false, 2, 5);
        }

        // ************************************************************************

        pros::delay(25);
    }
}
