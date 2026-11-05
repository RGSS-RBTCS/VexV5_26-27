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
pros::MotorGroup leftMotors({-2, -4}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({9, 10}, pros::MotorGearset::blue);

// collectors
// pros::Motor FirstCollector(16, pros::MotorGearset::blue);   // front bottom collector
// pros::Motor SecondCollector(12, pros::MotorGearset::blue);  // back collector
// pros::Motor ThirdCollector(-11, pros::MotorGearset::blue);   // front top collector

// sensors
pros::Imu imu(3);

// pneumatics
// pros::adi::Pneumatics wing('E', false);
// pros::adi::Pneumatics feeder('G', false);
// pros::adi::Pneumatics stopper('H', false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 15.25, lemlib::Omniwheel::NEW_275, 450, 2); // TODO: drivetrain RPM
// controllers
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// tracking wheels
pros::Rotation horizontalEnc(17);
pros::Rotation verticalEnc(6);
lemlib::TrackingWheel horizontal(&horizontalEnc, 1.875, 12.5-8.75);
lemlib::TrackingWheel vertical(&verticalEnc, 1.875, (15.75/2)-13);

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

/* ASSETS */
ASSET(drivetrain_showcase_txt);

/* Code that runs during autonomous period */
void autonomous() {
    chassis.setPose(0, 0, 0);

    chassis.follow(drivetrain_showcase_txt, 15, 20000);
}

// -------------------- Driver Control -------------------- //
bool driveDirection = true; // default direction, brain side
int modifier = driveDirection ? 1 : -1;

/* Code that runs during driver control; The manual controls for the robot */
void opcontrol() {
    while (true) {
        int leftY = modifier*controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY/2, rightX/2);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            driveDirection = !driveDirection;
            modifier = driveDirection ? 1 : -1;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)
            ) {
                chassis.moveToPose(0, 0, 0, 5000); // move to initial point
                chassis.follow(drivetrain_showcase_txt, 15, 20000);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)
            ) {
                chassis.moveToPose(0, 0, 0, 5000); // move to initial point
                chassis.follow(drivetrain_showcase_txt, 15, 20000);
        }

        pros::delay(25);
    }
}
