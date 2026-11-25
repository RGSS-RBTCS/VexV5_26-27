#include "main.h" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cmath>


// ----------- INIT PORTS FOR ROBOT MODULES ----------- //
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-2, 4, -6}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({3, 5, -7}, pros::MotorGearset::blue);
pros::MotorGroup cascadeLiftMotors({-8, 9}, pros::MotorGearset::green);
pros::Motor claw(15, pros::MotorGearset::blue); // Assuming port 15 for the claw motor
pros::Motor clawOrientationMotor(14, pros::MotorGearset::blue); // Assuming port 14 for the claw orientation motor

// collectors
// pros::Motor FirstCollector(16, pros::MotorGearset::blue);   // front bottom collector
// pros::Motor SecondCollector(12, pros::MotorGearset::blue);  // back collector
// pros::Motor ThirdCollector(-11, pros::MotorGearset::blue);   // front top collector

// pneumatics
// pros::adi::Pneumatics wing('E', false);
// pros::adi::Pneumatics feeder('G', false);
// pros::adi::Pneumatics stopper('H', false);

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

// sensors
pros::Imu imu(10); // IMU sensor on port 10

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

//-------------------- ROBOT FUNCTION CREATION --------------------//

void cascadeLift(double targetInches, int velocityRPM = 200) {
    constexpr double LIFT_DEG_PER_INCH = 45.0; // distance scale factor: motor degrees per 1 inch of lift travel
    double targetDegrees = targetInches * LIFT_DEG_PER_INCH;
    cascadeLiftMotors.move_absolute(targetDegrees, velocityRPM);
}

void clawMovement(bool open){
    const int CLAW_OPEN_POSITION = 100; // Adjust this value based on your claw's open position
    const int CLAW_CLOSED_POSITION = 70; // Adjust this value based on your claw's closed position
    if (open) {
        claw.move_absolute(CLAW_OPEN_POSITION, 100); // Move to open position at 100 RPM
    } else {
        claw.move_absolute(CLAW_CLOSED_POSITION, 100); // Move to closed position at 100 RPM
    }
}

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

/* Code that runs during autonomous period */
void autonomous() {
    chassis.setPose(0, 0, 0); // reset the robot's position to (0, 0) with a heading of 0 degrees
    //----------Variable Declarations----------//
    const int movementTime = 1500;
    const int targetinches = 14; //Target height for the cascade lift in inches
    const int firstPinHeight = 10;
    //-----------------------------------------//

    //----------------Autonomous Stage 1-----------------//
    chassis.moveToPose(9, 5.7, -90, movementTime, {.maxSpeed = 127}, true); //Turn and move to point (9, 5.7) within halfRotationTime ms
    chassis.waitUntilDone(); //Wait until the robot has finished turning
    clawMovement(true); //Open the claw
    chassis.waitUntilDone(); //Wait until the claw has finished opening
    clawMovement(false); //Close the claw
    chassis.waitUntilDone(); //Wait until the claw has finished closing
    cascadeLift(targetinches, 200); //Lift the cascade lift to the target height at 200 RPM
    chassis.moveToPose(97, 29, -90, movementTime, {.maxSpeed = 127}, true); //Turn and move to point (97, 29) within halfRotationTime ms
    chassis.waitUntilDone(); //Wait until the robot has finished turning and moving lift
    cascadeLift(firstPinHeight, 200); //Lower the cascade lift back to the first pin height at 200 RPM
    clawMovement(true); //Open the claw to release the object
    chassis.waitUntilDone(); //Wait until the claw has finished opening
    cascadeLift(0, 200); //Lower the cascade lift back to the starting position at 200 RPM

    //----------------Autonomous Stage 2-----------------//
    chassis.moveToPose(-12, 0.75, 90, movementTime, {.maxSpeed = 127}, true); //Turn and move to point (-12, 0.75) within movementTime ms
    chassis.waitUntilDone(); //Wait until the robot has finished turning
    clawMovement(false); //Close the claw to prepare for picking up another object
    chassis.waitUntilDone(); //Wait until the claw has finished closing
    cascadeLift(targetinches + 3.3, 200); //Lift the cascade lift to the target height at 200 RPM
    chassis.moveToPose(-12, 11.7, 180, movementTime, {.maxSpeed = 127}, true); //Turn and move to point (-12, 11.7) within movementTime ms
    chassis.waitUntilDone(); //Wait until the robot has finished lifting and moving
    clawMovement(true); //Open the claw to release the object
    chassis.waitUntilDone(); //Wait until the claw has finished opening

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

        pros::delay(25);
    }
}
