#include "main.h"
#include "autons.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
using namespace pros;

// 1, 2, 11, 15, 18 BAD

// Drive Motors :: Slide Side is the Front
// pros::Motor driveLeftFrontTop(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor driveLeftFront(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor driveLeftBack(5, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);

// pros::Motor driveRightFrontTop(12, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor driveRightFront(12, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
// pros::Motor driveRightBack(3, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);

// flywheel Motor
pros::Motor Fly(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// intake motors
// pros::Motor IntakeRight(10, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor IntakeLeft(10, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_COUNTS);

// Expansion
pros::ADIDigitalOut walls('H');
pros::ADIDigitalOut verticalWalls('G');
pros::ADIDigitalOut lift('A');

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// // Gyro
// pros::Imu gyro(17);


// Chassis constructor
Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    {-16, -19, 14}

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {9, 7, -6}

    // IMU Port (gyro)
    ,
    4

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    3.25

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
    // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
    ,
    1.25

    // Uncomment if using tracking wheels
    /*
    // Left Tracking Wheel Ports (negative port will reverse it!)
    // ,{1, 2} // 3 wire encoder
    // ,8 // Rotation sensor

    // Right Tracking Wheel Ports (negative port will reverse it!)
    // ,{-3, -4} // 3 wire encoder
    // ,-9 // Rotation sensor
    */

    // Uncomment if tracking wheels are plugged into a 3 wire expander
    // 3 Wire Port Expander Smart Port
    // ,1
);

/**
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/
void initialize() {
    pros::delay(500);  // Stop the user from doing anything while legacy ports configure.

    // Configure your chassis controls
    chassis.toggle_modify_curve_with_controller(true);  // Enables modifying the controller curve with buttons on the joysticks
    chassis.set_active_brake(0);                        // Sets the active brake kP. We recommend 0.1.
    chassis.set_curve_default(0, 0);                    // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
    default_constants();                                // Set the drive to your own constants from autons.cpp!

    // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
    // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used.
    // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

    // Autonomous Selector using LLEMU
    ez::as::auton_selector.add_autons({
        Auton("Defensive Side", DefensiveSideAuton),
        Auton("Offensive Side", OffensiveSideAuton),
        Auton("Programing Skills", SkillsAuton),
        
        Auton("Example Drive\n\nDrive forward and come back.", drive_example),
        Auton("Example Turn\n\nTurn 3 times.", turn_example),
        Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
        Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
        Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
        Auton("Combine all 3 movements", combining_movements),
        Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

    // Initialize chassis and auton selector
    chassis.initialize();
    ez::as::initialize();
}

/**
* Runs while the robot is in the disabled state of Field Management System or
* the VEX Competition Switch, following either autonomous or opcontrol. When
* the robot is enabled, this task will exit.
*/

void disabled() {
    // . . .
    walls.set_value(false);
    verticalWalls.set_value(false);
}

/**
* Runs after initialize(), and before autonomous when connected to the Field
* Management System or the VEX Competition Switch. This is intended for
* competition-specific initialization routines, such as an autonomous selector
* on the LCD.
*
* This task will exit when the robot is enabled and autonomous or opcontrol
* starts.
*/

void competition_initialize() {
     // . . .
}

/**
* Runs the user autonomous code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the autonomous
* mode. Alternatively, this function may be called in initialize or opcontrol
* for non-competition testing purposes.
*
* If the robot is disabled or communications is lost, the autonomous task
* will be stopped. Re-enabling the robot will restart the task, not re-start it
* from where it left off.
*/

void autonomous() {
    chassis.reset_pid_targets();                // Resets PID targets to 0
    chassis.reset_gyro();                       // Reset gyro position to 0
    chassis.reset_drive_sensor();               // Reset drive sensors to 0
    chassis.set_drive_brake(E_MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency.

    ez::as::auton_selector.call_selected_auton();  // Calls selected auton from autonomous selector.
}

/**
* Runs the operator control code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the operator
* control mode.
*
* If no competition control is connected, this function will run immediately
* following initialize().
*
* If the robot is disabled or communications is lost, the
* operator control task will be stopped. Re-enabling the robot will restart the
* task, not resume it from where it left off.
*/


bool intaketoggle = false;
bool shoottoggle = false;
bool verticalExpanded = false;
bool wallsExpanded = false;
bool liftExpanded = false;
double counter = 0;
bool flyReverse = false;
bool intakeallowed = false;
int power = 125;

void opcontrol() {
    // This is preference to what you like to drive on.
    chassis.set_drive_brake(E_MOTOR_BRAKE_COAST);
    chassis.set_active_brake(0);
    int timer = 0;
    int wallsTimer = 0;
    int verticalTimer = 0;
    int liftTimer = 0;
    int flyR = 0;
    int fly = 0;
    int pwr = 0;
    int ds = 0;

    while (true) {
        // chassis.tank();  // Tank control
        chassis.arcade_standard(ez::SPLIT); // Standard split arcade
        // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
        // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
        // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade

        // . . .
        // Put more user control code here!
        // . . .

        // intake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            IntakeLeft = 127;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            IntakeLeft = -127;
        } else {
            IntakeLeft = 0;
        }

        // walls
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && wallsTimer <= timer) {
            wallsExpanded = !wallsExpanded;
            walls.set_value(wallsExpanded);
            wallsTimer = timer + 10;
        }

        // vertical walls
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) && verticalExpanded <= timer) {
            verticalExpanded = !verticalExpanded;
            verticalWalls.set_value(verticalExpanded);
            verticalTimer = timer + 10;
        }

        // Fly wheel
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && shoottoggle == false && fly <= timer) {
            Fly = flyReverse ? -power : power;

            shoottoggle = true;
            fly = timer + 15;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && shoottoggle == true && fly <= timer) {
            Fly = 0;

            shoottoggle = false;
            fly = timer + 15;
        }

        // Reverse flywheel
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && flyR <= timer) {
            flyReverse = !flyReverse;
            flyR = timer + 10;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && pwr <= timer) {
            if (power < 126) {
                power = (int)power + 2;
            }
            pwr = timer + 5;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && pwr <= timer) {
            if (power > 2) {
                power = (int)power - 2;
            }
            pwr = timer + 5;
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && liftTimer <= timer) {
            liftExpanded = !liftExpanded;
            lift.set_value(liftExpanded);
            liftTimer = timer + 10;
        }

        controller.print(0, 0, "Counter: %d", power);
        timer++;

        pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
    }
}

