#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
using namespace pros;

Controller controller(E_CONTROLLER_MASTER);
Motor left_motor(11);
Motor right_motor(20);
Motor intake_motor(1);
Motor elastic_motor_1(19);
Motor elastic_motor_2(2);
Motor belt_motor(10);
//Motor pusher_motor(2);

MotorGroup left_drive({11});
MotorGroup right_drive({20});

Imu imu(18);

lemlib::Drivetrain drivetrain(
    &left_drive, &right_drive,
    13.5,  // track width (measure center-to-center of wheels in inches)
    3.2,  // wheel diameter in inches (measure your wheels!)
    200,   // RPM - depends on your motor cartridge:
           // 600 (blue/36:1), 200 (green/18:1), 100 (red/36:1)
    2      // chase power
);

lemlib::OdomSensors sensors(
    nullptr, nullptr, nullptr, nullptr,
    &imu
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
//13.5 track
//12.5 wheelbase
//4 inch wheels



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {


  left_motor.set_brake_mode(MOTOR_BRAKE_HOLD); 
  right_motor.set_brake_mode(MOTOR_BRAKE_HOLD);

  left_motor.set_gearing(MOTOR_GEARSET_18);
  right_motor.set_gearing(MOTOR_GEARSET_18);

  left_motor.set_reversed(false);
  right_motor.set_reversed(true);

  intake_motor.set_brake_mode(MOTOR_BRAKE_BRAKE);
  intake_motor.set_gearing(MOTOR_GEARSET_18);

  belt_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  belt_motor.set_gearing(MOTOR_GEARSET_18);

  intake_motor.set_reversed(true);
  belt_motor.set_reversed(true);

  //pusher_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
  //pusher_motor.set_encoder_units(MOTOR_ENCODER_DEGREES);

  elastic_motor_1.set_brake_mode(MOTOR_BRAKE_HOLD);
  elastic_motor_2.set_brake_mode(MOTOR_BRAKE_HOLD);

  elastic_motor_1.set_reversed(true);


  chassis.calibrate();
}


// ai wrote these funcs because im lazy and its simple
void drive_forward(int speed, int time) {
	left_motor.move(speed);
	right_motor.move(speed);
	delay(time);
	left_motor.brake();
	right_motor.brake();
}

void turn_right(int speed, int time) {
	left_motor.move(speed);
	right_motor.move(-speed);
	delay(time);
	left_motor.brake();
	right_motor.brake();
}

void turn_left(int speed, int time) {
	left_motor.move(-speed);
	right_motor.move(speed);
	delay(time);
	left_motor.brake();
	right_motor.brake();
}

void set_drive_forward(int speed) {
	left_motor.move(speed);
	right_motor.move(speed);
}
void set_drive_right(int speed) {
	left_motor.move(speed);
	right_motor.move(-speed);
}
void set_drive_left(int speed) {
	left_motor.move(-speed);
	right_motor.move(speed);
}

void activate_intake(int speed) {
	intake_motor.move(speed);
}
void activate_belt(int speed) {
	belt_motor.move(speed);
}

void test_auton() {
	drive_forward(70, 1000);
	delay(500);
	drive_forward(-70, 1000);
	delay(500);
	turn_right(70, 500);
	delay(500);
	turn_left(70, 500);
	delay(500);
	activate_intake(127);
	activate_belt(90);
	delay(2000);
	activate_intake(0);
	activate_belt(0);

}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {} // Not really used



/**
 * Runs the user autonomous code. TAhis function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() { // Autonomous section
	test_auton();
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
void opcontrol() { //Operator control section
	double drive_mult = 0.7;
	int elastic_drive_speed = 127;
	//bool pusher_extended = false;
	while (true) {
		int forward = controller.get_analog(ANALOG_LEFT_Y);    // Forward/backward movement
		int turn = controller.get_analog(ANALOG_LEFT_X);       // Left/right turning
		
		// intake r1
		// belt r2
		// inverse for reverse modes	
		 
		// drive direction toggle
		if (controller.get_digital(E_CONTROLLER_DIGITAL_A)) {
			drive_mult = -drive_mult;
		}
		
		// intake control
		if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			intake_motor.move(elastic_drive_speed); //make intake intake
			elastic_motor_1.move(elastic_drive_speed);
		} else if (controller.get_digital(E_CONTROLLER_DIGITAL_L1))
		{
			intake_motor.move(-elastic_drive_speed); //run intake in reverse
			elastic_motor_1.move(-elastic_drive_speed);

		} else {
			intake_motor.move(0); // stop intake`
			elastic_motor_1.move(0);
			}

		// belt control
		if (controller.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			elastic_motor_2.move(-elastic_drive_speed); // make middle part of block mover go
		} else {elastic_motor_2.move(0);}
		if (controller.get_digital(E_CONTROLLER_DIGITAL_L2)){
			belt_motor.move(-75); //make belt run down
		} else {
			belt_motor.move(0); // stop belt
		}


		// Calculate motor speeds for tank drive
		int left_speed = forward + turn;   // Left motor: forward + turn
		int right_speed = forward - turn;  // Right motor: forward - turn
		
		left_motor.move(left_speed*drive_mult);
		right_motor.move(right_speed*drive_mult);
		
		delay(20);
	}
}