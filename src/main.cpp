#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.h"
#include "pros/rtos.hpp"
#include <chrono>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}



pros::MotorGroup left_side_motors({-11,-12,-13}, pros::MotorGearset::blue);
pros::MotorGroup right_side_motors({16,15,14}, pros::MotorGearset::blue);


pros::Rotation vert_enc(-9); // ports A and B, reversed
pros::Rotation horiz_enc(8); // ports C and D, not reversed
pros::Motor lift(18);

pros::Motor intake(19);
pros::ADIDigitalOut goalLock('B');
pros::ADIDigitalOut arm('A');
pros::ADIDigitalOut intakeLift('C');
pros::ADIDigitalIn limits('D');
pros::Rotation liftRotation(20);
/*
pros::MotorGroup intake({intake1, intake2});
pros::ADIDigitalOut pto('E');
pros::ADIDigitalOut climb('G');
pros::ADIDigitalOut frontFlapR('F');
pros::ADIDigitalOut frontFlapL('D');
pros::ADIDigitalOut backFlaps('H');
pros::ADIDigitalOut lock('C');
*/
// left tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vert_enc, 2, 1); // 2.75" wheel diameter, -4.6" offset from tracking center
// right tracking wheel 
//lemlib::TrackingWheel right_tracking_wheel(&right_rot, 3.25, 3.375); // 2.75" wheel diameter, 1.7" offset from tracking center

lemlib::TrackingWheel horizontal_tracking_wheel(&horiz_enc, 2, -2.56);
 
//
pros::Imu inertial_sensor(10);
// drivetrain
lemlib::Drivetrain drivetrain {
    &left_side_motors,
    &right_side_motors,
    11.1875, // track width
    lemlib::Omniwheel::NEW_275, // wheel size
    450, // rpm
    1
};

// lateral motion controller
lemlib::ControllerSettings linearController {
  10,
  0.7,
    35,
    1,
    0.15,
    1000,
    0.15,
    1200,
    0
};

// angular motion controller
lemlib::ControllerSettings angularController {
    4,
    1,
    35,
    1, 
    0.14,
    500,
    0.15,
    1000,
  0
};


// sensors for odometry
lemlib::OdomSensors sensors {
    &vertical_tracking_wheel, // vertical tracker 1
    nullptr, // vertical tracker 2
    &horizontal_tracking_wheel, // horizontal tracker 1
    nullptr, // horizontal tracker 2
    &inertial_sensor // inertial
};


lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);



 int s = 0;
 int g = 0;
 double Kp = 4;   
 double Kd = 5;   
 double previous_error = 0.0;  // To calculate derivative term
 int min_output = 10;
 bool canceling = false;
 float tolerance = 0.3;
pros::Controller master(pros::E_CONTROLLER_MASTER);
//lift void
 bool is_moving_arm = false;
void move_arm_to_position(int target_degrees) {
    float current_position = liftRotation.get_position() / 100.0;
    is_moving_arm = true;

    while ((fabs(current_position - target_degrees) > tolerance) && canceling == false) {
        current_position = liftRotation.get_position() / 100.0;

        //Calculate ERROR
         double error = target_degrees - current_position;
    
          // PROPORTIONAL
          double P = error * Kp;

          // DERIVITIVE
          double derivative = error - previous_error;
          double D = derivative * Kd;

          //ouTPUT STUFF AND PREV ERROR
          double output = P + D;
          previous_error = error;
           if (output > 100) output = 100;
           if (output < -100) output = -100;

           if (fabs(output) < min_output) {
              output = (output > 0) ? min_output : -min_output;
           }

          // Move the arm based on PD output
          lift.move(output);

        pros::delay(20);  // Delay for responsiveness
    }

    lift.brake();  // Stop the motor once the target is reached
    is_moving_arm = false;  // Reset the movement flag
    if(canceling == true){
      canceling = false;
      is_moving_arm = true;
    }
}


int arm_target_position = 0;

// Arm control task function
void arm_control_task_fn() {
    while (true) {
        if (is_moving_arm) {
            move_arm_to_position(arm_target_position);  // Move to the current target position
        }
        pros::delay(20);  // Task delay to avoid overwhelming the CPU
    }
}

bool intake_is_moving = false;
int voltage = 0;
void intakeit(int voltage){
  intake_is_moving = true;
  intake.move(voltage);
  while(intake_is_moving){
    pros::delay(100);
    if(intake.get_actual_velocity()<1){
      intake.move(-127);
      pros::delay(190);
      intake.move(voltage);
    }
  }
  intake.brake();
}
void intake_stop(){
  intake_is_moving = false;
}

void intake_task_fn(void*) {
  while (true) {
    if (intake_is_moving) {
      intakeit(voltage);  // Call your original intakeit function
    }
    pros::delay(20);  // Small delay to keep the loop running smoothly
  }
}
void start_intake_task(int set_voltage) {
  voltage = set_voltage;   // Set the global voltage value for the intake
  intake_is_moving = true; // Indicate that the intake should be running
  static pros::Task intake_task(intake_task_fn);  // Start the task that runs the intake system
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */


void initialize(){
	
  // calibrate chassis
  pros::lcd::initialize();
  chassis.calibrate();
  //chassis.setPose(-55.3,-24,270);
  chassis.setPose(-58, 13, 0);
  left_side_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
  intake.set_brake_mode(MOTOR_BRAKE_COAST);
  right_side_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
  lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

  lift.set_zero_position(0);
  lift.set_zero_position(0);
  liftRotation.set_position(0);
  pros::Task arm_control_task(arm_control_task_fn);
  //chassis.setPose(-5, 60, 180);
  // print odom values to the brain
  pros::Task screenTask([=]() {
   while (true) {
      pros::lcd::print(0, "X: %f", chassis.getPose().x);
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
      pros::delay(10);
    }
  });
  
}
/*
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
void competition_initialize() {}

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
 //Arm Control

//
 //////


 ASSET(awp1_txt);
 ASSET(redSide2_txt);
 /*
void auto1(){
  intake.move(127);
  chassis.moveToPose(9.9, -59.4, 270, 800, {.minSpeed = 40});
  chassis.follow(farRush4_txt, 12, 3000, false, true);
  pros::delay(700);
  backFlaps.set_value(true);
  pros::delay(230);
  backFlaps.set_value(false);
  chassis.moveToPose(56, -44, 180, 1000,{.minSpeed = 80});
  chassis.turnToHeading(4, 800);
  pros::delay(800);
  intake.move(-127);
  frontFlapR.set_value(true);
  frontFlapL.set_value(true);
  chassis.moveToPoint(60, -20, 800, {}, false);
  pros::delay(100);
  frontFlapR.set_value(false);
  frontFlapL.set_value(false);
   chassis.follow(farRush4_5_txt, 12, 2000, false);
  chassis.turnToPoint(4, -20, 500);
  intake.move(127);
  chassis.follow(farRush5_txt, 12, 3000, true);
  chassis.turnToHeading(67, 800);
  chassis.follow(farRush6_txt, 12, 1000, true, true);
  pros::delay(300);
  intake.move(-100);
  chassis.turnToHeading(260, 1000);
  intake.move(127);
  chassis.follow(farRush7_txt, 12, 1000, true, false);
  chassis.turnToHeading(95, 800);
  pros::delay(500);
  intake.move(-127);
  frontFlapR.set_value(true);
  frontFlapL.set_value(true);
  chassis.moveToPoint(56, -8, 2000, {}, false);
  chassis.moveToPoint(40, -8, 2000, {}, false);
 
}
*/
void autonomous() {
  
  //limit.set_position(500);
  chassis.moveToPoint(-58, 0, 820, {.forwards=false,.maxSpeed=70});
  chassis.turnToHeading(90, 690,{}, false);
   chassis.moveToPoint(-60.55, 0, 850, {.forwards=false}, false);
  intake.move(127);
  pros::delay(170);
  chassis.turnToPoint(-47, 0, 300, {.maxSpeed=70});
  chassis.moveToPoint(-56, 0, 600, {.maxSpeed=90}, false);
  chassis.turnToPoint(-23, 3, 680, {.forwards=false});
  chassis.follow(awp1_txt, 12, 2000, false, true);
  pros::delay(1200);
  goalLock.set_value(true);
  start_intake_task(127);
  chassis.turnToPoint(-11, 37, 800);
  chassis.moveToPoint(-11, 37,1000);
  chassis.turnToPoint(-11, 48, 1000);
  chassis.moveToPoint(-11, 48, 700);
  chassis.turnToPoint(-22, 40, 700);
  chassis.moveToPoint(-19.3, 42, 800);
  chassis.turnToPoint(-55, 31, 700);
  intake_stop();
  chassis.moveToPoint(-55, 31, 800);
  chassis.turnToPoint(-48, 57, 600, {false}, false);
  chassis.moveToPoint(-48, 57, 600, {false});
  chassis.turnToPoint(-27, 12, 490);
  start_intake_task(127);
  chassis.moveToPoint(-27, 12, 1000, {}, false);
  arm.set_value(true);
  
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

void testForwardPid(){
  chassis.moveToPoint(0, 20, 6000);
}

void testAngularPid(){
  chassis.turnToHeading(90, 2000,{}, false);
}


int i = 1;
int j = 0;
int h = 0;
int k = 0;
int l = 0;
int m = 0;
int f = 0;
int z = 0;
int r = 0;
void opcontrol() {
  left_side_motors.set_brake_mode(MOTOR_BRAKE_COAST);
  right_side_motors.set_brake_mode(MOTOR_BRAKE_COAST);
  //limit.set_position(500);
  arm_target_position = 10;
    canceling = false;
    is_moving_arm = true;  
  while(true){
    intake.set_brake_mode(MOTOR_BRAKE_COAST);
      if(master.get_digital(DIGITAL_R2)==1){
          intake.move(-127);
        } else if (master.get_digital(DIGITAL_R1)==1){
        intake.move(127);
      } else {
        intake.brake();
      }

      /*
      if(master.get_digital(DIGITAL_L1) == 1) {
        if(i==1){
          move_arm_to_position(36);
          i++;
        } else if(i==2) {
           move_arm_to_position(160);
           i++;
        }else {
           i=3;
        }
    } else if(master.get_digital(DIGITAL_L2) == 1) {
        if(i==1){
          i=2;
        } else if(i==2) {
           move_arm_to_position(10);
           i=1;
        }else {
           move_arm_to_position(47);
           i=2;
        }
  
    }
*/
//
 if (master.get_digital_new_press(DIGITAL_L1) == 1) {
      
            if (i == 1) {
                arm_target_position = 44;  // Set arm target
                if (is_moving_arm==true){
                  canceling=true;
                  pros::delay(20);
                  canceling=false;
                } else {
                canceling = false;
                is_moving_arm = true;      // Trigger movement
                }
                i++;
            } else if (i == 2) {
                arm_target_position = 153; // Another target
                if (is_moving_arm==true){
                  canceling=true;
                  pros::delay(20);
                  canceling=false;
                } else {
                canceling = false;
                is_moving_arm = true;      // Trigger movement
                }
                i++;
            } else {
                i = 3;
            }
        
  
        } else if (master.get_digital_new_press(DIGITAL_L2) == 1){
         
            if (i == 1) {
                i = 2;
            } else if (i == 2) {
                arm_target_position = 10;  // Set arm target
                if (is_moving_arm==true){
                  canceling=true;
                  pros::delay(20);
                  canceling=false;
                } else {
                canceling = false;
                is_moving_arm = true;      // Trigger movement
                }
                i = 1;
            } else {
                arm_target_position = 44;  // Set arm target
                if (is_moving_arm==true){
                  canceling=true;
                  pros::delay(20);
                  canceling=false;
                } else {
                canceling = false;
                is_moving_arm = true;      // Trigger movement
                }
                i = 2;
            }
          
        }
        
//
    
      if(master.get_digital_new_press(DIGITAL_A)){
          if(j==0){
          goalLock.set_value(true);
          j++;
        } else {
          goalLock.set_value(false);
          j=0;
        }
      }
      if(master.get_digital_new_press(DIGITAL_B)){
          if(f==0){
          arm.set_value(true);
          f++;
        } else {
          arm.set_value(false);
          f=0;
        }
      }
       if(master.get_digital_new_press(DIGITAL_Y)){
          if(z==0){
          intakeLift.set_value(true);
          z++;
        } else {
          intakeLift.set_value(false);
          z=0;
        }
      }
     

      /*
      if(f==1){
      if(sensor.get_proximity()>230){
          puncher.move(127);
          pros::delay(300);
          while(limit.get_position()<6000){
            puncher.move(127);
          } 
      }
      puncher.brake();
      }
      */
    //int val = limit.get_angle();
    int avgDrivetrainTemp = (left_side_motors.get_temperature()+right_side_motors.get_temperature())/2;
    //int punchTemp = (puncher1.get_temperature()+puncher2.get_temperature())/2;
   //master.print(1, 2, "P: %d", pros::c::rotation_get_position(20)/100);
 
   //master.print(1, 2, "P:%d(%d) D:%d(%d)", punchTemp, (int)((punchTemp*1.8)+32), avgDrivetrainTemp, (int)((avgDrivetrainTemp*1.8)+32));
   master.print(1, 2, "D:%d(%d)", avgDrivetrainTemp, (int)((avgDrivetrainTemp*1.8)+32));
		int left = master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X);
		int right = master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X);
		left_side_motors.move(left);
		right_side_motors.move(right);
    
		pros::delay(10);
	
      
  }
}
