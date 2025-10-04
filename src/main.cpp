#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include <iostream>
#include "liblvgl/lvgl.h"
//#include "liblvgl/core/lv_event.h"
#include "liblvgl/misc/lv_event.h"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "liblvgl/core/lv_obj_style.h"
#include "liblvgl/misc/lv_color.h"
#include "liblvgl/misc/lv_style.h"
//#include "liblvgl/widgets/lv_btn.h"
#include "liblvgl/widgets/button/lv_button.h"

//#include "liblvgl/widgets/lv_label.h"
#include "liblvgl/widgets/label/lv_label.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"     // Include necessary headers
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <iostream>
#include "lemlib/asset.hpp"

lv_style_t pretty;
pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor frontroller(8);

pros::Motor storagebottom(1);

pros::Motor storagetop(4);

int selected_program;


pros::MotorGroup left_mg({1,11,-12});    //11 is the stacked motor, 1 is the far motor, -12 is the motor under the stacked motor
pros::MotorGroup right_mg({-18,-20, 10});  
//pros::MotorGroup left_mg ({1});
//pros::MotorGroup right_mg ({2});

void createbutton(lv_obj_t *obj, int x, int y, const char *text, lv_palette_t color){
lv_obj_set_pos(obj, x, y);
lv_obj_t* screen = lv_screen_active();
lv_obj_set_size(obj, 150, 100);
lv_obj_t*label = lv_label_create(obj);
lv_label_set_text(label, text);
lv_style_init(&pretty);
lv_style_set_bg_color(&pretty, lv_palette_main(color));
lv_obj_add_style(obj, &pretty, 0);
}

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


// drivetrain settings
lemlib::Drivetrain drivetrain(&left_mg, // left motor group
                              &right_mg, // right motor group
                              11.5, // 12 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
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

// angular motion controller
lemlib::ControllerSettings angularController(4, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             26, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr// inertial sensor
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(0, // joystick deadband out of 127
                                     5, // minimum output where drivetrain will move out of 127
                                     1.02 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(0, // joystick deadband out of 127
                                  5, // minimum output where drivetrain will move out of 127
                                  1.02 // expo curve gain
); 

lemlib::Chassis chassis(drivetrain,
                 linearController, 
                 angularController, 
                 sensors, &throttleCurve, 
                 &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */ 
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
    chassis.calibrate();
    pros::lcd::register_btn1_cb(on_center_button);

    pros::Task screenTask([&]()->void{
        while(true) {
            pros::lcd::print(0,"X:%f", chassis.getPose().x);
            pros::lcd::print(1,"Y:%f", chassis.getPose().y);
            pros::lcd::print(2,"Theta:%f", chassis.getPose().theta);
            lemlib::telemetrySink() ->info("chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }

    });
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
lv_obj_t*screen = lv_obj_create(NULL);
//lv_obj_t*buttonA = lv_btn_create(screen); Used for all buttons previously in version 8.0
lv_obj_t*buttonA = lv_button_create(screen);
lv_obj_t*buttonB = lv_button_create(screen);
lv_obj_t*buttonC = lv_button_create(screen);
lv_obj_t*buttonD = lv_button_create(screen);
lv_obj_t*buttonE = lv_button_create(screen);
//lv_style_t pretty;

void competition_initialize() {
     std::cout << "Loadded on 2/9/2025 7:21pm";
  //  lv_scr_load(screen); update from scr to screen.
lv_screen_load(screen);
createbutton(buttonA,20,30,"Red Left", (LV_PALETTE_BLUE));
//std::cout<<"Button created";
createbutton(buttonB,20,130,"Red Right", (LV_PALETTE_RED));

createbutton(buttonC,120,30,"Blue Right", (LV_PALETTE_RED));
createbutton(buttonD,120,130,"Blue Left", (LV_PALETTE_RED));
createbutton(buttonE,250,130,"Skills", (LV_PALETTE_RED));
}


void changeautonomous(lv_event_t*eventstart){
   // lv_obj_t*firstbutton = lv_event_get_target(eventstart);
//lv_event_get_current_target(eventstart);
lv_obj_t* firstbutton = (lv_obj_t*) lv_event_get_target(eventstart);
 
    if (firstbutton==buttonA){
        selected_program=5;
    }
      if (firstbutton==buttonB){
        std::cout<<"selected program 2 for reightredlight";
        selected_program=2;
    }
     if (firstbutton==buttonC){
        selected_program=3;
    }
      if (firstbutton==buttonD){
        selected_program=4;
    }
      if (firstbutton==buttonE){
        selected_program=6;
    }

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
/**chassis.moveToPoint(0,14,5000);
    chassis.turnToPoint(-27,14,5000,{.forwards = false},true);
    chassis.moveToPoint(-27,14,5000,{.forwards = false},true);
    pros::delay(1000);*/
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
 
    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs


        // Arcade control scheme
        int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
        int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
        left_mg.move(turn + dir);                      // Sets left motor voltage
        right_mg.move(turn - dir);                     // Sets right motor voltage
        pros::delay(20);   
        
        

         if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            frontroller.move_voltage(12000);
        } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            frontroller.move_voltage(-12000);
        }
         else {
       frontroller.move_voltage(0);}



  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        storagetop.move_voltage(12000);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
         storagetop.move_voltage(-12000);
    }
    else {
       storagetop.move_voltage(0);
    }



      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        storagebottom.move_velocity(100);
    }
     else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
         storagebottom.move_velocity(100);
    }
    else {
       storagebottom.move_velocity(0);
    }
}}


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


