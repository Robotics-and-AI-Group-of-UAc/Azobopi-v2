#ifndef main_h // ifndef main_h to prevent double declaration of any identifiers such as types, enums and static variables
#define main_h // ifndef main_h 

// debug setup
//#define DEBUG_VAR
//#define DEBUG_ACT
//#define DEBUG_FCT
//#define DEBUG_STATE

//@ Debugging forward motors output
//#define DEBUG_FORWARD_OUTPUT
//@ Debugging turn
#define DEBUG_TURN_OUTPUT
//@ Debugging backward motors output
//#define DEBUG_BACKWARD_OUTPUT

#ifdef DEBUG_VAR
  #define DEBUG_PRINT_VAR(x) Serial.print(x)
  #define DEBUG_PRINTLN_VAR(x) Serial.println(x)
#else
  #define DEBUG_PRINTLN_VAR(x)
  #define DEBUG_PRINT_VAR(x) 
#endif

#ifdef DEBUG_ACT
  #define DEBUG_PRINT_ACT(x) Serial.println(x)
  #define DEBUG_PRINTLN_ACT(x) Serial.println(x)
#else
  #define DEBUG_PRINT_ACT(x)
  #define DEBUG_PRINTLN_ACT(x)
#endif

#ifdef DEBUG_FCT
  #define DEBUG_PRINTLN_FCT(x) Serial.println(x)
#else
  #define DEBUG_PRINTLN_FCT(x)
#endif

#ifdef DEBUG_STATE
  #define DEBUG_PRINTLN_STATE(x) Serial.print("Machine State: "); Serial.println(x)
#else
  #define DEBUG_PRINTLN_STATE(x)
#endif // debug setup

// include software header files
#include <Arduino.h>

//Pin Settings and hardware header files

// NeoPixel LED pins and header
#include "Adafruit_NeoPixel.h"
#define NEOPIXEL_PIN 23
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_RGB + NEO_KHZ800); //setup Neopixel

// ezButton pins and header
#include "ezButton.h"
#define button_debounce_time 100 
ezButton button_command(4); 
ezButton button_left(16);
ezButton button_backwards(19);
ezButton button_forwards(17);
ezButton button_right(18);
ezButton button_stop (5);

// States
#define VOID_ST 0
#define INIT_ST 1
#define START_EXEC_ST 2
#define EXEC_ST 3
#define READ_COMM_ST 4
#define STOP_ST 5
#define FORWARD_ST 6
#define TURN_RIGHT_ST 7
#define TURN_LEFT_ST 8
#define BACK_ST 9
#define TUNE_ST 10
#define WAIT_ST 11

// Movement Commands
#define MAX_NR_COMMANDS 20
#define STOP 0
#define TURN_LEFT 1
#define TURN_RIGHT 3
#define FORWARD 2
#define BACKWARD 4
#define WAIT 5
#define ROTATION_TICKS 1920

// commands
int nr_comm;
int comm_index;         // the index of the action that is being executed...
int recorded_button[MAX_NR_COMMANDS];
int button_index = 0;
int mov;                // Programed data from buttons
unsigned long button_command_count;  // Nr. of times command button is pressed
unsigned long button_stop_count = 0; // Nr. of times stop button is pressed

int on_execute_test_st; // state control variable
int on_execute_comm_st; // state control variable

// TODO - state in movement
int machine_state;
int last_machine_state;
int stop_next_state;

// PID
#include "PID_simple.h"

unsigned long time_now;

double val_outputL;
double val_outputR;
double enc_readL;
double enc_readR;
//@Not using PID. Shift power allows to give more power to right or to left.
double shift_powerL = 0;//.
double shift_powerR = 1; //
//@The fixed power to both motors
int power_base = 50;
int    kspeed = 1;
volatile int counterPID;
// freq for calculating PID
int freq = 2000; //2000

//PID for run
// K values
double kpR_r = 0.016; //previous values: 0.01
double kpL_r = 0.015;
double ki_r = 0.0019; //previous values: 0.0001 0.0008
double kd_r = 0.009; // previous values: 0.006  
// ** Simulation of the value for the SETPOINT_RUN **
// Kp: The value for Kp = 0.01 will be aprox 39 and it reduced as it approaches the final value.
// Ki: if error is reduced by 100 each time it is evaluated
// then 3900 + 3800 + 3700 + 3600 + ... is the incremented value 
// A value 0.001 increments 3.9 + 3.8 + 3.6. A value of 0.0001 increments
// 0.3 + 0.3 + 0.3 ...
//
// Kd: The value is reduced by 100 each time. So a negative value is added 
// in counter cycle of Ki. If it has a value of 0.01 it reduces by 1: -1-1-1-1...
//
// When the distance to target is half of the initial value (1950)...
// kp will reduce the value to 20, 19, 18....
// Ki increments  value by 0.1
// Kd: still reduces by -1
// ************************************************** 

//PID for turn

double kpR_t = 0.0358; //previous values: 0.01
double kpL_t = 0.035;
double ki_t = 0.004; //previous values: 0.0001 0.0008
double kd_t = 0.006; // previous values: 0.006  
// ** Simulation of the value for the SETPOINT_RUN **
// Kp: The value for Kp = 0.01 will be aprox 39 and it reduced as it approaches the final value.
// Ki: if error is reduced by 100 each time it is evaluated
// then 3900 + 3800 + 3700 + 3600 + ... is the incremented value 
// A value 0.001 increments 3.9 + 3.8 + 3.6. A value of 0.0001 increments
// 0.3 + 0.3 + 0.3 ...
//
// Kd: The value is reduced by 100 each time. So a negative value is added 
// in counter cycle of Ki. If it has a value of 0.01 it reduces by 1: -1-1-1-1...
//
// When the distance to target is half of the initial value (1950)...
// kp will reduce the value to 20, 19, 18....
// Ki increments  value by 0.1
// Kd: still reduces by -1
// ************************************************** 



// tune turn movement
#define SETPOINT_TURN 1540;//Reference:1540; 
#define SETPOINT_VALUES_TURN 7 // number of possible tuning setpoints in equivalent distances

int setpoint_values_turn[SETPOINT_VALUES_TURN];
int setpoint_turn_min = -200;
int setpoint_turn_max = 200;
int tune_counter_turn;
double Setpoint_t;

// tune forward/backward movement 
#define SETPOINT_RUN 3450; 
// SetPoints for PID
// SetPoints: Original 3900

#define  SETPOINT_VALUES_RUN 7// number of possible tuning setpoints in equivalent distances
float setpoint_values_move[SETPOINT_VALUES_RUN];
float setpoint_move_min = -200.00;
float setpoint_move_max = 200.00;
int tune_counter_move;
// initial straight run
double Setpoint_r;




// Encoders Interrupt function variables and table
volatile double encoder1_pos;
volatile double encoder2_pos;
byte encoder1_state, encoder2_state;
int  encoder_table[] = { 0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 };

// Motors
#include "ESP32MotorControl.h"
// Initialize motors library
ESP32MotorControl MotorControl = ESP32MotorControl();

// initial motor speed
//int speedL = 60; // because azobopi floated to right side     
//int speedR = 60;
//@ reducing the speed
int speedL = 50; // because azobopi floated to right side     
int speedR = 50;


// motor speed for turning -> set lower fixed speed for turning
// @ lowering the speed
//int turnspeedL = 60;
//int turnspeedR = 60;
int turnspeedL = 40;
int turnspeedR = 40;


// time motors are stopped
#define STOP_DELAY 1000
#define WAIT_DELAY 2000 // time the robo is waiting in delay state
unsigned long time_wait; // waiting timer
bool reset_time_wait = 1; // bool to reset waiting timer

// Wheels
#define WHEEL_DIAMETER 66 // wheel diameter in mm
#define WHEEL_CIRCUMFERENCE (3.14 * WHEEL_DIAMETER)
#define WHEELS_DISTANCE 120
#define CURVE_CIRCUMFERENCE (3.14 * WHEELS_DISTANCE)

// Encoders pins
//@Chaning encoder pins
#define ENC2_A 34
#define ENC2_B 35

#define ENC1_A 36
#define ENC1_B 39

// Timer & Mutex for encoders and PID counters
hw_timer_t  *timer      = NULL;
portMUX_TYPE timerMux   = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;

// Initialize PID control for each motor forward and backward
PID pidleft_r(&Setpoint_r, &enc_readL, &val_outputL, kpL_r, ki_r, kd_r);
PID pidright_r(&Setpoint_r, &enc_readR, &val_outputR, kpR_r, ki_r, kd_r);
// Initialize PID control for each motor left and right
PID pidleft_t(&Setpoint_t, &enc_readL, &val_outputL, kpL_t, ki_t, kd_t);
PID pidright_t(&Setpoint_t, &enc_readR, &val_outputR, kpR_t, ki_t, kd_t);


// OLED DISPLAY SSD1306

#include <SPI.h> // inlucde libraries for use of OLED
#include <Wire.h> // inlucde libraries for use of OLED
#include <Adafruit_GFX.h> // inlucde libraries for use of OLED
#include <Adafruit_SSD1306.h> // inlucde libraries for use of OLED

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Display bitmaps

#define bitmap_height   128 // define bitmap size
#define bitmap_width    64  // define bitmap size

#include "displayuaclogo.h"
#include "smileys.h"
#endif // ifndef main_h

// Speaker setup

#define PIN_SPEAKER 12
#include "sounds.h"