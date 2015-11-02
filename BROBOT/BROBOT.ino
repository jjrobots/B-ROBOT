// B-ROBOT  SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// JJROBOTS BROBOT KIT: (Arduino Leonardo + BROBOT ELECTRONIC BRAIN v2 + STEPPER MOTOR drivers)
// This code is prepared for new BROBOT shield v2.0 with ESP8266 Wifi module
// You need to install libraries JJROBOTS_OSC, JJROBOTS_BROBOT, I2Cdev and MPU6050 (from Github repository)
// Author: JJROBOTS.COM (Jose Julio & Juan Pedro)
// Date: 02/09/2014
// Updated: 26/10/2015
// Version: 2.21
// License: GPL v2
// Project URL: http://jjrobots.com/b-robot (Features,documentation,build instructions,how it works, SHOP,...)
// New updates:
//   - New default parameters specially tuned for BROBOT EVO version
//   - Support for TouchMessages(/z). Main controls resets when user lift the fingers
//   - BROBOT sends the battery status to the interface
//   Remember to update the libraries and install the new TouchOSC layout
//  Thanks to our users on the forum for the new ideas. Specially sasa999, KomX, ... 

// The board needs at least 10-15 seconds with no motion (robot steady) at beginning to give good values...
// MPU6050 IMU using internal DMP processor. Connected via I2C bus
// Angle calculations and control part is running at 200Hz from DMP solution
// DMP is using the gyro_bias_no_motion correction method.

// The robot is OFF when the angle is high (robot is horizontal). When you start raising the robot it
// automatically switch ON and start a RAISE UP procedure.
// You could RAISE UP the robot also with the robot arm servo [OPTIONAL] (Push1 on the interface)
// To switch OFF the robot you could manually put the robot down on the floor (horizontal)

// We use a standard PID controllers (Proportional, Integral derivative controller) for robot stability
// More info on the project page
// We have a PI controller for speed control and a PD controller for stability (robot angle)
// The output of the control (motors speed) is integrated so it´s really an acceleration not an speed.

// We control the robot from a WIFI module using OSC standard UDP messages (JJROBOTS_OSC library)
// You need an OSC app to control de robot (example: TouchOSC for IOS and Android)
// Join the module Wifi Access Point (by default: JJROBOTS) with your Smartphone/Tablet...
// Install the BROBOT layout into the OSC app (Touch OSC) and start play! (read the project page)
// OSC controls:
//    fader1: Throttle (0.0-1.0) OSC message: /1/fader1
//    fader2: Steering (0.0-1.0) OSC message: /1/fader2
//    push1: Move servo arm (and robot raiseup) OSC message /1/push1 [OPTIONAL]
//    if you enable the touchMessage on TouchOSC options, controls return to center automatically when you lift your fingers
//    toggle1: Enable PRO mode. On PRO mode steering and throttle are more aggressive
//    PAGE2: PID adjustements [optional][don´t touch if you don´t know what you are doing...;-) ]

#define WIFI_ESP   // New WIFI ESP8266 Module. Comment this line if you are using the old Wifi module (RN-171)

#include <JJROBOTS_OSC.h>
#include <JJROBOTS_BROBOT.h>
#include <Wire.h>
#include <I2Cdev.h>                // I2Cdev lib from www.i2cdevlib.com
#include <JJ_MPU6050_DMP_6Axis.h>  // Modified version of the MPU6050 library to work with DMP (see comments inside)
// This version optimize the FIFO (only contains quaternion) and minimize code size

// NORMAL MODE PARAMETERS (MAXIMUN SETTINGS)
#define MAX_THROTTLE 580
#define MAX_STEERING 150
#define MAX_TARGET_ANGLE 12

// PRO MODE = MORE AGGRESSIVE (MAXIMUN SETTINGS)
#define MAX_THROTTLE_PRO 980 //680
#define MAX_STEERING_PRO 250
#define MAX_TARGET_ANGLE_PRO 40 //20

// Default control terms
#define KP 0.19         
#define KD 28           
#define KP_THROTTLE 0.07    
#define KI_THROTTLE 0.04   

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.16
#define KD_RAISEUP 36
#define KP_THROTTLE_RAISEUP 0   // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500

// Servo definitions
#define SERVO_AUX_NEUTRO 1550  // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 650
#define SERVO_MAX_PULSEWIDTH 2600

// Battery management [optional]. This is not needed for alkaline or Ni-Mh batteries but usefull for if you use lipo batteries
#define BATTERY_CHECK 1                // 0: No  check, 1: check (send message to interface)
#define LIPOBATT 0             // Default 0: No Lipo batt 1: Lipo batt (to adjust battery monitor range)
#define BATTERY_WARNING 105    // (10.5 volts) aprox [for lipo batts, disable by default]
#define BATTERY_SHUTDOWN 95   // (9.5 volts) [for lipo batts]
#define SHUTDOWN_WHEN_BATTERY_OFF 0    // 0: Not used, 1: Robot will shutdown when battery is off (BATTERY_CHECK SHOULD BE 1)

#define DEBUG 0   // 0 = No debug info (default)

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define ZERO_SPEED 65535
#define MAX_ACCEL 7        // Maximun motor acceleration (MAX RECOMMENDED VALUE: 8) (default:7)

#define MICROSTEPPING 16   // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

#define I2C_SPEED 400000L  // 400kHz I2C speed 

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define ITERM_MAX_ERROR 25   // Iterm windup constants for PI control //40
#define ITERM_MAX 8000       // 5000

bool Robot_shutdown = false; // Robot shutdown flag => Out of

String MAC;  // MAC address of Wifi module

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (for us 18 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;

uint8_t loop_counter;       // To generate a medium loop 40Hz
uint8_t slow_loop_counter;  // slow loop 2Hz
uint8_t sendBattery_counter; // To send battery status

long timer_old;
long timer_value;
int debug_counter;
float debugVariable;
float dt;

// class default I2C address is 0x68 for MPU6050
MPU6050 mpu;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
bool newControlParameters = false;
bool modifing_control_parameters = false;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;

uint8_t mode;  // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

int16_t motor1;
int16_t motor2;

int16_t speed_M1, speed_M2;        // Actual speed of motors
int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors
int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;    // Estimated robot speed

// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invensense name is inv_key_0_96 (??)
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}

// Quick calculation to obtein Phi angle from quaternion solution (from DMP internal quaternion solution)
float dmpGetPhi() {
  mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.resetFIFO();  // We always reset FIFO

  //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
  //return Phi angle (robot orientation) from quaternion DMP output
  return (atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * RAD2GRAD);
}

// PD controller implementation(Proportional, derivative). DT is in miliseconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp * error + (Kd * (setPoint - setPointOld) - Kd * (input - PID_errorOld2)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}


// PI controller implementation (Proportional, integral). DT is in miliseconds
float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT * 0.001; // DT is in miliseconds...
  return (output);
}

// 16 single cycle instructions = 1us at 16Mhz
void delay_1us()
{
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}

// TIMER 1 : STEPPER MOTOR1 SPEED CONTROL
ISR(TIMER1_COMPA_vect)
{
  if (dir_M1 == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  SET(PORTE, 6); // STEP MOTOR 1
  delay_1us();
  CLR(PORTE, 6);
}
// TIMER 3 : STEPPER MOTOR2 SPEED CONTROL
ISR(TIMER3_COMPA_vect)
{
  if (dir_M2 == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  SET(PORTD, 6); // STEP MOTOR 2
  delay_1us();
  CLR(PORTD, 6);
}


// Set speed of Stepper Motor1
// tspeed could be positive or negative (reverse)
void setMotorSpeedM1(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M1 * 46; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M1 * 23; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M1 = 1;
    SET(PORTB, 4); // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M1 = -1;
    CLR(PORTB, 4); // Dir Motor 1
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR1A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT1 > OCR1A)
    TCNT1 = 0;
}

// Set speed of Stepper Motor2
// tspeed could be positive or negative (reverse)
void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M2 * 46; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M2 * 23; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M2 = 1;
    CLR(PORTC, 6);   // Dir Motor2 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M2 = -1;
    SET(PORTC, 6);  // DIR Motor 2
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR3A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT3 > OCR3A)
    TCNT3 = 0;
}

// Read control PID parameters from user. This is only for advanced users that want to "play" with the controllers...
void readControlParameters()
{
  // Parameter initialization (first time we enter page2)
  if ((OSC.page == 2) && (!modifing_control_parameters))
  {
    OSC.fadder1 = 0.5;
    OSC.fadder2 = 0.5;
    OSC.fadder3 = 0.5;
    OSC.fadder4 = 0.0;
    //OSC.toggle1 = 0;
    modifing_control_parameters = true;
  }
  // Parameters Mode (page2 controls)
  // User could adjust KP, KD, KP_THROTTLE and KI_THROTTLE (fadder1,2,3,4)
  // Now we need to adjust all the parameters all the times because we don´t know what parameter has been moved
  if (OSC.page == 2)
  {
    Kp_user = KP * 2 * OSC.fadder1;
    Kd_user = KD * 2 * OSC.fadder2;
    Kp_thr_user = KP_THROTTLE * 2 * OSC.fadder3;
    Ki_thr_user = (KI_THROTTLE + 0.1) * 2 * OSC.fadder4;
#if DEBUG>0
    Serial.print("Par: ");
    Serial.print(Kp_user);
    Serial.print(" ");
    Serial.print(Kd_user);
    Serial.print(" ");
    Serial.print(Kp_thr_user);
    Serial.print(" ");
    Serial.println(Ki_thr_user);
#endif
    // Kill robot => Sleep
    while (OSC.toggle2)
    {
      //Reset external parameters
      mpu.resetFIFO();
      PID_errorSum = 0;
      timer_old = millis();
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      OSC.MsgRead();
    }
    newControlParameters = true;
  }
  if ((newControlParameters) && (!modifing_control_parameters))
  {
    // Reset parameters
    OSC.fadder1 = 0.5;
    OSC.fadder2 = 0.5;
    //OSC.toggle1 = 0;
    newControlParameters = false;
  }
}

#ifdef WIFI_ESP

int ESPwait(String stopstr, int timeout_secs)
{
  String response;
  bool found = false;
  char c;
  long timer_init;
  long timer;

  timer_init = millis();
  while (!found) {
    timer = millis();
    if (((timer - timer_init) / 1000) > timeout_secs) { // Timeout?
      Serial.println("!Timeout!");
      return 0;  // timeout
    }
    if (Serial1_available()) {
      c = Serial1_read();
      Serial.print(c);
      response += c;
      if (response.endsWith(stopstr)) {
        found = true;
        delay(10);
        Serial1_flush();
        Serial.println();
      }
    } // end Serial1_available()
  } // end while (!found)
  return 1;
}

// getMacAddress from ESP wifi module
int ESPgetMac()
{
  char c1, c2;
  bool timeout = false;
  long timer_init;
  long timer;
  uint8_t state = 0;
  uint8_t index = 0;

  MAC = "";
  timer_init = millis();
  while (!timeout) {
    timer = millis();
    if (((timer - timer_init) / 1000) > 5) // Timeout?
      timeout = true;
    if (Serial1_available()) {
      c2 = c1;
      c1 = Serial1_read();
      Serial.print(c1);
      switch (state) {
        case 0:
          if (c1 == ':')
            state = 1;
          break;
        case 1:
          if (c1 == '\r') {
            MAC.toUpperCase();
            state = 2;
          }
          else {
            if ((c1 != '"') && (c1 != ':'))
              MAC += c1;  // Uppercase
          }
          break;
        case 2:
          if ((c2 == 'O') && (c1 == 'K')) {
            Serial.println();
            Serial1_flush();
            return 1;  // Ok
          }
          break;
      } // end switch
    } // Serial_available
  } // while (!timeout)
  Serial.println("!Timeout!");
  Serial1_flush();
  return -1;  // timeout
}

int ESPsendCommand(char *command, String stopstr, int timeout_secs)
{
  Serial1_println(command);
  ESPwait(stopstr, timeout_secs);
  delay(250);
}
#endif

// INITIALIZATION
void setup()
{
  // STEPPER PINS ON JJROBOTS BROBOT BRAIN BOARD
  pinMode(4, OUTPUT); // ENABLE MOTORS
  pinMode(7, OUTPUT); // STEP MOTOR 1 PORTE,6
  pinMode(8, OUTPUT); // DIR MOTOR 1  PORTB,4
  pinMode(12, OUTPUT); // STEP MOTOR 2 PORTD,6
  pinMode(5, OUTPUT); // DIR MOTOR 2  PORTC,6
  digitalWrite(4, HIGH);  // Disbale motors

  pinMode(10, OUTPUT);  // Servo1 (arm)
  pinMode(13, OUTPUT);  // Servo2

  Serial.begin(115200); // Serial output to console

  // Wifi module initialization: Wifi module should be preconfigured (use another sketch to configure the module)

  // Initialize I2C bus (MPU6050 is connected via I2C)
  Wire.begin();
  // I2C 400Khz fast mode
  TWSR = 0;
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2;
  TWCR = 1 << TWEN;

#if DEBUG > 0
  delay(9000);
#else
  delay(2000);
#endif
  Serial.println("BROBOT by JJROBOTS v2.2");
  Serial.println("Initializing I2C devices...");
  //mpu.initialize();
  // Manual MPU initialization... accel=2G, gyro=2000º/s, filter=20Hz BW, output=200Hz
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_10);  //10,20,42,98,188  // Default factor for BROBOT:10
  mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);

  delay(500);
  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println("Enabling DMP...");
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
  }
  else { // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  // Gyro calibration
  // The robot must be steady during initialization
  delay(500);
  Serial.println("Gyro calibration!!  Dont move the robot in 10 seconds... ");
  delay(500);

#ifdef WIFI_ESP
  // With the new ESP8266 WIFI MODULE WE NEED TO MAKE AN INITIALIZATION PROCESS
  Serial.println("Initalizing ESP Wifi Module...");
  Serial.println("WIFI RESET");
  Serial1_flush();
  Serial1_print("+++");  // To ensure we exit the transparent transmision mode
  delay(100);
  ESPsendCommand("AT","OK",1);
  ESPsendCommand("AT+RST", "OK", 2); // ESP Wifi module RESET
  ESPwait("ready", 6);
  ESPsendCommand("AT+GMR", "OK", 5);
  Serial1_println("AT+CIPSTAMAC?");
  ESPgetMac();
  Serial.print("MAC:");
  Serial.println(MAC);
  delay(250);
  ESPsendCommand("AT+CWQAP", "OK", 3);
  ESPsendCommand("AT+CWMODE=2", "OK", 3); // Soft AP mode
  // Generate Soft AP. SSID=JJROBOTS, PASS=87654321
  char *cmd =  "AT+CWSAP=\"JJROBOTS_XX\",\"87654321\",5,3";
  // Update XX characters with MAC address (last 2 characters)
  cmd[19] = MAC[10];
  cmd[20] = MAC[11];
  //const char cmd[50] = "AT+CWSAP=\"JJROBOTS_"+MAC.substring(MAC.length()-2)+"\",\"87654321\",5,3";
  ESPsendCommand(cmd, "OK", 6);
  // Start UDP SERVER on port 2222
  Serial.println("Start UDP server at port 2222");
  ESPsendCommand("AT+CIPMUX=0", "OK", 3);  // Single connection mode
  ESPsendCommand("AT+CIPMODE=1", "OK", 3); // Transparent mode
  //ESPsendCommand("AT+CIPSTART=\"UDP\",\"0\",2223,2222,0", "OK", 3);
  ESPsendCommand("AT+CIPSTART=\"UDP\",\"192.168.4.2\",2223,2222,0", "OK", 3);
  delay(250);
  ESPsendCommand("AT+CIPSEND",">",2);  // Start transmission (transparent mode)
  delay(5000);   // Time to settle things... the bias_from_no_motion algorithm needs some time to take effect and reset gyro bias.
#else
  delay(10000);   // Time to settle things... the bias_from_no_motion algorithm needs some time to take effect and reset gyro bias.
#endif

  // Verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  timer_old = millis();

  // Init servos
  Serial.println("Servo initialization...");
  BROBOT.initServo();
  BROBOT.moveServo1(SERVO_AUX_NEUTRO);

  // STEPPER MOTORS INITIALIZATION
  Serial.println("Steper motors initialization...");
  // MOTOR1 => TIMER1
  TCCR1A = 0;                       // Timer1 CTC mode 4, OCxA,B outputs disconnected
  TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler=8, => 2Mhz
  OCR1A = ZERO_SPEED;               // Motor stopped
  dir_M1 = 0;
  TCNT1 = 0;

  // MOTOR2 => TIMER3
  TCCR3A = 0;                       // Timer3 CTC mode 4, OCxA,B outputs disconnected
  TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
  OCR3A = ZERO_SPEED;   // Motor stopped
  dir_M2 = 0;
  TCNT3 = 0;

  //Adjust sensor fusion gain
  Serial.println("Adjusting DMP sensor fusion gain...");
  dmpSetSensorFusionAccelGain(0x20);

  delay(200);

  // Enable stepper drivers and TIMER interrupts
  digitalWrite(4, LOW);   // Enable stepper drivers
  // Enable TIMERs interrupts
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
  TIMSK3 |= (1 << OCIE1A); // Enable Timer1 interrupt

  // Little motor vibration and servo move to indicate that robot is ready
  for (uint8_t k = 0; k < 5; k++)
  {
    setMotorSpeedM1(5);
    setMotorSpeedM2(5);
    BROBOT.moveServo1(SERVO_AUX_NEUTRO + 100);
    delay(200);
    setMotorSpeedM1(-5);
    setMotorSpeedM2(-5);
    BROBOT.moveServo1(SERVO_AUX_NEUTRO - 100);
    delay(200);
  }
  BROBOT.moveServo1(SERVO_AUX_NEUTRO);

  Serial.print("BATTERY:");
  Serial.println(BROBOT.readBattery());

  // OSC initialization
  OSC.fadder1 = 0.5;
  OSC.fadder2 = 0.5;

  Serial.println("Let´s start...");

  mpu.resetFIFO();
  timer_old = millis();
  Robot_shutdown = false;
  mode = 0;
}


// MAIN LOOP
void loop()
{
  // If we run out of battery, we do nothing... STOP
#if SHUTDOWN_WHEN__OFF==1
  if (Robot_shutdown)
    return;
#endif

  debug_counter++;
  OSC.MsgRead();  // Read UDP OSC messages
  if (OSC.newMessage)
  {
    OSC.newMessage = 0;
    if (OSC.page == 1)  // Get commands from user (PAGE1 are user commands: throttle, steering...)
    {
      OSC.newMessage = 0;
      throttle = (OSC.fadder1 - 0.5) * max_throttle;
      // We add some exponential on steering to smooth the center band
      steering = OSC.fadder2 - 0.5;
      if (steering > 0)
        steering = (steering * steering + 0.5 * steering) * max_steering;
      else
        steering = (-steering * steering + 0.5 * steering) * max_steering;

      modifing_control_parameters = false;
      if ((mode == 0) && (OSC.toggle1))
      {
        // Change to PRO mode
        max_throttle = MAX_THROTTLE_PRO;
        max_steering = MAX_STEERING_PRO;
        max_target_angle = MAX_TARGET_ANGLE_PRO;
        mode = 1;
      }
      if ((mode == 1) && (OSC.toggle1 == 0))
      {
        // Change to NORMAL mode
        max_throttle = MAX_THROTTLE;
        max_steering = MAX_STEERING;
        max_target_angle = MAX_TARGET_ANGLE;
        mode = 0;
      }
    }
#if DEBUG==1
    Serial.print(throttle);
    Serial.print(" ");
    Serial.println(steering);
#endif
  } // End new OSC message

  timer_value = millis();

  // New DMP Orientation solution?
  fifoCount = mpu.getFIFOCount();
  if (fifoCount >= 18)
  {
    if (fifoCount > 18) // If we have more than one packet we take the easy path: discard the buffer and wait for the next one
    {
      Serial.println("FIFO RESET!!");
      mpu.resetFIFO();
      return;
    }
    loop_counter++;
    slow_loop_counter++;
    dt = (timer_value - timer_old);
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    angle_adjusted = dmpGetPhi();

#if DEBUG==8
    Serial.print(throttle);
    Serial.print(" ");
    Serial.print(steering);
    Serial.print(" ");
    Serial.println(mode);
#endif

#if DEBUG==1
    Serial.println(angle_adjusted);
#endif
    //Serial.print("\t");
    mpu.resetFIFO();  // We always reset FIFO

    // We calculate the estimated robot speed:
    // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
    actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 90.0; // 90 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed_Old - angular_velocity;     // We use robot_speed(t-1) or (t-2) to compensate the delay
    estimated_speed_filtered = estimated_speed_filtered * 0.95 + (float)estimated_speed * 0.05;  // low pass filter on estimated speed

#if DEBUG==2
    Serial.print(" ");
    Serial.println(estimated_speed_filtered);
#endif
    // SPEED CONTROL: This is a PI controller.
    //    input:user throttle, variable: estimated robot speed, output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output

#if DEBUG==3
    Serial.print(" ");
    Serial.println(estimated_speed_filtered);
    Serial.print(" ");
    Serial.println(target_angle);
#endif

    // Stability control: This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly on the output
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    // NOW we send the commands to the motors
    if ((angle_adjusted < 76) && (angle_adjusted > -76)) // Is robot ready (upright?)
    {
      // NORMAL MODE
      digitalWrite(4, LOW);  // Motors enable
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);

      // Push1 Move servo arm
      if (OSC.push1)  // Move arm
        {
        // Update to correct bug when the robot is lying backward
        if (angle_adjusted > -40)
          BROBOT.moveServo1(SERVO_MIN_PULSEWIDTH + 100);
        else
          BROBOT.moveServo1(SERVO_MAX_PULSEWIDTH + 100);
        }
      else
        BROBOT.moveServo1(SERVO_AUX_NEUTRO);

      // Push2 reset controls to neutral position
      if (OSC.push2)
      {
        OSC.fadder1 = 0.5;
        OSC.fadder2 = 0.5;
      }

      // Normal condition?
      if ((angle_adjusted < 45) && (angle_adjusted > -45))
      {
        Kp = Kp_user;            // Default user control gains
        Kd = Kd_user;
        Kp_thr = Kp_thr_user;
        Ki_thr = Ki_thr_user;
      }
      else    // We are in the raise up procedure => we use special control parameters
      {
        Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
        Kd = KD_RAISEUP;
        Kp_thr = KP_THROTTLE_RAISEUP;
        Ki_thr = KI_THROTTLE_RAISEUP;
      }
    }
    else   // Robot not ready (flat), angle > 70º => ROBOT OFF
    {
      digitalWrite(4, HIGH);  // Disable motors
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;

      // if we pulse push1 button we raise up the robot with the servo arm
      if (OSC.push1)
      {
        // Because we know the robot orientation (face down of face up), we move the servo in the appropiate direction for raise up
        if (angle_adjusted > 0)
          BROBOT.moveServo1(SERVO_MIN_PULSEWIDTH);
        else
          BROBOT.moveServo1(SERVO_MAX_PULSEWIDTH);
      }
      else
        BROBOT.moveServo1(SERVO_AUX_NEUTRO);

    }
    // Check for new user control parameters
    readControlParameters();

  } // End of new IMU data

  // Medium loop 40Hz
  if (loop_counter >= 5)
  {
    loop_counter = 0;
    // We do nothing here now...
#if DEBUG==10
    Serial.print(angle_adjusted);
    Serial.print(" ");
    Serial.println(debugVariable);
#endif
  } // End of medium loop

  if (slow_loop_counter >= 99) // 2Hz
  {
    slow_loop_counter = 0;
    // Read  status
#if BATTERY_CHECK==1
    int BatteryValue = BROBOT.readBattery();
    sendBattery_counter++;
    if (sendBattery_counter>=10){ //Every 5 seconds we send a message
      sendBattery_counter=0;
#if LIPOBATT==0
      // From >10.6 volts (100%) to 9.2 volts (0%) (aprox)
      float value = constrain((BatteryValue-92)/14.0,0.0,1.0);
#else
      // For Lipo battery use better this config: (From >11.5v (100%) to 9.5v (0%)
      float value = constrain((BatteryValue-95)/20.0,0.0,1.0);
#endif
      //Serial.println(value);
      OSC.MsgSend("/1/rotary1\0\0,f\0\0\0\0\0\0",20,value);
      }
    //if (Battery_value < BATTERY_WARNING){
    //  Serial.print("LOW BAT!! ");
    //  }
#endif   // BATTERY_CHECK
#if DEBUG==6
    Serial.print("B:");
    Serial.println(BatteryValue);
#endif
#if SHUTDOWN_WHEN_BATTERY_OFF==1

    if (BROBOT.readBattery(); < BATTERY_SHUTDOWN)
    {
      // Robot shutdown !!!
      Serial.println("LOW BAT!! SHUTDOWN");
      Robot_shutdown = true;
      // Disable steppers
      digitalWrite(4, HIGH);  // Disable motors
    }
#endif

  }  // End of slow loop
}




