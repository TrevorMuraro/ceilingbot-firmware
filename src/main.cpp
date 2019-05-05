#include <ros.h>
#include <Arduino.h>
#include <stdio.h>
#include <PID_v1.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

char log_buffer[50];

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define L_MOTOR_PWM_PIN 9
#define L_MOTOR_DIR_PIN 7
#define R_MOTOR_PWM_PIN 5
#define R_MOTOR_DIR_PIN 4
#define L_ENCODER_PIN 3
#define R_ENCODER_PIN 2

#define WHEELBASE 0.135 // in meters
#define WHEEL_RADIUS 0.033 // in meters
#define WHEEL_CIRCUMFERENCE 0.207 // in meters
#define ENCODER_TICKS_PER_ROTATION 40.0 // 20 holes in encoder disc
#define WHEEL_DISTANCE_PER_ENCODER_TICK (WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_ROTATION) // in meters

#define FORWARD true
#define BACKWARD false

#define L_MOTOR_FWD_OR_STOP LOW
#define L_MOTOR_BKWD HIGH
#define R_MOTOR_FWD_OR_STOP LOW
#define R_MOTOR_BKWD HIGH
#define MAX_PWM 255 //Arduino analogWrite (PWM) always on is 255

#define ENCODER_DEBOUNCE_TIME 1500 // in us

#define FEEDBACK_LOOP_FREQUENCY (float)2 // in Hz, must be in floating point or FEEDBACK_LOOP_TIME and TICK_CMR won't calculate properly
#define FEEDBACK_LOOP_TIME (1 / FEEDBACK_LOOP_FREQUENCY) // in seconds
#define TICK_CMR (int)((16000000 / (1024 * FEEDBACK_LOOP_FREQUENCY)) - 1) // When the timer counts to this, it interrupts

// PID constants
double Kp=2, Ki=5, Kd=1;

//How to log
//snprintf(log_buffer, sizeof(log_buffer) - 1, "Left wheel traveled %f meters", leftEncoderDistance);
//nh.loginfo(log_buffer);

void leftEncoderCallback();
void rightEncoderCallback();
void turnWheel(const int wheelCmdSpd, const unsigned int pwmPin, const unsigned int dirPin);
void cmdVelCallback(const geometry_msgs::Twist &twist);

geometry_msgs::Twist encoderVelocityMessage;
std_msgs::Float32 leftEncoderMessage;
std_msgs::Float32 rightEncoderMessage;
std_msgs::Int16 leftMotorMessage;
std_msgs::Int16 rightMotorMessage;

ros::Publisher encoderVelocityPub("encoder_velocity", &encoderVelocityMessage);
ros::Publisher leftEncoderPub("left_encoder_speed", &leftEncoderMessage);
ros::Publisher rightEncoderPub("right_encoder_speed", &rightEncoderMessage);
ros::Publisher leftMotorPub("left_motor_control", &leftMotorMessage);
ros::Publisher rightMotorPub("right_motor_control", &rightMotorMessage);

ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCallback);


/***************************** Encoders ******************************************/

volatile unsigned long leftEncoderLastChangeTime;
volatile unsigned long rightEncoderLastChangeTime;
volatile float leftEncoderCount;
volatile float rightEncoderCount;

volatile float leftWheelDesiredVelocity;
volatile float rightWheelDesiredVelocity;
volatile bool leftWheelLastDirection;
volatile bool rightWheelLastDirection;

double leftWheelDesiredSpeed, leftWheelEncoderSpeed, leftWheelPWM;
double rightWheelDesiredSpeed, rightWheelEncoderSpeed, rightWheelPWM;
PID leftWheelPID(&leftWheelEncoderSpeed, &leftWheelPWM, &leftWheelDesiredSpeed, Kp, Ki, Kd, DIRECT);
PID rightWheelPID(&rightWheelEncoderSpeed, &rightWheelPWM, &rightWheelDesiredSpeed, Kp, Ki, Kd, DIRECT);


void leftEncoderCallback() {
  unsigned long currentMicros = micros();
  if(currentMicros - leftEncoderLastChangeTime > ENCODER_DEBOUNCE_TIME) {
    leftEncoderCount++;
    leftEncoderLastChangeTime = currentMicros;
  }
}

void rightEncoderCallback() {
  unsigned long currentMicros = micros();
  if(currentMicros - rightEncoderLastChangeTime > ENCODER_DEBOUNCE_TIME) {
    rightEncoderCount++;
    rightEncoderLastChangeTime = currentMicros;
  }
}

// Encoder sum and PID control interrupt
ISR(TIMER1_COMPA_vect){
  float leftEncoderVelocity = (float) leftEncoderCount * WHEEL_DISTANCE_PER_ENCODER_TICK / FEEDBACK_LOOP_TIME;
  float rightEncoderVelocity = (float) rightEncoderCount * WHEEL_DISTANCE_PER_ENCODER_TICK / FEEDBACK_LOOP_TIME;

  leftEncoderCount = 0;
  rightEncoderCount = 0;

  if(leftWheelLastDirection == BACKWARD) {
    leftEncoderVelocity *= -1;
  }
  if(rightWheelLastDirection == BACKWARD) {
    rightEncoderVelocity *= -1;
  }

  leftEncoderMessage.data = leftEncoderVelocity;
  rightEncoderMessage.data = rightEncoderVelocity;

  encoderVelocityMessage.linear.x = (leftEncoderVelocity + rightEncoderVelocity) / (float)2;
  encoderVelocityMessage.angular.z = (rightEncoderVelocity - leftEncoderVelocity) / (2 * WHEELBASE);

  if(leftWheelDesiredVelocity < 0){
    leftWheelLastDirection = BACKWARD;
  } else {
    leftWheelLastDirection = FORWARD;
  }

  if(rightWheelDesiredVelocity < 0){
    rightWheelLastDirection = BACKWARD;
  } else {
    rightWheelLastDirection = FORWARD;
  }

  leftWheelDesiredSpeed = abs(leftWheelDesiredVelocity);
  rightWheelDesiredSpeed = abs(rightWheelDesiredVelocity);
  leftWheelEncoderSpeed = abs(leftEncoderVelocity);
  rightWheelEncoderSpeed = abs(rightEncoderVelocity);

  leftWheelPID.Compute();
  rightWheelPID.Compute();

  if(leftWheelDesiredVelocity >= 0) {
    turnWheel(leftWheelPWM, L_MOTOR_PWM_PIN, L_MOTOR_DIR_PIN);
    leftMotorMessage.data = leftWheelPWM;
  } else {
    turnWheel(leftWheelPWM * -1, L_MOTOR_PWM_PIN, L_MOTOR_DIR_PIN);
    leftMotorMessage.data = leftWheelPWM * -1;
  }

  if(rightWheelDesiredVelocity >= 0) {
    turnWheel(rightWheelPWM, R_MOTOR_PWM_PIN, R_MOTOR_DIR_PIN);
    rightMotorMessage.data = rightWheelPWM;
  } else {
    turnWheel(rightWheelPWM * -1, R_MOTOR_PWM_PIN, R_MOTOR_DIR_PIN);
    rightMotorMessage.data = rightWheelPWM * -1;
  }

  leftEncoderPub.publish(&leftEncoderMessage);
  rightEncoderPub.publish(&rightEncoderMessage);
  encoderVelocityPub.publish(&encoderVelocityMessage);
  leftMotorPub.publish(&leftMotorMessage);
  rightMotorPub.publish(&rightMotorMessage);
}



/*
 *  Takes a command speed from -255 to 255 (positive for forward, negative for backward),
 *  and the motor's PWM and direction pins, and controls the motor accordingly.
 */
void turnWheel(const int wheelCmdSpd,
      const unsigned int pwmPin,
      const unsigned int dirPin) {
  
  int wheelSpeed = abs(wheelCmdSpd);
  if (wheelSpeed > MAX_PWM) {
    wheelSpeed = MAX_PWM;
    nh.logwarn("Arduino: Motor command above max PWM value, adjusting down");
  }

  unsigned int motorDirVariable;

  if(wheelCmdSpd >= 0) {
    motorDirVariable = L_MOTOR_FWD_OR_STOP;
  } else {
    motorDirVariable = L_MOTOR_BKWD;
    wheelSpeed = MAX_PWM - wheelSpeed;
  }

  //snprintf(log_buffer, sizeof(log_buffer) - 1, "Wheel speed %i", wheelSpeed);
  //nh.loginfo(log_buffer);

  analogWrite(pwmPin, wheelSpeed);
  digitalWrite(dirPin, motorDirVariable);
}






void cmdVelCallback(const geometry_msgs::Twist &twist) {
  if (twist.linear.x == 0 && twist.angular.z == 0) {
    leftWheelPID.SetMode(MANUAL);
    rightWheelPID.SetMode(MANUAL);

    turnWheel(0, L_MOTOR_FWD_OR_STOP, L_MOTOR_DIR_PIN);
    turnWheel(0, R_MOTOR_FWD_OR_STOP, R_MOTOR_DIR_PIN);
  } else {
    leftWheelPID.SetMode(AUTOMATIC);
    rightWheelPID.SetMode(AUTOMATIC);
  }

  leftWheelDesiredVelocity = twist.linear.x - twist.angular.z*WHEELBASE;
  rightWheelDesiredVelocity = twist.linear.x + twist.angular.z*WHEELBASE;
}


/***************************** Setup and Loop ******************************************/

void setup() {
  // put your setup code here, to run once:

  //Initialize variables
  leftEncoderLastChangeTime = 0;
  rightEncoderLastChangeTime = 0;
  leftEncoderCount = 0;
  rightEncoderCount = 0;

  leftWheelDesiredVelocity = 0;
  rightWheelDesiredVelocity = 0;
  leftWheelLastDirection = FORWARD;
  rightWheelLastDirection = FORWARD;
  

  encoderVelocityMessage.linear.x = 0;
  encoderVelocityMessage.linear.y = 0;
  encoderVelocityMessage.linear.z = 0;
  encoderVelocityMessage.angular.x = 0;
  encoderVelocityMessage.angular.y = 0;
  encoderVelocityMessage.angular.z = 0;

  //Configure pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(L_MOTOR_PWM_PIN, OUTPUT);
  pinMode(L_MOTOR_DIR_PIN, OUTPUT);
  pinMode(R_MOTOR_PWM_PIN, OUTPUT);
  pinMode(R_MOTOR_DIR_PIN, OUTPUT);
  pinMode(L_ENCODER_PIN, INPUT);
  pinMode(R_ENCODER_PIN, INPUT);

  //Stop motors
  analogWrite(L_MOTOR_PWM_PIN, 0);
  digitalWrite(L_MOTOR_DIR_PIN, LOW);
  analogWrite(R_MOTOR_PWM_PIN, 0);
  digitalWrite(R_MOTOR_DIR_PIN, LOW);

  //Set encoder interrupts
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_PIN), leftEncoderCallback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_PIN), rightEncoderCallback, CHANGE);
  

  //Delay for encoder interrupts to initialize
  delay(500);

  //Initialize PID interrupt timer (Timer 1) to 1Hz
  //Copied from https://www.instructables.com/id/Arduino-Timer-Interrupts/

  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  // When the timer counts to 15624, the interrupt will trigger
  OCR1A = TICK_CMR;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  // This means that the timer will increment once for every 1024 cycles of the Arduino Uno's 16MHz clock
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  leftWheelPID.SetOutputLimits(0, MAX_PWM);
  rightWheelPID.SetOutputLimits(0, MAX_PWM);
  // PIDs are manual when stopped
  leftWheelPID.SetMode(MANUAL);
  rightWheelPID.SetMode(MANUAL);

  nh.initNode();
  nh.advertise(encoderVelocityPub);
  nh.advertise(leftEncoderPub);
  nh.advertise(rightEncoderPub);
  nh.advertise(leftMotorPub);
  nh.advertise(rightMotorPub);

  nh.subscribe(cmdVelSub);
}

void loop() {
  // put your main code here, to run repeatedly:

  nh.spinOnce();
}
