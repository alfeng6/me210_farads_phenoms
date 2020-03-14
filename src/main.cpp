#include <Arduino.h>
#include <Servo.h>

//Initialize Pins
IntervalTimer IRsensorTimer;
IntervalTimer rpmTimer;
Servo wedgeServo;

//MOTOR A
int mtrA_PWM_PIN = 4;
int mtrA_DIR1_PIN = 5;
int mtrA_DIR2_PIN = 6;

//MOTOR B
int mtrB_PWM_PIN = 9;
int mtrB_DIR1_PIN = 7;
int mtrB_DIR2_PIN = 8;

//SENSORS
int IR_PIN = 16;
int RPM_A_PIN = 3;
int RPM_B_PIN = 2;
int WEDGE_PIN = 22;
int LIMIT_LEFT_PIN = 10;
int LIMIT_RIGHT_PIN = 11;

//SERVO
int servoPos = 0;
int initialServoPos = 160;
int finalServoPos = 0;

//Global Variables
int currentReading = 0;
int previousReading = 0;
int IRsensor = 0;
int edgesA = 0;
int edgesB = 0;
float totalAError = 0;
float totalBError = 0;
int motorADutyCycle;
int motorBDutyCycle;
uint32_t endGameTime;

//CONSTANTS
#define GAME_TIME 130000000
#define PWM_FREQ 50000      //unit = hertz
#define SERIAL_BAUD 9600
#define SPIN_CYCLE_SPEED 150
#define DUTY_CYCLE_SPEED 185
#define IR_THRESHOLD 30
#define SPEED_RATIO 1.3
#define IR_PERIOD 500
#define MAX_DUTY_CYCLE 255
#define SECONDS_IN_MIN 60
#define COUNT_PER_REV 1120
#define MAX_MOTOR_SPEED 130
#define RPM_INTERVAL 10000 //unit = microseconds
#define KP_CONSTANT 1.92
#define A_KI_CONSTANT 0.22
#define B_KI_CONSTANT 0.09
#define MAX_PROPORTION 100
#define UPPER_IR_BOUND 200

//Functions
void spinRobot();
void handleGameEnded();
void checkGameEnded();
void handleOrienting();
void moveForwardLeft();
void moveForwardRight();
void IRsensorReading();
void dropWedge();
void pushRightWall();

void updateRPM();
void countFallingEdgesA();
void countFallingEdgesB();

void spinRobotLeft();
void spinRobotRight();

float microsecondsToSeconds(int time);
typedef enum {
  ENDED,
  ORIENTING,
  MOVING_LEFT,
  MOVING_RIGHT,
  PUSHING,
  TURNING_RIGHT,
  TURNING_LEFT
} States_t;

//Global Variables
States_t state;

float microsecondsToSeconds(int time) {
  return (float)time/1000000;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  wedgeServo.attach(WEDGE_PIN);

  //sensor PINs
  pinMode(LIMIT_LEFT_PIN, INPUT);
  pinMode(LIMIT_RIGHT_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(RPM_A_PIN, INPUT);
  pinMode(RPM_B_PIN, INPUT);

  //motor PINs
  pinMode(mtrA_PWM_PIN, OUTPUT);
  pinMode(mtrA_DIR1_PIN, OUTPUT);
  pinMode(mtrA_DIR2_PIN, OUTPUT);
  analogWriteFrequency(mtrA_PWM_PIN, PWM_FREQ);
  pinMode(mtrB_PWM_PIN, OUTPUT);
  pinMode(mtrB_DIR1_PIN, OUTPUT);
  pinMode(mtrB_DIR2_PIN, OUTPUT);
  analogWriteFrequency(mtrB_PWM_PIN, PWM_FREQ);

  //Speed Ratio makes robot travel in a circle
  motorADutyCycle = DUTY_CYCLE_SPEED;
  motorBDutyCycle = DUTY_CYCLE_SPEED*SPEED_RATIO;

  //Game Setup
  IRsensorTimer.begin(IRsensorReading, IR_PERIOD);
  rpmTimer.begin(updateRPM, RPM_INTERVAL);
  rpmTimer.priority(0);
  attachInterrupt(digitalPinToInterrupt(RPM_A_PIN), countFallingEdgesA, FALLING);
  attachInterrupt(digitalPinToInterrupt(RPM_B_PIN), countFallingEdgesB, FALLING);
  wedgeServo.write(initialServoPos);

  //Start Game — Orient
  state = ORIENTING;
  endGameTime = millis() + GAME_TIME;
}
void loop() {

  if (state == ORIENTING) {
    handleOrienting();
  }

  if (state != ENDED) {
    checkGameEnded();
  }
}

/*Spins robot */
void spinRobot() {
  //spin robot
  digitalWrite(mtrA_DIR1_PIN, HIGH);
  digitalWrite(mtrA_DIR2_PIN, LOW);
  digitalWrite(mtrB_DIR1_PIN, LOW);
  digitalWrite(mtrB_DIR2_PIN, HIGH);
  //start motors
  analogWrite(mtrA_PWM_PIN, SPIN_CYCLE_SPEED);
  analogWrite(mtrB_PWM_PIN, SPIN_CYCLE_SPEED);
}

/*Checks if the current time has surpassed the end game time*/
void checkGameEnded() {
  static uint32_t last = millis();
  if (last > endGameTime) {
    handleGameEnded();
  }
}
/*Sets the state to be "ENDED" and stops the motors*/
void handleGameEnded() {
  state = ENDED;
  analogWrite(mtrB_PWM_PIN, 0);
  analogWrite(mtrA_PWM_PIN, 0);
}

/*Reads in the highest signal between the current and previous reading*/
void IRsensorReading() {
  currentReading = analogRead(IR_PIN);
  IRsensor = abs(currentReading - previousReading);
  previousReading = currentReading;
}

/*Spins the robot and checks if the IR Sensor has surpassed the set threshold*/
void handleOrienting() {
  spinRobot();
  
  if (IRsensor > IR_THRESHOLD && IRsensor < UPPER_IR_BOUND) {
    IRsensorTimer.end();
    attachInterrupt(digitalPinToInterrupt(LIMIT_LEFT_PIN), dropWedge, RISING);
    moveForwardLeft();
  }
}

/*Moves the robot left and sets the state to reflect so*/
void moveForwardLeft() {
  digitalWrite(mtrA_DIR1_PIN, HIGH);
  digitalWrite(mtrA_DIR2_PIN, LOW);
  digitalWrite(mtrB_DIR1_PIN, HIGH);
  digitalWrite(mtrB_DIR2_PIN, LOW);
  analogWrite(mtrA_PWM_PIN, motorADutyCycle);
  analogWrite(mtrB_PWM_PIN, motorBDutyCycle);
  state = MOVING_LEFT;
}

/*Moves the robot right and sets the state to reflect so*/
void moveForwardRight() {
  digitalWrite(mtrA_DIR1_PIN, LOW);
  digitalWrite(mtrA_DIR2_PIN, HIGH);
  digitalWrite(mtrB_DIR1_PIN, LOW);
  digitalWrite(mtrB_DIR2_PIN, HIGH);
  analogWrite(mtrA_PWM_PIN, motorADutyCycle);
  analogWrite(mtrB_PWM_PIN, motorBDutyCycle);
  state = MOVING_RIGHT;
}

/*Drops the wedge by lowering the servo motor*/
void dropWedge() {
  detachInterrupt(digitalPinToInterrupt(LIMIT_LEFT_PIN));
  for(servoPos = initialServoPos; servoPos > finalServoPos; servoPos -= 1) {                               
    wedgeServo.write(servoPos);  
  }
  moveForwardRight();
  attachInterrupt(digitalPinToInterrupt(LIMIT_RIGHT_PIN), pushRightWall, RISING);
}

/*Sets the duty cycle for both motors to be 100%*/
void pushRightWall() {
  analogWrite(mtrA_PWM_PIN, MAX_DUTY_CYCLE);
  analogWrite(mtrB_PWM_PIN, MAX_DUTY_CYCLE);
  state = PUSHING;
}

/*Called every RPM_Interval — updates the duty cycle input based on the rpm output using
  integral control code */
void updateRPM() {
  if (state == MOVING_LEFT || state == MOVING_RIGHT) {
    float edges_per_rev = 1/(float)COUNT_PER_REV;
  
    //update Motor A speed
    float setPointRPMA = map(DUTY_CYCLE_SPEED, 0, MAX_DUTY_CYCLE,0, MAX_MOTOR_SPEED);
    float edgesA_per_time = edgesA/microsecondsToSeconds(RPM_INTERVAL);
    float measuredRPMA = edgesA_per_time*edges_per_rev*SECONDS_IN_MIN;
    float rpmAError = setPointRPMA - measuredRPMA;
    totalAError += rpmAError;
    float requestedADuty = KP_CONSTANT*(rpmAError + (A_KI_CONSTANT* totalAError));
    if (requestedADuty > MAX_PROPORTION) {
      requestedADuty = MAX_PROPORTION;
      totalAError -= rpmAError;
    } else if (requestedADuty < 0) {
      requestedADuty = 0;
      totalAError -= rpmAError;
    }
    motorADutyCycle = map(requestedADuty, 0, MAX_PROPORTION, 0, MAX_DUTY_CYCLE);
  
    //update Motor B speed
    float setPointRPMB = map(DUTY_CYCLE_SPEED*SPEED_RATIO, 0, MAX_DUTY_CYCLE,0, MAX_MOTOR_SPEED);
    float edgesB_per_time = edgesB/microsecondsToSeconds(RPM_INTERVAL);
    float measuredRPMB = edgesB_per_time*edges_per_rev*SECONDS_IN_MIN;
    float rpmBError = setPointRPMB - measuredRPMB;
  
    totalBError += rpmBError;
    float requestedBDuty = KP_CONSTANT*(rpmBError + (B_KI_CONSTANT* totalBError));
    if (requestedBDuty > MAX_PROPORTION) {
      requestedBDuty = MAX_PROPORTION;
      totalBError -= rpmBError;
    } else if (requestedBDuty < 0) {
      requestedBDuty = 0;
      totalBError -= rpmBError;
    }
    motorBDutyCycle = map(requestedBDuty, 0, MAX_PROPORTION, 0, MAX_DUTY_CYCLE);
    
    analogWrite(mtrA_PWM_PIN, motorADutyCycle);
    analogWrite(mtrB_PWM_PIN, motorBDutyCycle);

    //update edges
    edgesA = 0;
    edgesB = 0;

  }
}

/*Increases global variable, edgesA, everytime the rpmA output shows a falling edge*/
void countFallingEdgesA() {
  edgesA++;
}

/*Increases global variable, edgesB, everytime the rpmB output shows a falling edge*/
void countFallingEdgesB() {
  edgesB++;
}

 

