// Include the adafruit motor shield library
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Initialise the 4 motors (1 and 2 are fast, 3 and 4 are slow)
//Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
//Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(1);
Adafruit_DCMotor *motor4 = AFMS.getMotor(2);

// Pins initalised
const int followPin1 = 8; //LL
const int followPin2 = 9; //LC
const int followPin3 = 10; //RC
const int followPin4 = 11; //RR
const int detectColourPin = 7; 

// Variables initialised
int followPins[4];
int numJunctions = 1; // count down --> if robo reaches a junction and this value is 0, then robo must turn at that junction
// initially (when in the start box), robo must detect 2 junctions
int detection; // value for colour detected
// define number of readings in the last 5 for a junction before action is taken
int requiredJunctReadings = 5;
// define current number of junction readings
int currentJunctReadings = 0;
// define next turn direction
char nextTurn = 'L';

// variables used in lineFollowPID() function
int sensors_average, sensors_sum, position, P, I, D leftMotorSpeed, rightMotorSpeed;
int error =0; // set this starting to 0 in main loop
int P_past =0; // set this starting to 0 in main loop
int set_point;  // need to find this set point --> place bot at center of the line and the position reading (from code in the fucntion) is the set point
float Kp, Ki, Kd;

// defining speed of robot
int speed = 100;
int leftMotorSpeed = speed;
int rightMotorSpeed = speed;

// function to set motor speeds
void set_motors(int mot3speed, int mot4speed)
{   motor3->setSpeed(mot3speed);
    motor3->run(FORWARD);
    motor4->setSpeed(mot4speed);
    motor4->run(FORWARD);
}


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("IDP");
  pinMode(followPin1, INPUT); //Set all four line follower pins as inputs
  pinMode(followPin2, INPUT);
  pinMode(followPin3, INPUT);
  pinMode(followPin4, INPUT);
  pinMode(detectColourPin, INPUT); // set colout detector pin as input

// Check that motor shield has been found
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  set_motors(speed, speed);

  // turn off motors
  //motor1->run(RELEASE);
  //motor2->run(RELEASE);
  //motor3->run(RELEASE);
  //motor4->run(RELEASE);
}


// Define functions

void takeLineReadings() {
  followPins[0] = digitalRead(followPin1);
  followPins[1] = digitalRead(followPin2);
  followPins[2] = digitalRead(followPin3);
  followPins[3] = digitalRead(followPin4);

  for (int i = 0; i < 4; i++) {
    Serial.print(followPins[i]);
  }
  Serial.println();
}

void turn(char dir) {
  switch(dir) {

    case 'F':
      // SET MOTORS TO DRIVE FORWARDS IF NOT ALREADY
      Serial.println("Go forward");
      leftMotorSpeed = speed;
      rightMotorSpeed = speed;
      set_motors(leftMotorSpeed,rightMotorSpeed);
      break;

    case 'L':
      // SET MOTORS TO TURN LEFT IF NOT ALREADY
      // The right hand motor needs to be going faster than left
      Serial.println("Turn left");
      leftMotorSpeed = leftMotorSpeed - 5;
      rightMotorSpeed = rightMotorSpeed + 5;
      set_motors(leftMotorSpeed, rightMotorSpeed);
      break;

    case 'R':
      // SET MOTORS TO TURN RIGHT IF NOT ALREADY
      // The left hand motor needs to be going faster than right
      Serial.println("Turn right");
      leftMotorSpeed = leftMotorSpeed + 5;
      rightMotorSpeed = rightMotorSpeed - 5;
      set_motors(leftMotorSpeed, rightMotorSpeed);
      break;

    case 'C':
      // SET MOTORS TO ROTATE CLOCKWISE
      Serial.println("Rotate clockwise");
      /*       
      leftMotorSpeed = leftMotorSpeed + 20;  // may need to change these values according to distance between wheels
      // and radius of curvature i.e. w = v/r = const for all wheels
      rightMotorSpeed = rightMotorSpeed - 20;
      set_motors(leftMotorSpeed, rightMotorSpeed) */
      break;
    
    case 'A':
    // SET MOTORS TO ROTATE ANTICLOCKWISE
      Serial.println("Rotate anticlockwise");
      break;
  }

}


void lineFollow() {
  if (!followPins[1] && !followPins[2]) { //if all line follow sensors off
    turn('F'); //move forward
  }
  else if (followPins[1] && !followPins[2]) { //if LC sensor on
    turn('L'); // turn left
  }
  else if (!followPins[1] && followPins[2]) { //if RC sensor on
    turn('R'); // turn right
  }
}

void junctionDetect() { // determines whether a junction has ACTUALLY been reached. requiredJunctReadings determines the threshold (I think it's 5?)
  if (followPins[0]&&followPins[1] || followPins[3]&&followPins[2]) { //if all RHS or all LHS sensors detect a line
    currentJunctReadings += 10;
    if (currentJunctReadings > (requiredJunctReadings*10)) {
      currentJunctReadings = 0;
      numJunctions --;
      Serial.println("Junction detected!");
    }
    else if (currentJunctReadings > 0) {
    currentJunctReadings -= 3;
    }
  }
  else if ((followPins[0] && !followPins[1]) || (followPins[3] && !followPins[2]))
  {Serial.println("Strayed from line. Line following no longer correct.");}
  // need to add code for when just LL or just RR detects a line bc this might be an error
}
// line 160 need to add error correction code for when just LL or just RR detects a line bc this 
// indicates that line is no longer at centre of robot



void lineFollowPID(){
  // must do takeLineReadings() before this function
  for (int i = 0; i < 4; i++) {
    sensors_average += followPins[i] * i * 1000;  // calculating weighted mean for PID
    sensors_sum += int(followPins[i]);  // calculating sum of sensor readings
  }
  position = int(sensors_average/sensors_sum);

  // value of kp ki kd is found by testing
  Kp = 1.0;
  Ki = 1.0;
  Kd = 1.0;

  // only really need to run the below when finding the set point ,Kp, Ki ,Kd
  Serial.print(sensors_average);
  Serial.print(' '); 
  Serial.print(sensors_sum); 
  Serial.print(' '); 
  Serial.print(position); 
  Serial.println(); 
  delay(2000);
/* 
  int leftMotorSpeed, rightMotorSpeed;
  int error; // set this starting to 0 in main loop
  int P_past; // set this starting to 0 in main loop
  int set_point;  // need to find this set point --> place bot at center of the line and the position reading (from above) is the set point

  int P = position – set_point; // use modulus of this value!!!
  int I = I + P;
  int D = P - P_past;
  P_past = P;

  error = P*Kp + I*Ki + D*Kd;
  // restricting error value between ±50 --> can change this value to fit
  if (error<-50) {error=-50;}
  if (error>50) {error=50;}


  // If error is less than zero then calc right turn speed values
  if (error<0)
    {
    rightMotorSpeed = speed - error; 
    leftMotorSpeed = speed;
    set_motors(leftMotorSpeed, rightMotorSpeed)
    }

  // If error is greater than zero calc left turn values
  else
    {
    rightMotorSpeed = speed;
    leftMotorSpeed = speed + error;
    set_motors(leftMotorSpeed, rightMotorSpeed)
    }  */
  
}


void moveOutStartBox(){
  numJunctions = 1;
  nextTurn = 'L';
  turn('F'); // robot moves forward
// numJunctions--; //robot acknowledges edge of box as one junction --> numJunction = 0 now
// at next Junction, robo turns left
// by the end of this function, the robot must have turned left and started line following
}

void collectCube(){
// robo should already have turned left and be line following towards the cube
// robo knows how far it is from the cube using the sensor
// cubeIsNear = 1 (or this may be an analogue signal in which case need to test i.e. if (distance<xyz) {cubeIsNear = 1;})
// robo stops
// robo rotates clockwise until white line found again
// by the end of this function, the robot must be line following again
}

void dropOffCube(){
// robot goes into box
// robot reverses and keeps rotating until found white line
// by the end of this function, the robot must be line following again
// numJunctions = 0 if dropped off a blue box
// numJunctions = 2 if dropped off a brown box
}


int blueBoxDetected(){
    Serial.println("Blue box detected.");
    // must count 1 junction to reach the desired drop off spot
    numJunctions = 0; // count down --> if robo reaches a junction and this value is 0, then robo must turn at that junction
    return numJunctions;
}

int brownBoxDetected() {
    Serial.println("Brown box detected.");
    // must count 3 junctions to reach the desired drop off spot
    numJunctions = 2; // count down --> if robo reaches a junction and this value is 0, then robo must turn at that junction
    return numJunctions;
}

void detectColour(){
  int detection = digitalRead(detectColourPin);
  if (detection) {blueBoxDetected();}
  else {brownBoxDetected();}
}

void loop() {
    //detectColour();
    takeLineReadings(); //Default behaviour is to take line readings and follow line accordingly
    lineFollowPID();
    //junctionDetect();
    delay(500);

    /*
    if (start sequence):
      moveOutStartBox();
      start sequence = false;

    if (numJunctions < 0):
      90degreeturn(nextTurn); [then go back to line following]
    */


    // if (start sequence) {
    //   turn(F); // move forwards
    //   if (start box boundary detected) {
    //     // note that box boundary has been found
    //     // wait for a robot to move past the line -- e.g. maybe use ticker, or add a count to the loop?
    //     // then begin line following junction, and set 'currently looking for cross junction' to true
    //   }
    // }
    // if (T junction on RHS detected && currently looking for RHS turn cross junction) { //e.g. looking for box pick up spot, or box drop off spot

    // }
    // if (T junction on LHS detected && currently looking for LHS turn cross junction) { //e.g just out start box,

    // }
    // if (right hand side T junction detected && currently looking for RHS junction) { //e.g. 

    // }

}

