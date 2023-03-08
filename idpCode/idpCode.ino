// Include the adafruit motor shield library
#include <Adafruit_MotorShield.h>
#include <Servo.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Initialise the 4 motors (1 and 2 are fast, 3 and 4 are slow)
//Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
//Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(1);
Adafruit_DCMotor *motor4 = AFMS.getMotor(2);

// Pins initalised
const int movementLED = 5; // pin for flashing LED when robot is moving
const int detectColourPinBrown = 6;
const int detectColourPinBlue = 7; 
const int followPin1 = 8; //LL - white
const int followPin2 = 9; //LC - purple
const int followPin3 = 11; //RC - grey (blue sleeve)
const int followPin4 = 12; //RR - orange
const int blockTrigPin = 4; // distance sensor triger - pink
const int blockEchoPin = 3; // distance sensor echo - purple


// initialise servo
Servo myservo;
const int portalRaisedPos = 120; // define the lowered and raised servo positions - needs calibrating in final robot
const int portalLoweredPos = 75;

// Variables initialised
int followPins[4];
int numJunctions = 1; //change back to 2; // count down --> if robo reaches a junction and this value is 0, then robo must turn at that junction
// initially (when in the start box), robo must detect 2 junctions
int detection; // value for colour detected
// define number of readings in the last 5 for a junction before action is taken
int requiredJunctReadings = 5;
// define current number of junction readings
int currentJunctReadings = 0;
// define next turn direction
char nextTurn = 'L';
// variable for robot's current goal
int status = 1; //change back to 0
// variable for storing the movement LED state, so that it can be set in the flashLED() function
int movementLEDstate = 0;
// variable for holding current block colour
String currentBlockColour = "";
// variables for finding block
bool blockFound = false;
long durationInDigit, distanceInCm;
int detectBlockCount = 0;
int threshholdBlockDistance = 5; // distance in cm from block, at which point we can say blockFound=true
bool movedAwayFromLastJunction = true; // variable for junctionDetect() function that will avoid repeat readings from same junction
int notOnJunctionCount = 0; // to ensure that the variable movedAwayFromLastJunction is only reset when the robot has sufficiently cleared the last junction
const int speedAdjustment = 30;


//
//int blockFoundCount = 0;
//int maxBlockFoundCount = 50000000;

// for flashing led
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 50;  // interval at which to blink (milliseconds) - needs to be at 20Hz

// for moving forward
/* unsigned long currentT = millis();
unsigned long previousT =0;
const long timeMoving = 5000;
 */
// defining speed of robot
int speed = 150; //170 seems to be the healthy maximum
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
  pinMode(detectColourPinBlue, INPUT); // set colout detector pins as input
  pinMode(detectColourPinBrown, INPUT);
  pinMode(movementLED, OUTPUT); // set flashing LED pin as output
  pinMode(blockTrigPin, OUTPUT);
  pinMode(blockEchoPin, INPUT);
// attach servo object to pin 10 (maybe is pin 9?)
  myservo.attach(10);

  // setting trigger pin to low (used later in detectBlock function)
  digitalWrite(blockTrigPin, LOW);

// Check that motor shield has been found
   if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  set_motors(speed, speed-speedAdjustment); // error correction for mismatched wheels = 10
  // turn off motors
  //motor1->run(RELEASE);
  //motor2->run(RELEASE);
  motor3->run(FORWARD);
  motor4->run(FORWARD);
}


// Define functions

void takeLineReadings() {
  followPins[0] = digitalRead(followPin1);
  followPins[1] = digitalRead(followPin2);
  followPins[2] = digitalRead(followPin3);
  followPins[3] = digitalRead(followPin4);
  /*Serial.print(followPins[0]);
  Serial.print(followPins[1]);
  Serial.print(followPins[2]);
  Serial.print(followPins[3]);
  Serial.println();*/
}

/* void forwards()
{
  // SET MOTORS TO DRIVE FORWARDS for x seconds=
      unsigned long currentTime = millis();
      if (currentTime - previousTime >= timeMovingForward) 
    {
      // save the last time you blinked the LED
      previousTime = currentTime;
      // if the LED is off turn it on and vice-versa:
      set_motors(speed, speed);
    }  
}

void backwards()
{
  // SET MOTORS TO DRIVE FORWARDS for x seconds
      unsigned long currentTime = millis();
      if (currentTime - previousTime >= timeMovingForward) 
    {
      // save the last time you blinked the LED
      previousTime = currentTime;
      // if the LED is off turn it on and vice-versa:
          motor3->setSpeed(speed);
          motor3->run(FORWARD); // consider BACKWARDS if this doesn't work
          motor4->setSpeed(speed);
          motor4->run(FORWARD);
    }  
} */

void turn(char dir) {

  switch(dir) {

    case 'F':
      if ((leftMotorSpeed != speed) || rightMotorSpeed != speed) {
        leftMotorSpeed = speed;
        rightMotorSpeed = speed;
        set_motors(leftMotorSpeed, rightMotorSpeed-speedAdjustment);
      }
    break;

    case 'B':
      if (leftMotorSpeed != speed) {
        motor3->setSpeed(speed);
        motor3->run(BACKWARD);
        motor4->setSpeed(speed);
        motor4->run(BACKWARD);
    break;

    case 'L':
      // SET MOTORS TO TURN LEFT IF NOT ALREADY
      // The right hand motor needs to be going faster than left

      if (rightMotorSpeed == speed) {
        leftMotorSpeed = 0;
        rightMotorSpeed = 255;
        set_motors(leftMotorSpeed, rightMotorSpeed);
      }

      /*leftMotorSpeed = leftMotorSpeed - 0.1*speed; // changing wheel speed by 10% of original speed 
      rightMotorSpeed = rightMotorSpeed + 0.1*speed; 
      if (rightMotorSpeed >= 255) {rightMotorSpeed=255;}
      if (leftMotorSpeed <= 0) {leftMotorSpeed=0;}
      set_motors(leftMotorSpeed, rightMotorSpeed);*/
      break;

    case 'R':
      // SET MOTORS TO TURN RIGHT IF NOT ALREADY
      // The left hand motor needs to be going faster than right

      if (leftMotorSpeed == speed) {
        leftMotorSpeed = 255;
        rightMotorSpeed = 0;
        set_motors(leftMotorSpeed, rightMotorSpeed);
      }


      /*leftMotorSpeed = leftMotorSpeed + 0.1*speed;  // changing wheel speed by 10% of original speed
      rightMotorSpeed = rightMotorSpeed - 0.1*speed;
      if (leftMotorSpeed >= 255) {leftMotorSpeed=255;}
      if (rightMotorSpeed <= 0) {rightMotorSpeed=0;}
      set_motors(leftMotorSpeed, rightMotorSpeed); */
      break;

    case 'C':
      
      // SET MOTORS TO ROTATE CLOCKWISE
      leftMotorSpeed = 0;
      rightMotorSpeed = speed;
      set_motors(leftMotorSpeed, rightMotorSpeed);
      //delay(100);

      //motor3->setSpeed(speed);  // may need to change these values according to distance between wheels
      // and radius of curvature i.e. w = v/r = const for all wheels
      //motor4->setSpeed(0);  // bit extreme to have this at 0, may change with testing
      //motor3->run(FORWARD);
      //motor4->run(BACKWARD);
      break;
    
    case 'A':
    // SET MOTORS TO ROTATE ANTICLOCKWISE
      rightMotorSpeed = speed;  // may need to change these values according to distance between wheels
      // and radius of curvature i.e. w = v/r = const for all wheels
      leftMotorSpeed = 0;  // bit extreme to have this at 0, may change with testing
      set_motors(leftMotorSpeed, rightMotorSpeed);
      break;


  }

}
}


void lineFollow() {
  if (followPins[0] || followPins[3]) { // if at a junction, then keep moving forwards with highest priority
    turn('F');
  }
  else if (!followPins[1] && !followPins[2]) { //if all line follow sensors off
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
  if ((followPins[0]&&followPins[1]) || (followPins[3]&&followPins[2])) { //if all RHS or all LHS sensors detect a line
    currentJunctReadings += 10;
    notOnJunctionCount = 0; // if junction detected, then reset the notOnJunctionCounter
    if (currentJunctReadings > (requiredJunctReadings*10)) {
      currentJunctReadings = 0;
      if (numJunctions > 0 && movedAwayFromLastJunction) { // don't decrement junction code until robot has moved away from last junction i.e. to avoid multiple readings from same junction
        numJunctions --; // only decrement numJunctions if it is greater than zero
        movedAwayFromLastJunction = false; // to avoid repeat readings of same junction
        Serial.println("Junction detected!");
        Serial.println(numJunctions + " to go");
        notOnJunctionCount = 0;
      }
    }
    else if (currentJunctReadings > 0) {
    currentJunctReadings -= 3;
    }
  }
  else if (!followPins[0] && !followPins[3]) { // only runs when not at a junction of any kind
    notOnJunctionCount ++;
    if (notOnJunctionCount > 20000) { // if 10 readings indicate the robot has cleared the junction, then it is believable
      notOnJunctionCount = 0;
      movedAwayFromLastJunction = true;
      Serial.println("Moved away from junction");
    }
  }
}
// line 160 need to add error correction code for when just LL or just RR detects a line bc this 
// indicates that line is no longer at centre of robot

// LED flash function for when robot is moving
void flashLED() {
  movementLEDstate = !movementLEDstate;
  digitalWrite(movementLED, movementLEDstate);
}

// function for lifting portal frame
void raisePortalFrame() {
  myservo.write(portalRaisedPos);
}

//function for lowering portal frame
void lowerPortalFrame() {
  myservo.write(portalLoweredPos);
}

void detectColour(){
  int brownPin = digitalRead(detectColourPinBrown);
  int bluePin = digitalRead(detectColourPinBlue);
  if (brownPin && bluePin){
    // unsuccessful
  }
  else if (!brownPin && bluePin) {
    currentBlockColour = "brown";
  }
  else {
    currentBlockColour = "blue";
  }
}

void detectBlock(){
  //if block detected, set blockFound=true; else set blockFound=false;
  detectBlockCount ++; // making a count so that the distance to the block is measured every 50 counts
  if (detectBlockCount > 250) { // might have to change this value (larger value --> less frequent readings)
    digitalWrite(blockTrigPin, HIGH); // setting trigger pin to high for 10ms
    delayMicroseconds(10);
    digitalWrite(blockTrigPin, LOW);
    durationInDigit = pulseIn(blockEchoPin, HIGH); // the echo pin detects the signal given by the trigger pin
    distanceInCm = (durationInDigit/5)/29.1; // calculating distance from block - might have to change with testing
    detectBlockCount=0; // resetting count
  }
  if (distanceInCm<threshholdBlockDistance) {
    blockFound = true;}
}

void loop() {
  takeLineReadings(); //Default behaviour is to take line readings and follow line accordingly
  junctionDetect(); // maybe consider that this could cause bugs if outer line sensors are over the white line before it starts line following
  //lineFollow();  
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) 
  {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa:
    flashLED();
  }

/*
  raisePortalFrame();
  Serial.print("raising portal frame");
  delay(5000);
  lowerPortalFrame();
  Serial.print("Lowering portal frame");
  
  delay(5000);
  */



  if (status == 0) { // start sequence - make sure wheels are initially set to move forwards in the setup
    if (numJunctions == 0) { // when numJunctions hits zero i.e. when the main line is reached
      delay(500); // move forwards slightly before spinning anticlockwise
      turn('A'); // (might need to use a different turn function --> need to go forward a bit then turn anticlockwise)
      status = 10;
      takeLineReadings();
    }
  }
  if (status == 10) { // don't stop spinning until line is found
  Serial.println("status 10");
    if (followPins[1]) { // once the LC pin has found the line, set the right number of junctions then go to status 1 where the robot will begin to line follow normally
      status = 1; // MAYBE CHANGE TO 2
      if (currentBlockColour = "brown") { // set the number of junctions depending on whether the robot is just starting, or if it has just dropped off a block
        numJunctions = 3;
      }
      else if (currentBlockColour = "blue") {
        numJunctions = 1;
      }
      else {
        numJunctions = 2;
      }
    }
  }




  if (status == 1) { //line following to block
  Serial.print("status 1 ");
    lineFollow(); //run line follower algorithm
    if (numJunctions == 0) { // turn once at correct junction
      motor3->setSpeed(speed);
      motor4->setSpeed(speed);
      motor3->run(FORWARD);
      motor4->run(BACKWARD);
      delay(1000);
      takeLineReadings();

    //set_motors(speed,speed-speedAdjustment);
    //motor3->run(BACKWARD); // running backwards
    //motor4->run(BACKWARD);
    //delay(1000);
    //takeLineReadings();

    //turn('C');
    //set_motors(speed, speed);
    //delay(500);
    // setting this for robo to turn on the spot

    status = 11;
    }
  }
  if (status == 11) { // don't stop spinning until line is found
      lineFollow();
    //if (followPins[3]) { // once RC pin has seen line, go to status 2 and line follow up to the block
    //  turn('F');
    //  delay(1000);
      
    //  status = 2;
    }
  




  if (status == 2) { // hunting for block along block line
  Serial.print("status 2 ");
  lineFollow();
    //if (!blockFound){ // && blockFoundCount < maxBlockFoundCount) { // use distance sensor to determine whether or not block has been found
      //lineFollow();
      //detectBlock();
      //blockFoundCount ++;
    //}
    //else if (blockFound) { // need distance sensor to determine whether or not block has been found
      //turn('F'); // maybe will need a smaller distance than normal
      //detectColour();
      //lowerPortalFrame();
      
      //delay(1000);

      // setting this for robo to turn on the spot
      //motor3->setSpeed(speed+50);
      //motor4->setSpeed(speed);  
      //motor3->run(FORWARD); // running backwards
      //motor4->run(BACKWARD); // running forwards
      //status = 12; // used to be status 12
      //blockFound=false;
    //}
    }

  if (status == 12) { // keep spinning 180 degrees with block until line is found again
    if (followPins[0]) {
      status = 3;
      numJunctions = 1;
    }
  }




  if (status == 3) { //taking block back to line
  Serial.print("status 3 ");
    lineFollow();
    if (numJunctions == 7) { // once found line, turn left, nb numJunctions ideally = 0
      delay(500);
      turn('A');
      status = 13;
    }
  }
  if (status == 13) {
    if (followPins[1]) {
      status = 4;
      if (currentBlockColour = "blue") {
        numJunctions = 1;
      }
      else {
        numJunctions = 3;
      }
    }
  }


  

  if (status == 4) { // taking block along line to the correct junction for drop off. 
  //Num Junctions was set in the previous code
    lineFollow();
    if (numJunctions == 0) {
      turn('r');
      status = 14;
    }
    }
  if (status == 14) { // check for line while spinning into the turn off zone
    if (followPins[2]) {
      numJunctions = 1;
      status = 5;
    }
  }





  if (status == 5) { // block drop off algorithm
    lineFollow();
    if (numJunctions == 0) {
      turn('F');
      raisePortalFrame(); 
      turn('B');

      // KEEP REVERSING UNTIL THE EDGE OF THE BLOCK IS FOUND – could reverse for x seconds?
      turn('C'); // TURN 180 DEGREES - turn clockwise until one of the middle 2 sensors detects a line, 
      status = 10;
    }
  }

    if (status == 15) { // spin clockwise 180 degrees until sees the return path from block drop off to main line
    if (followPins[2]) {
      lineFollow();
      numJunctions = 1; // may have to change this value according to if robo detects drop off box
      status = 0;
    }
  }


}
