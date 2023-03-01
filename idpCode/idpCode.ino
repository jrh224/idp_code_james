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
const int followPin1 = 8; //LL
const int followPin2 = 9; //LC
const int followPin3 = 10; //RC
const int followPin4 = 11; //RR
const int blockTrigPin = 4; // pin for colour detector 
const int blockEchoPin = 3; // pin for colour detector


// initialise servo
Servo myservo;
const int portalRaisedPos = 0; // define the lowered and raised servo positions - needs calibrating in final robot
const int portalLoweredPos = 90;

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
// variable for robot's current goal
int status = 0;
// variable for storing the movement LED state, so that it can be set in the flashLED() function
int movementLEDstate = 0;
// variable for holding current block colour
string currentBlockColour = null;
// variables for finding block
bool blockFound = false;
long durationInDigit, distanceInCm;
int detectBlockCount = 0;
int threshholdBlockDistance = 5; // distance in cm from block, at which point we can say blockFound=true

// for flashing led
unsigned long previousMillis = 0;  // will store last time LED was updated
const long interval = 50;  // interval at which to blink (milliseconds) - needs to be at 20Hz

// for moving forward
unsigned long previousTime =0;
const long timeMovingForward = 5000;

// defining speed of robot
int speed = 75;
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
  set_motors(speed, speed);
  // turn off motors
  //motor1->run(RELEASE);
  //motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
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

void forwards()
{
  // SET MOTORS TO DRIVE FORWARDS for x seconds
      Serial.println("Go forward");
      unsigned long currentTime = millis();
      if (currentTime - previousTime >= timeMovingForward) 
    {
      // save the last time you blinked the LED
      previousTime = currentTime;
      // if the LED is off turn it on and vice-versa:
      set_motors(speed, speed);
    }  
}

void turn(char dir) {
  switch(dir) {

    case 'F':
      set_motors(speed, speed);

    break;

    case 'L':
      // SET MOTORS TO TURN LEFT IF NOT ALREADY
      // The right hand motor needs to be going faster than left
      Serial.println("Turn left");
      leftMotorSpeed = leftMotorSpeed - 0.1*speed; // changing wheel speed by 10% of original speed 
      rightMotorSpeed = rightMotorSpeed + 0.1*speed; 
      if (rightMotorSpeed >= 255) {rightMotorSpeed=255;}
      if (leftMotorSpeed <= 0) {leftMotorSpeed=0;}
      set_motors(leftMotorSpeed, rightMotorSpeed);
      break;

    case 'R':
      // SET MOTORS TO TURN RIGHT IF NOT ALREADY
      // The left hand motor needs to be going faster than right
      Serial.println("Turn right");
      leftMotorSpeed = leftMotorSpeed + 0.1*speed;  // changing wheel speed by 10% of original speed
      rightMotorSpeed = rightMotorSpeed - 0.1*speed;
      if (leftMotorSpeed >= 255) {leftMotorSpeed=255;}
      if (rightMotorSpeed <= 0) {rightMotorSpeed=0;}
      set_motors(leftMotorSpeed, rightMotorSpeed);
      break;

    case 'C':
      // SET MOTORS TO ROTATE CLOCKWISE
      Serial.println("Rotate clockwise");
            
      leftMotorSpeed = speed;  // may need to change these values according to distance between wheels
      // and radius of curvature i.e. w = v/r = const for all wheels
      rightMotorSpeed = 0;  // bit extreme to have this at 0, may change with testing
      set_motors(leftMotorSpeed, rightMotorSpeed);
      break;
    
    case 'A':
    // SET MOTORS TO ROTATE ANTICLOCKWISE
      Serial.println("Rotate anticlockwise");
      rightMotorSpeed = speed;  // may need to change these values according to distance between wheels
      // and radius of curvature i.e. w = v/r = const for all wheels
      leftMotorSpeed = 0;  // bit extreme to have this at 0, may change with testing
      break;

    case 'l':
      // move forward for x seconds - will have to find x with testing
      turn('f');
      // then turn(A) until LC finds the line 
      turn('A');
      break;

    case 'r':
      // move forward for x seconds - will have to find x with testing
      turn('f');
      // then turn(A) until LC finds the line 
      turn('C');

      break;
    
    case 'f':
      // SET MOTORS TO DRIVE FORWARDS for x seconds
      forwards();
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
  if ((followPins[0]&&followPins[1]) || (followPins[3]&&followPins[2])) { //if all RHS or all LHS sensors detect a line
    currentJunctReadings += 10;
    if (currentJunctReadings > (requiredJunctReadings*10)) {
      currentJunctReadings = 0;
      if (numJunctions > 0) {
        numJunctions --; // only decrement numJunctions if it is greater than zero
      }
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

// LED flash function for when robot is moving
void flashLED() {
  movementLEDstate = !movementLEDstate;
  Serial.print(movementLEDstate);
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
    //detectColour();
    //takeLineReadings(); //Default behaviour is to take line readings and follow line accordingly
    //junctionDetect();
    
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) 
    {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      // if the LED is off turn it on and vice-versa:
      flashLED();
    }

    raisePortalFrame();
    Serial.print("raising portal frame");
    delay(10000);
    lowerPortalFrame();
    Serial.print("Lowering portal frame");
    
    delay(10000);
 
    if (status == 0) { // start sequence - make sure wheels are initially set to move forwards in the setup
      if (numJunctions == 0) { // when numJunctions hits zero i.e. when the main line is reached
        turn('l'); // (might need to use a different turn function --> need to go forward a bit then turn anticlockwise)
        status = 10;
      }
    }
    if (status == 10) { // don't stop spinning until line is found
      if (followPins[1]) { // once the LC pin has found the line, set the right number of junctions then go to status 1 where the robot will begin to line follow normally
        status = 1;
        numJunctions = 2;
      }
    }
    if (status == 1) { //line following to block
      lineFollow(); //run line follower algorithm
      if (numJunctions == 0) { // turn once at correct junction
        turn('r');
        status = 11;
      }
    }
    if (status == 11) { // don't stop spinning until line is found
      if (followPins[2]) { // once RC pin has seen line, go to status 2 and line follow up to the block
        status = 2;
      }
    }
    if (status == 2) { // hunting for block along block line
      if (!blockFound && count < max_count) { // use distance sensor to determine whether or not block has been found
        lineFollow();
        
        detectBlock();
        count ++;
      }
      else if (blockFound) { // need distance sensor to determine whether or not block has been found
        turn('f'); // maybe will need a smaller distance than normal
        detectColour();
        lowerPortalFrame();
        turn('C');
        status = 12;
        blockFound=false;
      }
      else {
        ;
        // ADD CONTINGENCY FOR IF BLOCK ISN'T FOUND
        // set a timer, if time has gone above a value, block could reverse???
      }
    }
    if (status == 12) { // keep spinning 180 degrees with block until line is found again
      if (followPins[2]) {
        status = 3;
        numJunctions = 1;
      }
    }
    if (status == 3) { //taking block back to line
      lineFollow();
      if (numJunctions == 0) { // once found line, turn left
        turn('l');
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

    // CONTINUE HERE

    
    if (status == 4) { // taking block along line to the correct junction for drop off. 
    //Num Junctions was set in the previous code, so this applies regardless of block colour
      lineFollow();
      if (numJunctions == 0) {
        turn('r'); // into the drop off box
        numJunctions = 1;
        status = 5;
      }
      
    }
    if (status == 5) {
      lineFollow();
      if (numJunctions = 0) {
        ;
        forwards(); // MOVE FORWARDS A LITTLE BIT MORE
        // LIFT UP PORTAL FRAME
        // REVERSE TO LEAVE BLOCK BEHIND
        // KEEP REVERSING UNTIL THE EDGE OF THE BLOCK IS FOUND – could reverse for x seconds?
        // TURN 180 DEGREES - turn clockwise until one of the middle 2 sensors detects a line, 
        // MOVE FORWARDS UNTIL LINE DETECTED
        // TURN LEFT
        if (detection) { // set number of junctions for return journey depending on which block was deposited
          numJunctions = 1;
        }
        else {
          numJunctions = 3;
        }
        status = 1;
      }
    }


}




/*
// variables used in lineFollowPID() function
int pid_output, P, I, D;
//int leftMotorSpeed, rightMotorSpeed;
int error =0; // set this starting to 0 in main loop
int last_error =0; // set this starting to 0 in main loop
float Kp, Ki, Kd;

void lineFollowPID(){
  // must do takeLineReadings() before this function
  // calculating the error
  error += followPins[0] * 2 
                      + followPins[1] * 1
                      + followPins[2] * -1
                      + followPins[3] * -2;  // calculating mean for PID
  //error /= 1; //sensor readings are in range 0-1023, so dividing error by 100 
  //scales error value to a range of approx -10-10, which is more reasonable

  //if (error * last_error < 0) {I=0;} // if the error crosses 0, then set I
  // to zero, to remove integral wind up

  // value of kp ki kd is found by testing
  Kp = 0.05; // proportionality
  Ki = 0; // integral
  Kd = 0.0; // derivative

// calculating PID output
  P = error;
  I = I + P;
  D = error - last_error;
  pid_output = P*Kp + I*Ki + D*Kd;
  last_error = error;

  // restricting error value between ±50 --> can change this value to fit
  if (pid_output<-50) {pid_output=-50;}
  if (pid_output>50) {pid_output=50;}

  Serial.println(pid_output); 

  //pid_output*=0.25;
  // adjusting speed of each motor by the pid_output
 // if (rightMotorSpeed == speed)
 { 
  rightMotorSpeed = speed + pid_output; 
  Serial.println("right motor speed");
  Serial.println(rightMotorSpeed);}

 // if (leftMotorSpeed ==speed)
 { 
  leftMotorSpeed = speed - pid_output;
  Serial.println("left motor speed");
  Serial.println(leftMotorSpeed);}
  
  set_motors(leftMotorSpeed, rightMotorSpeed);
    
}
*/
