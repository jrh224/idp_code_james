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
// variable for robot's current goal
int status = 0;

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



/* void lineFollowPID(){
  // must do takeLineReadings() before this function
  for (int i = 0; i < 4; i++) {
    int sensors_average += followPins[i] * i * 1000;  // calculating weighted mean for PID
    int sensors_sum += int(followPins[i]);  // calculating sum of sensor readings
  }
  int position = int(sensors_average/sensors_sum);

  float Kp, Ki, Kd;
  // value of kp ki kd is found by testing
  Kp = 1.0;
  Ki = 1.0;
  Kd = 1.0;
 */
  /*
  // only really need to run the below when finding the set point ,Kp, Ki ,Kd
  Serial.print(sensors_average);
  Serial.print(' '); 
  Serial.print(sensors_sum); 
  Serial.print(' '); 
  Serial.print(position); 
  Serial.println(); 
  delay(2000); */

/*   int leftMotorSpeed, rightMotorSpeed;
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
    } 
  
} */


void moveOutStartBox(){
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


void blueBoxDetected(){
    Serial.println("Blue box detected.");
    // must count 1 junction to reach the desired drop off spot
    numJunctions = 1; // count down --> if this value hits 0, then robo must turn at that junction
}

void brownBoxDetected() {
    Serial.println("Brown box detected.");
    // must count 3 junctions to reach the desired drop off spot
    numJunctions = 3; // count down --> if this value hits 0, then robo must turn at that junction
}

void detectColour(){
  int detection = digitalRead(detectColourPin);
  if (detection) {blueBoxDetected();}
  else {brownBoxDetected();}
}

void loop() {
    //detectColour();
    takeLineReadings(); //Default behaviour is to take line readings and follow line accordingly
    lineFollow();
    //junctionDetect();
    delay(500);

    if (status == 0) { // start sequence
      if (numJunctions == 0) { // when numJunctions hits zero i.e. when the main line is reached
        turn(left); // (might need to use a different turn function)
        status = 1;
        numJunctions = 2;
      }
    }
    if (status == 1) { //line following to block
      PID(); //run PID line follower algorithm
      if (numJunctions == 0) { // turn once at correct junction
        turn(right);
        status = 2;
      }
    }
    if (status == 2) { // turning off line to hunt for block
      turn(right);
      while (block not found && count < max_count) {
        PID();
        count ++;
      }
      if (block found) {
        turn(180degrees);
        status = 3;
      }
      else {
        ;
        // ADD CONTINGENCY FOR IF BLOCK ISN'T FOUND
      }
    }
    if (status == 3) { //taking block back to line
      
      PID();
      if (numJunctions == 0) { // once found line, turn left
        turn(left);
        detectColour(); // *this contains a 'set number of junctions' command* - if there are issues, maybe try running detectColour early on when the block is first found. Be careful though since this will mess up the numJunctions for finding the line again
        status = 4;
      }
    }
    if (status == 4) { // taking block along line to the correct junction for drop off. Num Junctions was set in the previous code, so this applies regardless of block colour
      PID();
      if (numJunctions == 0) {
        turn(right);
        numJunctions = 1;
        status = 5;
      }
    }
    if (status == 5) {
      PID();
      if (numJunctions = 0) {
        ;
        // MOVE FORWARDS A LITTLE BIT MORE
        // REVERSE TO LEAVE BLOCK BEHIND
        // KEEP REVERSING UNTIL THE EDGE OF THE BLOCK IS FOUND
        // TURN 180 DEGREES
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

