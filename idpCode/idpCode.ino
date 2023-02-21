// Include the adafruit motor shield library
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Initialise the 4 motors (1 and 2 are fast, 3 and 4 are slow)
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

// Pins initalised
const int followPin1 = 9; //LL
const int followPin2 = 10; //LC
const int followPin3 = 11; //RC
const int followPin4 = 12; //RR



// Variables initialised
int followPins = {0,0,0,0};

// defining speed of robot
int speed = 100;


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("IDP");
  pinMode(followPin1, INPUT); //Set all four line follower pins as inputs
  pinMode(followPin2, INPUT);
  pinMode(followPin3, INPUT);
  pinMode(followPin4, INPUT);

// Check that motor shield has been found
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  motor3->setSpeed(speed);
  motor4->setSpeed(speed);
  // turn off motors
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
}

void loop() {


  // obtain digital reading from each pin
  while(1) {
    takeLineReadings();

    switch (status) {
      case "find new block":
        findNewBlock();
    }
  }

  if (followleftV==0 && followCentreV==1 && followRightV==0) {
    moveForward;
    Serial.println("moving forward");
    delay(100);
  }
  
  ///else if (followleftV==1 && followCentreV==0 && followRightV==0) {turnLeft;}
  // else if (followleftV==0 && followCentreV==0 && followRightV==1) {turnRight;}
  // would have more for other situations
}

// Define functions

void takeLineReadings() {
  followPins[0] = digitalRead(followPin1);
  followPins[1] = digitalRead(followPin2);
  followPins[2] = digitalRead(followPin3);
  followPins[3] = digitalRead(followPin4);

  Serial.println(followPins);
}

void moveForward() {
    
  Serial.println("Moving forward");
}

void moveBackward() {

}

void turn(dir) {

  switch(dir) {

    case 'F':
      // SET MOTORS TO DRIVE FORWARDS

    case 'L':

      // SET MOTORS TO TURN LEFT
      break;

    case 'R':

      // SET MOTORS TO TURN RIGHT
      break;
  }

}


void stopMoving() {

}

void lineFollow() {
  if (!followPins[1] && !followPins[2]) { //if all line follow sensors off
    turn('F');
  }
  else if (followPins[1] && !followPins[2]) {
    turn('L');
  }
  else if (!followPins[1] && followPins[2]) {
    turn('R');
  }
}