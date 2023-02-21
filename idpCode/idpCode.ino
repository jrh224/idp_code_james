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
int followLL = 1;
int followLC = 1;
int followRC = 1;
int followRR = 1;

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
  motor3->setSpeed(150);
  motor4->setSpeed(150);
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
  followLL = digitalRead(followPin1);
  Serial.println("left sensor: "+followLL);
  delay(100);

  followLC = digitalRead(followPin2);
  Serial.println("central sensor: "+followLC);
  delay(100);

  followRC = digitalRead(followPin3);
  Serial.println("right sensor: "+followRC);
  delay(100);

  followRR = digitalRead(followPin4);
  Serial.println("right sensor: "+followRR);
  delay(100);
}

void moveForward() {

}

void turnLeft() {

}

void turnRight() {

}

void stopMoving() {

}

void lineFollow() {
  
}