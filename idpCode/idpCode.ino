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
const int followPin1 = xx; //Left
const int followPin2 = xx; //Centre
const int followPin3 = xx; //Right

pinMode(followPin1, INPUT);
pinMode(followPin2, INPUT);
pinMode(followPin3, INPUT);

// Variables initialised
bool followLeftV = 1;
bool followCentreV = 1;
bool followRightV = 1;


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("IDP");

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
  followLeftV = digitalRead(followPin1);
  followCentreV = digitalRead(followPin2);
  followRightV = digitalRead(followPin3);

  while(1):
    serial.println("Pin readings: %d", followLeftV);
    delay(50);

}