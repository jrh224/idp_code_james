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
const int followPin1 = 8; //LL
const int followPin2 = 9; //LC
const int followPin3 = 10; //RC
const int followPin4 = 11; //RR


// Variables initialised
int followPins[4];

// defining speed of robot
int speed = 100;

// function to set motor speeds
int set_motors(int mot3speed, int mot4speed)
{   motor3->setSpeed(mot3speed);
    motor4->setSpeed(mot4speed);
}


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
  set_motors(speed, speed);

  // turn off motors
  motor1->run(RELEASE);
  motor2->run(RELEASE);
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

void turn(char dir) {

  int leftMotorSpeed, rightMotorSpeed;
  switch(dir) {

    case 'F':
      // SET MOTORS TO DRIVE FORWARDS IF NOT ALREADY
      Serial.println("Go forward");
      leftMotorSpeed = speed;
      rightMotorSpeed = speed;
      set_motors(leftMotorSpeed,rightMotorSpeed)
      break;

    case 'L':
      // SET MOTORS TO TURN LEFT IF NOT ALREADY
      // The right hand motor needs to be going faster than left
      Serial.println("Turn left");
      leftMotorSpeed = leftMotorSpeed – 5;
      rightMotorSpeed = rightMotorSpeed + 5;
      set_motors(leftMotorSpeed, rightMotorSpeed)
      break;

    case 'R':
      // SET MOTORS TO TURN RIGHT IF NOT ALREADY
      // The left hand motor needs to be going faster than right
      Serial.println("Turn right");
      leftMotorSpeed = leftMotorSpeed + 5;
      rightMotorSpeed = rightMotorSpeed - 5;
      set_motors(leftMotorSpeed, rightMotorSpeed)
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


void lineFollowPID(){
  
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

  /*
  // only really need to run the below when finding the set point ,Kp, Ki ,Kd
  Serial.print(sensors_average);
  Serial.print(' '); 
  Serial.print(sensors_sum); 
  Serial.print(' '); 
  Serial.print(position); 
  Serial.println(); 
  delay(2000); */

  int leftMotorSpeed, rightMotorSpeed;
  int error; // set this starting to 0 in main loop
  int P_past; // set this starting to 0 in main loop
  int set_point;  // need to find this set point --> place bot at center of the line and the position reading (from above) is the set point

  int P = position – set_point;
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
  
}


void moveOutStartBox(){

}

void skipJunction(){

}

void followJunction(){

}

void loop() {

    takeLineReadings(); //Default behaviour is to take line readings and follow line accordingly
    lineFollow();
    delay(500);

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

