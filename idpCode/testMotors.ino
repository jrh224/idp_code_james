// /*
// This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
// It won't work with v1.x motor shields! Only for the v2's with built in PWM
// control

// For use with the Adafruit Motor Shield v2
// ---->	http://www.adafruit.com/products/1438
// */

// #include <Adafruit_MotorShield.h>

// // Create the motor shield object with the default I2C address
// Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// // Or, create it with a different I2C address (say for stacking)
// // Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// // Select which 'port' M1, M2, M3 or M4. In this case, M1
// Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
// Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

// void setup() {
//   Serial.begin(9600);           // set up Serial library at 9600 bps
//   Serial.println("Adafruit Motorshield v2 - DC Motor test!");

//   if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
//   // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
//     Serial.println("Could not find Motor Shield. Check wiring.");
//     while (1);
//   }
//   Serial.println("Motor Shield found.");

// }

// void loop() {
//   uint8_t i;

//     // Set the speed to start, from 0 (off) to 255 (max speed)
//   myMotor3->setSpeed(150);
//   myMotor3->run(FORWARD);

//   myMotor4->setSpeed(150);
//   myMotor4->run(FORWARD);

//   Serial.print("tick");

//   Serial.print("tech");
//   //myMotor3->run(RELEASE);
//   delay(1000);
// }