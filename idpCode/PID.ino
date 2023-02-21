//Program to define the arduino pin numbers connected to motor

//defined 4 motors

// Variables initialised
int followPins = {0,0,0,0};  // array to store IR sensor values


int last_proportional = 0;
int integral = 0;



//function prototype of different functions

// function to decide which direction to turn
char select_turn(unsigned char found_left, unsigned char found_right, unsigned char found_st);  

int mod(int v); // v is a weighted average of where the line is relative to the sensors
// with 4 sensors, v=0 when travelling in a straight line

// setting motor speeds
int set_motors(int a, int b);

void turn(char dir);

void PID();

int right = 0;
int left = 0;


int set_motors(int mot3speed, int mot4speed)
{   myMotor3->setSpeed(mot3speed);
    myMotor4->setSpeed(mot4speed);
}


void PID()     // PID function definition
// Variable initialization
  {
    int i;
    int power_difference = 0;
    float Kp, Ki, Kd;
    unsigned int position;
    int derivative,proportional;
    while(1) 


    takeLineReadings();
    position = followPins;            //reading the sensor value and position is a list of digital readings


     //replace value 2000 with your position by placing your robot at the dead centre and read the value

    proportional = ((int)position - 2000);       
    derivative = proportional - last_proportional;

    integral = integral+proportional;

    last_proportional = proportional;

  // value of kp ki kd, you have to make changes to make your robot move without oscillation (hit and trial method)
    Kp = 0.08;
    Ki = 0.0002;
    Kd = 1.0;

  //formula for pid to calculate error (i.e power difference)

    power_difference = proportional * Kp + integral * Ki + derivative *Kd;

    const int max = 200;  //setting the maximum speed of motor

    if(power_difference>max)

     power_difference = max;

    if(power_difference < -max)

      power_difference = (-1*max);

      if(power_difference < 0)

      {

        //set_motors(max, max-power_difference);

        set_motors(max+power_difference, max);

      }

      else

      {

        //set_motors(max+power_difference, max);

        set_motors(max, max-power_difference);

      }

      readline();

      if(a[0] == LOW && a[1] == LOW && a[2] == LOW && a[3] == LOW && a[4] == LOW)

      return;

      else if(a[0]== HIGH || a[4] == HIGH)

      return;

  }