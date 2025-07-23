//OUTPUT = (kp * err_val) + (ki * integral) + (kd * dervative)
//OUTPUT = Q [ (kp * err_val) + (ki * (prev_err + err_val)) + (kd * (err_val - prev_err)) ]


//includes
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <AccelStepper.h>

//variables
float kp = 10 ; //proportional
float ki = 0.5 ; //integral
float kd = 1 ; //derivative
float integral = 0 ; //default intergral
float derivative = 0; //default derivative
float prev_err = 0 ; //default
float err_val = 0 ; //default
float setpoint = 0.55; //balanced position x-axis
int interval = 1 ; //interval of 1 miliseconds
float output; // output of the PID controller
const int Q = 100; // constant variable to multiply with PID output for getting motor input values
float speed;
const int max_speed = 3000;

#define MotorL_STEP 5
#define MotorL_DIR 7
#define MotorR_STEP 6
#define MotorR_DIR 8


AccelStepper StepperL(AccelStepper::DRIVER, MotorL_STEP, MotorL_DIR);
AccelStepper StepperR(AccelStepper::DRIVER, MotorR_STEP, MotorR_DIR);
Adafruit_MPU6050 mpu;


void setup(){

  Serial.begin(9600); // start the serial monitor
  while (!Serial) //checking for the connection of mpu6050
    delay(10);
  Serial.println("Adafruit MPU6050 test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  pinMode(4, OUTPUT);
  digitalWrite(4,LOW);

  Serial.println("MPU6050 Found!");

  StepperL.setMaxSpeed(max_speed);
  StepperR.setMaxSpeed(max_speed);

}
void loop(){

  prev_err = err_val;

  /* Get Value of X-axis from MPU6050 and print */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float x = a.acceleration.x;
  Serial.print("Acceleration X: ");
  Serial.print(x);
  Serial.println(" ");

  /* Calculate the PID values */
  err_val = setpoint - x;
  integral += err_val;
  derivative = (err_val - prev_err) / interval;
  output = ( (kp * err_val) + (ki * integral) + (kd * derivative) );
  speed = Q * output;


  /* maxing out the speed if more than max speed */
  if (speed > max_speed){
    speed = max_speed;
  }


  /* running the motors */
  StepperL.setSpeed(-speed);
  StepperR.setSpeed(speed);
  StepperL.runSpeed();
  StepperR.runSpeed();

  // delay for the interval time
  delay(interval);

}

