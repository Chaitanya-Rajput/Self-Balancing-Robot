#include <Arduino.h>
#include <MPU6050_light.h>
#include <AccelStepper.h>

#define sampleTime 20
#define kp 230
#define ki 20
#define kd 70
#define maxSpeed 15000

AccelStepper stepper_right(AccelStepper::FULL2WIRE, 18, 19);
AccelStepper stepper_left(AccelStepper::FULL2WIRE, 32, 33);

MPU6050 mpu(Wire);

long timer = 0;
float angle;
float prevAngle = 0;
float sumAngle = 0;
float currentSpeed = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(800000);
  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050
  stepper_right.setMaxSpeed(10000);
  stepper_left.setMaxSpeed(10000);
  mpu.setAccOffsets(-0.06, 0.005, 0.05);
  mpu.setGyroOffsets(0, 4.32, 0);

}

void loop() {
  if(millis() - timer > sampleTime){ // print data every second
    mpu.update();
    angle = mpu.getAngleY();
    angle += 1.5;
    sumAngle += angle;
    currentSpeed = angle * kp + (angle - prevAngle) * kd + sumAngle * ki;
    if(currentSpeed > maxSpeed) currentSpeed = maxSpeed;
    if(currentSpeed < -maxSpeed) currentSpeed = -maxSpeed;
    prevAngle = angle;
    timer = millis();
  }
  stepper_right.setSpeed(currentSpeed);
  stepper_left.setSpeed(currentSpeed);
  stepper_right.runSpeed();
  stepper_left.runSpeed();
  
}
