#include "Romi_Motor_Power.h"
#include "RSLK_Pins.h"
#include "SimpleRSLK.h"
#include <stdlib.h>

Romi_Motor_Power left_motor;
Romi_Motor_Power right_motor;

int lSpeed;
int rSpeed;

//allows negative speed values by changing motor direction
void moveBot() {
  
  if(lSpeed >= 0) left_motor.directionForward();
  else left_motor.directionBackward();

  if(rSpeed >= 0) right_motor.directionForward();
  else right_motor.directionForward();

  left_motor.setSpeed(abs(lSpeed));
  right_motor.setSpeed(abs(rSpeed));
  
}

void setup() {
  // put your setup code here, to run once:
  
  left_motor.begin(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);

  right_motor.begin(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);
}

void loop() {
  
  // put your main code here, to run repeatedly: 
  
  left_motor.enableMotor();
  right_motor.enableMotor();

  lSpeed = 30;
  rSpeed = 30;
  moveBot();
  
  delay(5000);

  lSpeed = 35;
  rSpeed = 25;
  moveBot();
  
  delay(5000);

  
  left_motor.disableMotor();
  right_motor.disableMotor();
  delay(20000);
}
