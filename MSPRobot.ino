#include "Romi_Motor_Power.h"
#include "RSLK_Pins.h"
#include "SimpleRSLK.h"

Romi_Motor_Power left_motor;
Romi_Motor_Power right_motor;

void setup() {
  // put your setup code here, to run once:
  
  left_motor.begin(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);

  right_motor.begin(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);
}

void loop() {
  // put your main code here, to run repeatedly: 
  
  left_motor.enableMotor();
  right_motor.enableMotor();

  left_motor.directionForward();
  right_motor.directionForward();

  delay(2000);

  left_motor.setSpeed(5);
  right_motor.setSpeed(5);

  delay(5000);

  left_motor.disableMotor();
  right_motor.disableMotor();
}
