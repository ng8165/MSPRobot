#include "Romi_Motor_Power.h"
#include "RSLK_Pins.h"
#include "SimpleRSLK.h"
#include <driverlib.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

int count = 0;
int toggle = 0;

// WIFI METHODS AND VARIABLES

char ssid[] = "eec172";
char server[] = "io.adafruit.com";
WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);
uint8_t direction, speed;

void callback(char* topic, byte* payload, unsigned int length) {
  char* str = (char*) payload;
  
  if (strcmp(topic, "ryanachen/feeds/msp-robot-direction") == 0) {
    direction = str[0] - '0';
  } else if (strcmp(topic, "ryanachen/feeds/speed") == 0) {
    speed = str[0] - '0';
  }
}

void pollBroker() {
  while (!client.connected()) {
    Serial.print("Connecting... ");

    if (!client.connect("energiaClient", "ryanachen", "aio_uWnL05y5P3xSEhlR4PUiSAn5AkUz")) {
      Serial.println("Connection failed.");
    } else {
      Serial.print("Connection successful. ");
      
      if (client.subscribe("ryanachen/feeds/msp-robot-direction")) {
        Serial.print("Direction successful. ");

        if (client.subscribe("ryanachen/feeds/speed")) {
          Serial.println("Speed successful.");
          break;
        } else {
          Serial.println("Speed failed.");
        }
      } else {
        Serial.println("Direction failed.");
      }
    }

    delay(1000);
  }

  client.poll();
}

void wifiInit() {
  // try to connect to network
  Serial.print("Attempting to connect to network named: ");
  Serial.print(ssid);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  // wait for IP address
  Serial.println("\nNow connected to a network.");
  Serial.print("Waiting for an IP address");
  while (WiFi.localIP() == INADDR_NONE) {
    Serial.print(".");
    delay(300);
  }

  // print wifi status
  Serial.println("\nIP Address obtained.\n");
}

// ROBOT MOTOR METHODS AND VARIABLES

Romi_Motor_Power left_motor;
Romi_Motor_Power right_motor;
int8_t lSpeed, rSpeed;

// allows negative speed values by changing motor direction
void moveBot() {
  if (lSpeed >= 0) left_motor.directionForward();
  else left_motor.directionBackward();

  if (rSpeed >= 0) right_motor.directionForward();
  else right_motor.directionBackward();
  
  left_motor.setSpeed(speed*abs(lSpeed));
  right_motor.setSpeed(speed*abs(rSpeed));
}

void moveLeft() { lSpeed = -10; rSpeed = 10; moveBot(); }
void moveRight() { lSpeed = 10; rSpeed = -10; moveBot(); }
void moveForward() { lSpeed = 20; rSpeed = 20; moveBot(); }
void moveBackward() { lSpeed = -20; rSpeed = -20; moveBot(); }
void stopBot() { lSpeed = 0; rSpeed = 0; moveBot(); }

void setup() {
  WDT_A_hold(WDT_A_BASE);
  
  Serial.begin(115200);
  
  left_motor.begin(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);
  right_motor.begin(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);
  
  pinMode(BP_SW_PIN_0, INPUT_PULLUP);
  pinMode(BP_SW_PIN_1, INPUT_PULLUP);
  pinMode(BP_SW_PIN_2, INPUT_PULLUP);
  pinMode(BP_SW_PIN_3, INPUT_PULLUP);
  pinMode(BP_SW_PIN_4, INPUT_PULLUP);
  pinMode(BP_SW_PIN_5, INPUT_PULLUP);

  stopBot();
  speed = 1;
  left_motor.enableMotor();
  right_motor.enableMotor();

  wifiInit();
}

void loop() {
  pollBroker();
  
  switch (direction) {
    case 0:
      stopBot();
      break;
    case 1:
      moveForward();
      break;
    case 2:
      moveBackward();
      break;
    case 3:
      moveLeft();
      break;
    case 4:
      moveRight();
      break;
  }
  
  if (digitalRead(BP_SW_PIN_0) == 0 ||
      digitalRead(BP_SW_PIN_1) == 0 ||
      digitalRead(BP_SW_PIN_2) == 0 ||
      digitalRead(BP_SW_PIN_3) == 0 ||
      digitalRead(BP_SW_PIN_4) == 0 ||
      digitalRead(BP_SW_PIN_5) == 0) {
    Serial.println("Collision detected");
    stopBot();
  }
  
  Serial.print(direction);
  Serial.print(" ");
  Serial.println(speed);

  
  int tempSensor = analogRead(A19);
  float tempC = ((3.3*tempSensor/1023.0))/0.01;
  float tempF = tempC*(9.0/5.0)+32.0;
  Serial.println(tempF); // degrees Fahrenheit

  int aqiSensor = analogRead(A18);
  Serial.println(aqiSensor); // ppm CO2

  
  if(count >= 6){

    if(toggle == 0){
      char str[50];
      sprintf(str, "%0.2f", tempF);
      if(client.publish("ryanachen/feeds/msp-robot.temperature", str)){
        Serial.println("Temp Publish Succeeded");
      }
      toggle = 1;
    }else if(toggle == 1){
      char str[50];
      sprintf(str, "%d", aqiSensor);
      if(client.publish("ryanachen/feeds/msp-robot.air-quality", str)){
        Serial.println("AQ Publish Succeeded");
      }
      toggle = 0;
    }

    count = 0;
  }
  count++;
  
  
  delay(500);
}
