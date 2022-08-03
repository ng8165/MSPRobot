#include "Romi_Motor_Power.h"
#include "RSLK_Pins.h"
#include "SimpleRSLK.h"
#include <driverlib.h>
#include <stdlib.h>
#include <stdio.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "Adafruit_TMP006.h"

// TEMPERATURE VARIABLES
int count = 0;
Adafruit_TMP006 tmp006;

// WIFI METHODS AND VARIABLES

char ssid[] = "eec172";
char server[] = "io.adafruit.com";
WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);
uint8_t status;

void callback(char* topic, byte* payload, unsigned int length) {
  char* str = (char*) payload;
  status = str[0] - '0';
}

void pollBroker() {
  while (!client.connected()) {
    Serial.print("Connecting... ");

    if (!client.connect("energiaClient", "ryanachen", "aio_ALPe40WNq4C5A7Hz089DIJA0zFo5")) {
      Serial.println("Connection failed.");
    } else {
      Serial.print("Connection successful. ");
      
      if (client.subscribe("ryanachen/feeds/msp-robot-direction")) {
        Serial.println("Subscription successful.");
        break;
      } else {
        Serial.println("Subscription failed.");
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

  left_motor.setSpeed(abs(lSpeed));
  right_motor.setSpeed(abs(rSpeed));
}

void moveLeft() { lSpeed = -15; rSpeed = 15; moveBot(); }
void moveRight() { lSpeed = 15; rSpeed = -15; moveBot(); }
void moveForward() { lSpeed = 20; rSpeed = 20; moveBot(); }
void moveBackward() { lSpeed = -20; rSpeed = -20; moveBot(); }
void stopBot() { lSpeed = 0; rSpeed = 0; moveBot(); }

void setup() {
  WDT_A_hold(WDT_A_BASE);
  
  Serial.begin(115200);
  
  left_motor.begin(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN);
  right_motor.begin(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN);

  stopBot();

  left_motor.enableMotor();
  right_motor.enableMotor();

  wifiInit();

  if(!tmp006.begin(TMP006_CFG_8SAMPLE)){
    Serial.println("No sensor found");
    while(1);
  }
  
}

void loop() {
  pollBroker();

  switch (status) {
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
  
  if(count >= 6){
    
    float temp = tmp006.readDieTempC();
    temp = (temp * 9/5) + 32;
    
    char str[50];
    sprintf(str, "%0.2f", temp);
    client.publish("ryanachen/feeds/msp-robot.temperature", str);

    count = 0;
  }
  count++;
  
  delay(500);
}
