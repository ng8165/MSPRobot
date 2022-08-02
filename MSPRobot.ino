#include "Romi_Motor_Power.h"
#include "RSLK_Pins.h"
#include "SimpleRSLK.h"
#include <driverlib.h>
#include <stdlib.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

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

    if (!client.connect("energiaClient", "ng8165", "aio_YOoT86vFIEE2dhRchNNUb47Fen7j")) {
      Serial.println("Connection failed.");
    } else {
      Serial.print("Connection successful. ");
      
      if (client.subscribe("ng8165/feeds/robot.direction")) {
        Serial.println("Subscription successful.");
        break;
      } else {
        Serial.println("Subscription failed.");
      }
    }

    delay(1000);
  }

  client.poll();
  
  delay(100);
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
  else right_motor.directionForward();

  left_motor.setSpeed(abs(lSpeed));
  right_motor.setSpeed(abs(rSpeed));
}

void moveLeft() { lSpeed = -20; rSpeed = 20; moveBot(); }
void moveRight() { lSpeed = 20; rSpeed = -20; moveBot(); }
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
}

void loop() {
  Serial.println(status);
  delay(500);
}
