/**
 * Incubator Controller V2
 *
 * Author: Tony Kambourakis
 * License: Apache License v2
 */

extern "C" {
  #include "user_interface.h"
  #include "gpio.h"
}
//#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include "SensitiveConfig_incubatorcontroller.h"
#include <Time.h>

// void init_lcd();
// void configureOTA();
// void init_wifi();
// void initTimers();
// void environmentTimerFinished(void *pArg);
// void trayTiltTimerFinished(void *pArg);
// void readDHTSensor();
// void connectWithBroker();
// void publishPayload();
// void displayLCD();
// void debugDisplayPayload();
void callback(char* topic, byte* payload, unsigned int length);

// Configure 2 line LCD display with I2C interface

LiquidCrystal_I2C lcd(0x27,16,2);
//LiquidCrystal_I2C lcd(0x27,20,4);

// Timers

os_timer_t environmentTimer;
bool isEnvironmentTimerComplete = false;

os_timer_t trayTiltTimer;
bool isTrayTiltTimerComplete = false;


// State Machine variables

enum SystemState {
  IDLE,
  PUBLISHING_ENVIRONMENT_EVENT,
  TILTING_TRAY_LEFT,
  TILTING_TRAY_RIGHT,
  OTA_IN_PROGRESS,
  MOTOR_SWITCH_DETECTED
};

SystemState state = IDLE;
// WiFI credentials

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// IBM Internet of Things MQTT configuration

//#define ORG "ow3qa2"
//#define DEVICE_TYPE "AmicaNodeMCU"
//#define DEVICE_ID "LB01"
//#define TOKEN "d8ehyDPvxjWVI1n6Kn"

char server[] = IOT_ORG IOT_BASE_URL;
char topic[] = "iot-2/evt/status/fmt/json";
char debugTopic[] = "iot-2/evt/status/fmt/debug";
char authMethod[] = "use-token-auth";
char token[] = IOT_TOKEN;
char clientId[] = "d:" IOT_ORG ":" IOT_DEVICE_TYPE ":" IOT_DEVICE_ID;

// Global Variables

WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);

uint16_t volts;

// Temperature and Humidity Sensor (DHT22) Configuration

#define DHTPIN D2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Motor

#define DIRA 0 // D3 = GPIO0
#define PWMA 5 // D1 = GPIO5

// Switches

#define PORT_MOTOR_SWITCH_LEFT 13 // D7 = GPIO13
#define PORT_MOTOR_SWITCH_RIGHT 12 // D6 = GPIO12
#define MOTOR_SWITCH_LEFT 0
#define MOTOR_SWITCH_RIGHT 1
int motorSwitchDetected = 0;


int counter = 0;
float h; // humidity
float t; // temperature
float hic; // heat index


//*** Initialisation ********************************************

void init_lcd() {
  // SDA = 2 (GPIO4), SCL = 14 (GPIO5)
  lcd.init(2,14);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
}

void configureOTA() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
    ArduinoOTA.setHostname(IOT_DEVICE_ID);


  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

    ArduinoOTA.onStart([]() {
      Serial.println("Starting OTA update");
      publishDebug("Starting OTA update");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnding OTA update");
      publishDebug("Ending OTA update");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      publishDebug("OTA Error");
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.print("Test: ");
    Serial.println(WiFi.localIP());
}

void init_wifi() {
  Serial.println("Incubator Controller OTA v6");
  Serial.println("Initialising Wifi");
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");
  Serial.println(ssid);


  if (strcmp (WiFi.SSID().c_str(), ssid) != 0) {
//if (strcmp, (WiFi.SSID(),ssid.c_str()) != 0) {
    WiFi.begin(ssid, password);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }


  Serial.println("");
  Serial.print("WiFi connected with OTA4, IP address: ");
  Serial.println(WiFi.localIP());
  publishDebug("WiFi Connected with OTA on IP: "+WiFi.localIP());

  configureOTA();
}
// Initialise Timers

void initTimers() {
  os_timer_setfn(&environmentTimer, environmentTimerFinished, NULL);
  os_timer_arm(&environmentTimer,10000, true);

  os_timer_setfn(&trayTiltTimer, trayTiltTimerFinished, NULL);
  os_timer_arm(&trayTiltTimer,15000, true);
}

void initSwitches() {
  // Configure the ports for reading the motor switches
  publishDebug("Configuring switch sensors");
  lcdStatus("Init switch sensors");
  pinMode(PORT_MOTOR_SWITCH_LEFT, INPUT_PULLUP);
  pinMode(PORT_MOTOR_SWITCH_RIGHT, INPUT_PULLUP);
  pinMode(BUILTIN_LED, OUTPUT);
  attachInterrupt(PORT_MOTOR_SWITCH_LEFT, motorSwitchLeftActivated, FALLING);
  attachInterrupt(PORT_MOTOR_SWITCH_RIGHT, motorSwitchRightActivted, FALLING);
  delay(1000);
}

void lcdStatus(String message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
}
// Initialise motor

void initMotor() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Setting Motor L");
  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);

  analogWrite(PWMA,0);
  digitalWrite(DIRA,0);
  delay(5000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Starting Motor L");
  //publishDebug("Starting Motor");
  analogWrite(PWMA,1000);
  delay(1000);
  analogWrite(PWMA,0);
  delay(1000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Finished Motor");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Setting Motor R");

  analogWrite(PWMA,0);
  digitalWrite(DIRA,1);
  delay(5000);


  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Starting Motor R");

  analogWrite(PWMA,1000);
  delay(1000);
  analogWrite(PWMA,0);
  delay(1000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Finished Motor");
  delay(1000);

}
// Initialisation

void setup() {

  // Configure serial port

  Serial.begin(115200);
  Serial.println();

  init_lcd();
  lcd.print("Init v8");

  init_wifi();

  connectWithBroker();

  // Start the DHT22
  publishDebug("WiFi On");


  publishDebug("Initialising Sensors");
  dht.begin();
  Serial.println("sensor is starting..");
  Serial.print("Reading Analog...");
  Serial.println(analogRead(0));
  //publishDebug("Initialising Motor");
  initMotor();
  initSwitches();
  publishDebug("Initialising Timers");
  initTimers();
//----------------- wifi_set_sleep_type(LIGHT_SLEEP_T);
//----------------- gpio_pin_wakeup_enable(GPIO_ID_PIN(2),GPIO_PIN_INTR_HILEVEL);
}

// *************************************************************

void loop() {

  switch (state) {
    case MOTOR_SWITCH_DETECTED:
      digitalWrite(BUILTIN_LED, HIGH);
      motorSwitchActivated();
      break;
    default:
      break;
  }

  if (isEnvironmentTimerComplete == true) {
    isEnvironmentTimerComplete = false;

    Serial.println("Environment timer completed");
    publishDebug("Environment timer completed");
    ++counter;
    readDHTSensor();
    //connectWithBroker();
    debugDisplayPayload();
    displayLCD();
    publishPayload();
  }

  if (isTrayTiltTimerComplete == true) {
    // change the tilt of the tray

    isTrayTiltTimerComplete = false;

    Serial.println("Tray tilt timer completed");
    publishDebug("Tray tilt time completed");
    //lcd.setCursor(0,2);
    lcd.setCursor(0,1);
    lcd.print("Tray tilting");
    publishDebug("Tray tilting");
    delay(500);
    //lcd.setCursor(0,2);
    lcd.setCursor(0,1);
    lcdClearLine();
    publishDebug("Tray tilting completed");

  }
  ArduinoOTA.handle();
  delay(300);
  //delay(5000);
}

// Timer Call Backs

void environmentTimerFinished(void *pArg) {
  isEnvironmentTimerComplete = true;
}

void trayTiltTimerFinished(void *pArg) {
  isTrayTiltTimerComplete = true;
}
// ************************************************************************

void readDHTSensor() {
   // reading DHT22

  h = dht.readHumidity();
  t = dht.readTemperature();

  // Check if we fail to read from the DHT sensor
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor");
    publishDebug("Failed to read from DHT sensor");
    //TODO: Do more here
  }

  hic = dht.computeHeatIndex(t, h, false);
}

void connectWithBroker() {

  if (!!!client.connected()) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Connecting to broker");
    Serial.print("Reconnecting client to ");
    Serial.println(server);
    lcd.setCursor(0,1);
    int cursorPosition = 0;
    while (!!!client.connect(clientId, authMethod, token)) {
      Serial.print(".");
      lcd.setCursor(cursorPosition++,1);
      lcd.print(".");
      delay(500);
    }
    Serial.println();
  }
}

void publishPayload() {

  String payload = "{\"d\":{\"myName\":\"ESP8266.Test1\",\"counter\":";

  payload += counter;
  payload += ",\"volts\":";
  payload += volts;
  payload += ",\"temperature\":";
  payload += t;
  payload += ",\"humidity\":";
  payload += h;
  payload += ",\"heatIndex\":";
  payload += hic;
  payload += "}}";

  Serial.print("Sending payload: ");
  Serial.println(payload);

  if (client.publish(topic, (char*) payload.c_str())) {
    Serial.println("Publish ok");
  } else {
    Serial.print("Publish failed with error:");
    Serial.println(client.state());
  }
}

void publishDebug(String debugMessage) {
  String payload = "{\"d\":{\"myName\":\"ESP8266.Test1\",\"message\":";

  payload += "\""+debugMessage+"\"}";

  if (client.publish(debugTopic, (char *) payload.c_str())) {
    Serial.println("Published debug OK");
  } else {
    Serial.print("Publish failed with error:");
    Serial.println(client.state());
  }
}

void displayLCD() {

  static char tempStr[15];
  static char humidStr[15];
  static char hicStr[15];

  char firstLine[20];
  char secondLine[20];

  dtostrf(t,2,1,tempStr);
  dtostrf(h,2,1,humidStr);
  dtostrf(hic,2,1,hicStr);

  sprintf(firstLine,"Temp:%sC Hum:%s%",tempStr,humidStr);
  sprintf(secondLine,"HI:%sC C:%d",hicStr,counter);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(firstLine);
  lcd.setCursor(0,1);
  lcd.print(secondLine);
  publishDebug("Test message2");
}

void debugDisplayPayload() {
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.println(" *C ");
}

void callback(char* topic, byte* payload, unsigned int length) {
 Serial.println("callback invoked");
}

void lcdClearLine() {
  lcd.print("                ");

}

// Sensors

void motorSwitchLeftActivated() {
  state = MOTOR_SWITCH_DETECTED;
  motorSwitchDetected = MOTOR_SWITCH_LEFT;
}

void motorSwitchRightActivted() {
  state = MOTOR_SWITCH_DETECTED;
  motorSwitchDetected = MOTOR_SWITCH_RIGHT;
}

void motorSwitchActivated() {
  state = PUBLISHING_ENVIRONMENT_EVENT;
  if (motorSwitchDetected == MOTOR_SWITCH_LEFT) {
    lcdStatus("Switch Left");
    publishDebug("Left Motor Switch Detected");
  } else {
    lcdStatus("Switch Right");
    publishDebug("Right Motor Switch Detected");
  }
  delay(1000);
  digitalWrite(BUILTIN_LED, LOW);
  state = IDLE;
}
