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
  TRAY_TILTING,
  TILTING_TRAY_LEFT,
  TILTING_TRAY_RIGHT,
  OTA_IN_PROGRESS,
  MOTOR_SWITCH_DETECTED,
  TIMER_SWITCH_ACTIVATED
};

enum MotorPosition {
  LEFT,
  RIGHT,
  MIDDLE
};

SystemState state = IDLE;

MotorPosition currentMotorPosition = MIDDLE;

// WiFI credentials

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// IBM Internet of Things MQTT configuration

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

// Timer Trigger Switches

#define PORT_TIMER_SWITCH 15 // D8 = GPIO15

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

void initMotorTimer() {
  // This timer is for tilting the tray (by activating the motor)
  os_timer_setfn(&trayTiltTimer, trayTiltTimerFinished, NULL);
}

void initEnvironmentTimer() {
  // This timer is for taking temperature and humidity readings
  os_timer_setfn(&environmentTimer, environmentTimerFinished, NULL);
}

void startMotorTimer() {
  os_timer_arm(&trayTiltTimer,30000, true);
}

void stopMotorTimer() {
  os_timer_disarm(&trayTiltTimer);
}

void startEnvironmentTimer() {
  os_timer_arm(&environmentTimer,10000, true);
}

void stopEnvironmentTimer() {
  os_timer_disarm(&environmentTimer);
}

void initSwitches() {
  // Configure the ports for reading the motor switches
  publishDebug("Configuring switch sensors");
  lcdStatus("Init switch sensors");
  pinMode(PORT_MOTOR_SWITCH_LEFT, INPUT_PULLUP);
  pinMode(PORT_MOTOR_SWITCH_RIGHT, INPUT_PULLUP);
  pinMode(BUILTIN_LED, OUTPUT);
  delay(500);
}

void lcdStatus(String message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
}

void tiltMotor(MotorPosition targetPosition) {
  // set the direction of moving the motor
  if (targetPosition == LEFT) {
    digitalWrite(DIRA,1);
    publishDebug("Tilting motor LEFT");
    lcdStatus("Tilting LEFT");
  } else {
    digitalWrite(DIRA,0);
    publishDebug("Tilting motor RIGHT");
    lcdStatus("Tilting RIGHT");
  }

  // start the motor moving
  analogWrite(PWMA,1000);

  int motorSwitchLeft = digitalRead(PORT_MOTOR_SWITCH_LEFT);
  int motorSwitchRight = digitalRead(PORT_MOTOR_SWITCH_RIGHT);

  // Continue looping until a switch is Activated
  // TODO: Add add a safety timer so if we don't detect a switch
  // within a certain time we switch off the motor
  // in case a switch is faulty

  if (targetPosition == LEFT) {
    // only check for the left switch as the right switch would already
    // be active
    while (motorSwitchLeft != HIGH) {
      motorSwitchLeft = digitalRead(PORT_MOTOR_SWITCH_LEFT);
      delay(100);
    }
  } else {
    while (motorSwitchRight != HIGH) {
      motorSwitchRight = digitalRead(PORT_MOTOR_SWITCH_RIGHT);
      delay(100);
    }
  }

  switchOffMotor();
  if (targetPosition == LEFT) {
    currentMotorPosition = LEFT;
    publishDebug("Tilt completed to Left");
  } else {
    currentMotorPosition = RIGHT;
    publishDebug("Tilt completed to right");
  }
  lcdStatus("Finished tilting");

}


// Estbalish a known position of the motor
// If the neither switch is active set it to the default side
// If a switch is active, leave it there

void calibrateMotor() {

  publishDebug("Calibrating motor");
  lcdStatus("Calibrating motor");

  pinMode(DIRA, OUTPUT);
  pinMode(PWMA, OUTPUT);

  // read Motor Switches
  int motorSwitchLeft = digitalRead(PORT_MOTOR_SWITCH_LEFT);
  int motorSwitchRight = digitalRead(PORT_MOTOR_SWITCH_RIGHT);

  // TODO: Capture the state of the motor
  // so we know when to just stop and wait or
  // we're just taking off

  if (motorSwitchLeft == HIGH) {
    // Make sure motor is off
    switchOffMotor();
    publishDebug("Motor Switch Left is HIGH");
    lcdStatus("Motor SW Left High");
    currentMotorPosition = LEFT;
    delay(100);
    return;
  }
  if (motorSwitchRight == HIGH) {
    switchOffMotor();
    publishDebug("Motor Switch Right is HIGH");
    lcdStatus("Motor SW Right High");
    currentMotorPosition = RIGHT;
    delay(100);
    return;
  }

  // Neither switch is on
  switchOffMotor();
  publishDebug("Motor is in middle");
  lcdStatus("Motor in middle");
  currentMotorPosition = MIDDLE;

  // Track motor to default position
  tiltMotor(LEFT);
}

// Initialisation

void setup() {

  // Configure serial port

  Serial.begin(115200);
  Serial.println();

  init_lcd();
  lcdStatus("Initialisaing");
  init_wifi();
  connectWithBroker();

  // Start the DHT22
  publishDebug("WiFi On");
  publishDebug("Initialising Sensors");
  dht.begin();
  Serial.println("sensor is starting..");
  Serial.print("Reading Analog...");
  Serial.println(analogRead(0));

  initSwitches();
  calibrateMotor();

  // Start the timers
  initMotorTimer();
  startMotorTimer();
  initEnvironmentTimer();
  startEnvironmentTimer();

  publishDebug("Setup complete");
  lcdStatus("Setup complete");
//----------------- wifi_set_sleep_type(LIGHT_SLEEP_T);
//----------------- gpio_pin_wakeup_enable(GPIO_ID_PIN(2),GPIO_PIN_INTR_HILEVEL);
}

// *************************************************************

void loop() {
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
    stopMotorTimer();
    state = TRAY_TILTING;
    if (currentMotorPosition == LEFT) {
      tiltMotor(RIGHT);
    } else {
      tiltMotor(LEFT);
    }
    startMotorTimer();
    state = IDLE;
  }
  ArduinoOTA.handle();
}

// Timer Call Backs

void environmentTimerFinished(void *pArg) {
  // don't do too much in here. Just set the state
  // and handle it in the main loop
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
// Motor control

void switchOffMotor() {
  analogWrite(PWMA,0);
  publishDebug("Switching off motor");
  lcdStatus("Motor off");
}

// Sensors

void motorSwitchLeftActivated() {
  state = MOTOR_SWITCH_DETECTED;
  motorSwitchDetected = MOTOR_SWITCH_LEFT;
}

void motorSwitchRightActivated() {
  state = MOTOR_SWITCH_DETECTED;
  motorSwitchDetected = MOTOR_SWITCH_RIGHT;
}

void motorSwitchAction() {
  state = PUBLISHING_ENVIRONMENT_EVENT;
  switchOffMotor();
  if (motorSwitchDetected == MOTOR_SWITCH_LEFT) {
    lcdStatus("Switch Left");
    publishDebug("Left Motor Switch Detected");
  } else {
    lcdStatus("Switch Right");
    publishDebug("Right Motor Switch Detected");
  }
  digitalWrite(BUILTIN_LED, LOW);
  delay(100);
  state = IDLE;
}

// External switch trigger - TBD

// void timerSwitchActivated() {
//   state = TIMER_SWITCH_ACTIVATED;
// }

// void timerSwitchAction() {
//   state = IDLE;
//   publishDebug("Timer Switch Activated");
//   lcdStatus("Timer Switch On");
//   digitalWrite(BUILTIN_LED, LOW);
//   initTimers();
// }
