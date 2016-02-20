/**
 * Incubator Controller V2
 *
 * Author: Tony Kambourakis
 * License: Apache License v2
 */

extern "C" {
  #include "user_interface.h"
//  #include "Esp.h"
 // #include "gpio.h"
}
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient/releases/tag/v2.3
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include "SensitiveConfig.h"
#include <Time.h>

// Configure 2 line LCD display with I2C interface

LiquidCrystal_I2C lcd(0x27,16,2);
//LiquidCrystal_I2C lcd(0x27,20,4);

os_timer_t myTimer;

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

int counter = 0;
float h; // humidity
float t; // temperature
float hic; // heat index

//*** Initialisation ********************************************

void init_lcd() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
}

void init_wifi() {
  Serial.print("Connecting to ");
  Serial.print(ssid);

  if (strcmp (WiFi.SSID().c_str(), ssid) != 0) {
//if (strcmp, (WiFi.SSID(),ssid.c_str()) != 0) {
    WiFi.begin(ssid, password);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());
}

// Initialisation

void setup() {
  // Configure serial port

  Serial.begin(115200);
  Serial.println();

  init_lcd();
  lcd.print("Initialising");

  // Start the DHT22

  dht.begin();
  Serial.println("sensor is starting..");
  Serial.print("Reading Analog...");
  Serial.println(analogRead(0));

  init_wifi();
//----------------- wifi_set_sleep_type(LIGHT_SLEEP_T);
//----------------- gpio_pin_wakeup_enable(GPIO_ID_PIN(2),GPIO_PIN_INTR_HILEVEL);
}

// *************************************************************

void loop() {
  readDHTSensor();
  connectWithBroker();
  debugDisplayPayload();
  displayLCD();
  publishPayload();

  ++counter;
  delay(15000);
}

// ************************************************************************

void readDHTSensor() {
   // reading DHT22

  h = dht.readHumidity();
  t = dht.readTemperature();

  // Check if we fail to read from the DHT sensor
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor");
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
