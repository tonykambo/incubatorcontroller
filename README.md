# Incubator Controller
The incubator controller will be used to monitor and control a custom built egg incubator. The tilt of the eggs must be periodically changed. The temperature and humidity must be maintained within as specific range depending on the breed. A actuator will be controlled to alter the tilt angle of the egg tray inside the incubator. A sensor that measures temperature and humidity will be placed inside the incubator to take regular measurements. If the temperature or humidity falls outside a nominated range a push notification will be sent to an iOS app. The iOS app will also support querying the current temperature and humidity readings. The iOS app may also be used to alter the period that the tray tilt is changed. The temperature and humidity measurements will be recorded for historical analysis. An LCD will be mounted locally to allow observation of temperature and humidity including the progress within tilting cycle.

Attach a device to an existing letterbox that will send a push notification when a letter or package is inserted. It will also report the temperature, humidity and heat index. This code will control an actuator that tilts an egg tray in an incubator. The tray's tilt is changed every 4 hours however the period may be remotely configured via an iOS app. 

The controller will also measure the temperature and humidity within the incubator using a DHT22 and display them on an LCD. These values will also be published to IBM Internet of Things Foundation, recorded in DashDB and displayed using IBM IoT Real-Time Insights. 

The configuration details for the WiFi SSID/password and the IBM IoT Foundation broker are defined in SensitiveConfig.h. This file should be placed in the ~/Arduino/libraries/SensistiveConf folder. You may use the included template (SensitiveConfig-Template.h) as a starting point. If you check your project into a public source repository ensure you add the SensitiveConfig.h file to .gitignore. 

This code was written to run on the NodeMCU (ESP8266) using the Arduino IDE.

