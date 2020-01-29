#ifndef config_h
#define config_h

#include <IPAddress.h>

#define SERIAL_BAUD 115200                      // The BAUD rate (speed) of the serial port (console).
#define ENABLE_OTA                              // Comment this line to disable OTA updates.
#define ENABLE_MDNS                             // Comment this line to disable the MDNS.
#define CONFIG_FILE_PATH "/config.json"         // The config file path. Do not alter unless you are sure.
#define DEFAULT_CLOCK_TIMEZONE -4               // The timezone this device is located in. (For example, EST when observing DST = GMT-4, when not = GMT-5)
#define DEFAULT_SSID "CYSAFETY"                 // Put the SSID of your WiFi here.
#define DEFAULT_PASSWORD "ThereIsNoSpoon"       // Put your WiFi password here.
#define DEVICE_NAME "CYSAFETY1"                 // The device name.
#define CHECK_WIFI_INTERVAL 30000               // How often to check WiFi status (milliseconds).
#define CLOCK_SYNC_INTERVAL 3600000             // How often to sync the local clock with NTP (milliseconds).
#define CHECK_SENSORS_INTERVAL 1000             // How often to check sensors (milliseconds).
#define CHECK_MQTT_INTERVAL 35000               // MQTT connectivity check interval (milliseconds).
#define MQTT_TOPIC_STATUS "cysafety/status"     // MQTT status channel to publish to.
#define MQTT_TOPIC_CONTROL "cysafety/control"   // MQTT control channel to subscribe to.
#define MQTT_BROKER "your_mqtt_broker_IP"       // MQTT broker hostname or IP.
#define MQTT_PORT 8883                          // MQTT port number.
#define WEBSERVER_PORT 80                       // The internal webserver port (only used for configuration)
#ifdef ENABLE_OTA
    #include <ArduinoOTA.h>
    #define OTA_HOST_PORT 8266                     // The OTA updater port.
    #define OTA_PASSWORD "your_OTA_password_here"  // The OTA updater password.
#endif
IPAddress ip(192, 168, 0, 248);
IPAddress gw(192, 168, 0, 1);
IPAddress sm(255, 255, 255, 0);
IPAddress dns(gw);

#endif