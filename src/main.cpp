#include <Arduino.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_ADS1015.h"
#include "Adafruit_BME280.h"
#include "ArduinoJson.h"
#include "ESPCrashMonitor-master/ESPCrashMonitor.h"
#include "LED.h"
#include "PubSubClient.h"
#include "ResetManager.h"
#include "TaskScheduler.h"
#include "TelemetryHelper.h"
#include "config.h"

#define FIRMWARE_VERSION "1.0"

// Workaround to allow an MQTT packet size greater than the default of 128.
#ifdef MQTT_MAX_PACKET_SIZE
#undef MQTT_MAX_PACKET_SIZE
#endif
#define MQTT_MAX_PACKET_SIZE 200

#define PIN_HEARTBEAT_LED 5
#define PIN_MQ2_SENSOR A0
#define PIN_ADC_LDR 0
#define BME280_ADDRESS 0x76
#define ADS1115_ADDRESS 0x48

// Forward declarations
void onCheckWiFi();
void onCheckSensors();
void onCheckMqtt();
void onMqttMessage(char* topic, byte* payload, unsigned int length);
void onSyncClock();

#ifdef ENABLE_MDNS
    #include <ESP8266mDNS.h>
    MDNSResponder mdns;
#endif
Adafruit_ADS1115 adc(ADS1115_ADDRESS);
Adafruit_BME280 bme;
WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
LED heartbeatLED(PIN_HEARTBEAT_LED, NULL);
int webServerPort = WEBSERVER_PORT;
ESP8266WebServer server(webServerPort);
Task tCheckWifi(CHECK_WIFI_INTERVAL, TASK_FOREVER, &onCheckWiFi);
Task tCheckSensors(CHECK_SENSORS_INTERVAL, TASK_FOREVER, &onCheckSensors);
Task tCheckMqtt(CHECK_MQTT_INTERVAL, TASK_FOREVER, &onCheckMqtt);
Task tSyncClock(CLOCK_SYNC_INTERVAL, TASK_FOREVER, &onSyncClock);
Scheduler taskMan;
String hostName = DEVICE_NAME;
String ssid = DEFAULT_SSID;
String password = DEFAULT_PASSWORD;
String mqttBroker = MQTT_BROKER;
String controlChannel = MQTT_TOPIC_CONTROL;
String statusChannel = MQTT_TOPIC_STATUS;
String serverFingerprintPath;
String caCertificatePath;
String fingerprintString;
String lastState = "";
String mqttUsername = "";
String mqttPassword = "";
int mqttPort = MQTT_PORT;
int gasValue = 0;
int ldrValue = 0;
int timeZone = DEFAULT_CLOCK_TIMEZONE;
float humidityValue = 0;
float tempValue = 0;
float pressureValue = 0;
volatile SystemState sysState = SystemState::BOOTING;
bool isDHCP = false;
bool filesystemMounted = false;
bool connSecured = false;
bool needsConfig = true;
#ifdef ENABLE_OTA
    int otaPort = OTA_HOST_PORT;
    String otaPassword = OTA_PASSWORD;
#endif

/**
 * Waits for user input from the serial console.
 */
void waitForUserInput() {
    while (Serial.available() < 1) {
        ESPCrashMonitor.iAmAlive();
        delay(50);
    }
}

/**
 * Gets an IPAddress value from the specified string.
 * @param value The string containing the IP.
 * @return The IP address.
 */
IPAddress getIPFromString(String value) {
    unsigned int ip[4];
    unsigned char buf[value.length()];
    value.getBytes(buf, value.length());
    const char* ipBuf = (const char*)buf;
    sscanf(ipBuf, "%u.%u.%u.%u", &ip[0], &ip[1], &ip[2], &ip[3]);
    return IPAddress(ip[0], ip[1], ip[2], ip[3]);
}

/**
 * Scan for available networks and dump each discovered network to the console.
 */
void getAvailableNetworks() {
    ESPCrashMonitor.defer();
    Serial.println(F("INFO: Scanning WiFi networks..."));
    int numNetworks = WiFi.scanNetworks();
    for (int i = 0; i < numNetworks; i++) {
        Serial.print(F("ID: "));
        Serial.print(i);
        Serial.print(F("\tNetwork name: "));
        Serial.print(WiFi.SSID(i));
        Serial.print(F("\tSignal strength:"));
        Serial.println(WiFi.RSSI(i));
    }
    Serial.println(F("----------------------------------"));
}

/**
 * Prints network information details to the serial console.
 */
void printNetworkInfo() {
    Serial.print(F("INFO: Local IP: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("INFO: Gateway: "));
    Serial.println(WiFi.gatewayIP());
    Serial.print(F("INFO: Subnet mask: "));
    Serial.println(WiFi.subnetMask());
    Serial.print(F("INFO: DNS server: "));
    Serial.println(WiFi.dnsIP());
    Serial.print(F("INFO: MAC address: "));
    Serial.println(WiFi.macAddress());
    WiFi.printDiag(Serial);
}

/**
 * Gets string input from the serial console.
 * @param isPassword If true, echos back a '*' instead of the character
 * that was entered.
 * @return The string that was entered.
 */
String getInputString(bool isPassword = false) {
    char c;
    String result = "";
    bool gotEndMarker = false;
    while (!gotEndMarker) {
        ESPCrashMonitor.iAmAlive();
        if (Serial.available() > 0) {
            c = Serial.read();
            if (c == '\n') {
                gotEndMarker = true;
                break;
            }

            if (isPassword) {
                Serial.print('*');
            }
            else {
                Serial.print(c);
            }
            result += c;
        }
    }

    return result;
}

/**
 * Reboots the MCU after a 1 second delay.
 */
void reboot() {
    Serial.println(F("INFO: Rebooting... "));
    Serial.flush();
    delay(1000);
    mdns.end();
    server.stop();
    WiFi.disconnect(true);
    ResetManager.softReset();
}

/**
 * Synchronize the local system clock via NTP. Note: This does not take DST
 * into account. Currently, you will have to adjust the CLOCK_TIMEZONE define
 * manually to account for DST when needed.
 */
void onSyncClock() {
    configTime(timeZone * 3600, 0, "pool.ntp.org", "time.nist.gov");

    Serial.print("INIT: Waiting for NTP time sync...");
    delay(500);
    while (!time(nullptr)) {
        ESPCrashMonitor.iAmAlive();
        Serial.print(F("."));
        delay(500);
    }

    time_t now = time(nullptr);
    struct tm *timeinfo = localtime(&now);
    
    Serial.println(F(" DONE"));
    Serial.print(F("INFO: Current time: "));
    Serial.println(asctime(timeinfo));
}


/**
 * Stores the in-memory configuration to a JSON file stored in SPIFFS.
 * If the file does not yet exist, it will be created (see CONFIG_FILE_PATH).
 * Errors will be reported to the serial console if the filesystem is not
 * mounted or if the file could not be opened for writing.
 */
void saveConfiguration() {
    Serial.print(F("INFO: Saving configuration to: "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.println(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }

    StaticJsonDocument<350> doc;
    doc["needsConfigured"] = needsConfig;
    doc["hostname"] = hostName;
    doc["useDHCP"] = isDHCP;
    doc["ssid"] = ssid;
    doc["password"] = password;
    doc["mqttBroker"] = mqttBroker;
    doc["mqttPort"] = mqttPort;
    doc["mqttStatusChannel"] = statusChannel;
    doc["mqttControlChannel"] = controlChannel;
    doc["configurationPort"] = webServerPort;
    #ifdef ENABLE_OTA
        doc["otaPort"] = otaPort;
        doc["otaPassword"] = otaPassword;
    #endif
    doc["ip"] = ip.toString();
    doc["subnetMask"] = sm.toString();
    doc["gateway"] = gw.toString();
    doc["dns"] = dns.toString();
    doc["timeZone"] = timeZone;

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "w");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to open config file for writing."));
        doc.clear();
        return;
    }

    serializeJsonPretty(doc, configFile);
    doc.clear();
    configFile.flush();
    configFile.close();
    Serial.println(F("DONE"));
}

/**
 * Loads the configuration from CONFIG_FILE_PATH into memory and uses that as
 * the running configuration. Will report errors to the serial console and
 * revert to the default configuration under the following conditions:
 * 1) The filesystem is not mounted.
 * 2) The config file does not exist in SPIFFS. In this case a new file
 * will be created and populated with the default configuration.
 * 3) The config file exists, but could not be opened for reading.
 * 4) The config file is too big ( > 1MB).
 * 5) The config file could not be deserialized to a JSON structure.
 */
void loadConfiguration() {
    Serial.print(F("INFO: Loading config file: "));
    Serial.print(CONFIG_FILE_PATH);
    Serial.print(F(" ... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return;
    }

    if (!SPIFFS.exists(CONFIG_FILE_PATH)) {
        Serial.println(F("FAIL"));
        Serial.println(F("WARN: Config file does not exist. Creating with default config..."));
        saveConfiguration();
        return;
    }

    File configFile = SPIFFS.open(CONFIG_FILE_PATH, "r");
    if (!configFile) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to open config file. Using default config."));
        return;
    }

    size_t size = configFile.size();
    if (size > 1024) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Config file size is too large. Using default config."));
        configFile.close();
        return;
    }

    std::unique_ptr<char[]> buf(new char[size]);
    configFile.readBytes(buf.get(), size);
    configFile.close();

    StaticJsonDocument<350> doc;
    DeserializationError error = deserializeJson(doc, buf.get());
    if (error) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to parse config file to JSON. Using default config."));
        return;
    }

    needsConfig = doc["needsConfigured"].as<bool>();
    hostName = doc["hostname"].as<String>();
    isDHCP = doc["useDHCP"].as<bool>();
    ssid = doc["wifiSSID"].as<String>();
    password = doc["wifiPassword"].as<String>();
    mqttBroker = doc["mqttBroker"].as<String>();
    mqttPort = doc["mqttPort"].as<int>();
    statusChannel = doc["mqttStatusChannel"].as<String>();
    controlChannel = doc["mqttControlChannel"].as<String>();
    webServerPort = doc["configurationPort"].as<int>();
    #ifdef ENABLE_OTA
        otaPort = doc["otaPort"].as<int>();
        otaPassword = doc["otaPassword"].as<String>();
    #endif

    if (!ip.fromString(doc["ip"].as<String>())) {
        Serial.println(F("WARN: Invalid IP in configuration. Falling back to factory default."));
    }
    
    if (!sm.fromString(doc["subnetMask"].as<String>())) {
        Serial.println(F("WARN: Invalid subnet mask in configuration. Falling back to default."));
    }

    if (!gw.fromString(doc["gateway"].as<String>())) {
        Serial.println(F("WARN: Invalid gateway in configuration. Falling back to factory default."));
    }
    
    if (!dns.fromString(doc["dnsServer"].as<String>())) {
        Serial.println(F("WARN: Invalid DSN server in configuration. Falling back to default."));
    }

    timeZone = doc["timeZone"].as<int>();
    
    doc.clear();
    Serial.println(F("DONE"));
}

/**
 * Loads the SSL certificates and server fingerprint necessary to establish
 * a connection the the MQTT broker over TLS.
 * @return true if the certificates and server fingerprint were successfully
 * loaded; Otherwise, false.
 */
bool loadCertificates() {
    Serial.print(F("INFO: Loading SSL certificates... "));
    if (!filesystemMounted) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Filesystem not mounted."));
        return false;
    }

    if (!SPIFFS.exists(caCertificatePath)) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: CA certificate does not exist."));
        return false;
    }

    File ca = SPIFFS.open(caCertificatePath, "r");
    if (!ca) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Could not open CA certificate."));
        return false;
    }

    String caContents = ca.readString();
    ca.close();
    X509List caCertX509(caContents.c_str());

    wifiClient.setTrustAnchors(&caCertX509);
    wifiClient.allowSelfSignedCerts();

    if (!SPIFFS.exists(serverFingerprintPath)) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Server fingerprint file path does not exist."));
        return false;
    }

    File fp = SPIFFS.open(serverFingerprintPath, "r");
    if (!fp) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Could not open fingerprint file."));
        return false;
    }

    String val;
    if (fp.available()) {
        String fileContent = fp.readString();
        val = fileContent.substring(fileContent.lastIndexOf("=") + 1);
        val.replace(':', ' ');
    }

    fp.close();
    if (val.length() > 0) {
        fingerprintString = val;
        wifiClient.setFingerprint(fingerprintString.c_str());
    }
    else {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Failed to read server fingerprint."));
        return false;
    }
    
    Serial.println(F("DONE"));
    return true;
}

/**
 * Verifies a connection can be made to the MQTT broker over TLS.
 * @return true if a connection to the MQTT broker over TLS was established
 * successfully; Otherwise, false.
 */
bool verifyTLS() {
    // Because it can take longer than expected to establish an
    // encrypted connection the MQTT broker, we need to disable
    // the watchdog to prevent reboot due to watchdog timeout during
    // connection, then re-enable when we are done.
    ESPCrashMonitor.disableWatchdog();

    // Currently, we sync the clock any time we need to verify TLS. This is
    // because in a future version, this will be required in order to validate
    // public CA certificates.
    onSyncClock();

    Serial.print(F("INFO: Verifying connectivity over TLS... "));
    bool result = wifiClient.connect(mqttBroker, mqttPort);
    if (result) {
        wifiClient.stop();
        Serial.println(F("DONE"));
    }
    else {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: TLS connection failed."));
    }

    ESPCrashMonitor.enableWatchdog(ESPCrashMonitorClass::ETimeout::Timeout_2s);
    return result;
}

/**
 * Confirms with the user that they wish to do a factory restore. If so, then
 * clears the current configuration file in SPIFFS, then reboots. Upon reboot,
 * a new config file will be generated with default values.
 * @param fromSubmit Set true if request came from the configuration page.
 */
void doFactoryRestore(bool fromSubmit) {
    String str = "N";
    if (!fromSubmit) {
        Serial.println();
        Serial.println(F("Are you sure you wish to restore to factory default? (Y/n)?"));
        waitForUserInput();
        str = getInputString();
    }
    
    if (str == "Y" || str == "y" || fromSubmit) {
        Serial.print(F("INFO: Clearing current config... "));
        if (filesystemMounted) {
            if (SPIFFS.remove(CONFIG_FILE_PATH)) {
                Serial.println(F("DONE"));
                Serial.print(F("INFO: Removed file: "));
                Serial.println(CONFIG_FILE_PATH);

                Serial.print(F("INFO: Rebooting in "));
                for (uint8_t i = 5; i >= 1; i--) {
                    Serial.print(i);
                    Serial.print(F(" "));
                    delay(1000);
                }

                reboot();
            }
            else {
                Serial.println(F("FAIL"));
                Serial.println(F("ERROR: Failed to delete configuration file."));
            }
        }
        else {
            Serial.println(F("FAIL"));
            Serial.println(F("ERROR: Filesystem not mounted."));
        }
    }

    Serial.println();
}

/**
 * Publishes the system state to the MQTT status channel and
 * blinks the WiFi status LED.
 */
void publishSystemState() {
    if (mqttClient.connected()) {
        heartbeatLED.on();

        DynamicJsonDocument doc(200);
        doc["clientId"] = hostName;
        doc["firmwareVersion"] = FIRMWARE_VERSION;
        doc["systemState"] = (uint8_t)sysState;
        doc["humidity"] = humidityValue;
        doc["temperature"] = tempValue;
        doc["pressure"] = pressureValue;
        doc["gasValue"] = gasValue;
        doc["ambientLightLevel"] = ldrValue;

        String jsonStr;
        size_t len = serializeJson(doc, jsonStr);
        Serial.print(F("INFO: Publishing system state: "));
        Serial.println(jsonStr);
        if (!mqttClient.publish(statusChannel.c_str(), jsonStr.c_str(), len)) {
            Serial.println(F("ERROR: Failed to publish message."));
        }

        doc.clear();
        heartbeatLED.off();
    }
}

/**
 * Resume normal operation. This will resume any suspended tasks.
 */
void resumeNormal() {
    Serial.println(F("INFO: Resuming normal operation..."));
    taskMan.enableAll();
    heartbeatLED.off();
    sysState = SystemState::NORMAL;
    publishSystemState();
}

/**
 * Attempts to re-connect to the MQTT broker if then connection is
 * currently broken.
 * @return true if a connection to the MQTT broker is either already
 * established, or was successfully re-established; Otherwise, false.
 * If the connection is re-established, then will also re-subscribe to
 * the status channel.
 */
bool reconnectMqttClient() {
    if (!mqttClient.connected()) {
        Serial.print(F("INFO: Attempting to establish MQTT connection to "));
        Serial.print(mqttBroker);
        Serial.print(F(" on port: "));
        Serial.print(mqttPort);
        Serial.println(F("..."));
        if (!connSecured) {
            connSecured = verifyTLS();
            if (!connSecured) {
                Serial.println(F("ERROR: Unable to establish TLS connection to host."));
                Serial.println(F("ERROR: Invalid certificate or SSL negotiation failed."));
                return false;
            }
        }

        bool didConnect = false;
        if (mqttUsername.length() > 0 && mqttPassword.length() > 0) {
            didConnect = mqttClient.connect(hostName.c_str(), mqttUsername.c_str(), mqttPassword.c_str());
        }
        else {
            didConnect = mqttClient.connect(hostName.c_str());
        }

        if (didConnect) {
            Serial.print(F("INFO: Subscribing to channel: "));
            Serial.println(controlChannel);
            mqttClient.subscribe(controlChannel.c_str());

            Serial.print(F("INFO: Publishing to channel: "));
            Serial.println(statusChannel);
        }
        else {
            String failReason = TelemetryHelper::getMqttStateDesc(mqttClient.state());
            Serial.print(F("ERROR: Failed to connect to MQTT broker: "));
            Serial.println(failReason);
            return false;
        }
    }

    return true;
}

/**
 * Callback method for checking the MQTT connection state and
 * reconnecting if necessary. If the connection is broken, and
 * reconnection fails, another attempt will be made after CHECK_MQTT_INTERVAL.
 */
void onCheckMqtt() {
    Serial.println(F("INFO: Checking MQTT connection status..."));
    if (reconnectMqttClient()) {
        Serial.println(F("INFO: Successfully reconnected to MQTT broker."));
        publishSystemState();
    }
    else {
        Serial.println(F("ERROR: MQTT connection lost and reconnect failed."));
        Serial.print(F("INFO: Retrying connection in "));
        Serial.print(CHECK_MQTT_INTERVAL % 1000);
        Serial.println(F(" seconds."));
    }
}

/**
 * 
 */
void onCheckSensors() {
    // First get gas/smoke value.
    gasValue = analogRead(PIN_MQ2_SENSOR);

    // Get LDR value.
    ldrValue = adc.readADC_SingleEnded(PIN_ADC_LDR);

    // Temperature
    tempValue = bme.readTemperature();

    // Humidity
    humidityValue = bme.readHumidity();

    // Barometric pressure
    pressureValue = bme.readPressure();

    // TODO How do we get altitude? we could call bme.readAltitude()
    // but it requires a seaLevel param in hPa (not elevation). So
    // how do we get sea-level hPa without sampling it?
}

bool handleFileRead(String path) {
    if (path.endsWith("/")) {
        path += "index.html";
    }

    Serial.print(F("INFO: WebServer request for path:"));
    Serial.println(path);
    if (SPIFFS.exists(path)) {
        File file = SPIFFS.open(path, "r");
        size_t sent = server.streamFile(file, "text/html");
        file.close();
        Serial.print(F("INFO: Sent file "));
        Serial.print(path);
        Serial.print(F(" ("));
        Serial.print(sent);
        Serial.println(F(" bytes) OK."));
        return true;
    }

    Serial.print(F("ERROR: File not found: "));
    Serial.println(path);
    return false;
}

void handleRoot() {
    if (!handleFileRead(server.uri())) {
        server.send(404, "text/plain", "404: Not Found");
    }
}

void handleSubmit() {
    bool result = true;
    int argCount = server.args();
    if (argCount > 0) {
        for (uint8_t i = 0; i < argCount; i++) {
            String argName = server.argName(i);
            String argVal = server.arg(i);
            argVal.trim();
            if (argName.equals("SSID")) {
                ssid = argVal;
            }
            else if (argName.equals("PASSWORD")) {
                password = argVal;
            }
            else if (argName.equals("MQTT_BROKER")) {
                mqttBroker = argVal;
            }
            else if (argName.equals("MQTT_PORT")) {
                mqttPort = argVal.toInt();
            }
            else if (argName.equals("MQTT_CTRL")) {
                controlChannel = argVal;
            }
            else if (argName.equals("MQTT_STAT")) {
                statusChannel = argVal;
            }
            else if (argName.equals("OTA_PORT")) {
                otaPort = argVal.toInt();
            }
            else if (argName.equals("OTA_PASSWORD")) {
                otaPassword = argVal;
            }
            else if (argName.equals("TIME_ZONE")) {
                timeZone = argVal.toInt();
            }
            else if (argName.equals("DHCP")) {
                isDHCP = argVal.toInt() == 1;
            }
            else if (argName.equals("IP_ADDR")) {
                if (!ip.fromString(argVal)) {
                    Serial.println(F("ERROR: Parsing IP failed from /submit."));
                }
            }
            else if (argName.equals("SM_ADDR")) {
                if (!sm.fromString(argVal)) {
                    Serial.println(F("ERROR: Parsing subnet mask failed from /submit."));
                }
            }
            else if (argName.equals("GW_ADDR")) {
                if (!gw.fromString(argVal)) {
                    Serial.println(F("ERROR: Parsing gateway IP failed from /submit."));
                }
            }
            else if (argName.equals("DNS_ADDR")) {
                if (!dns.fromString(argVal)) {
                    Serial.println(F("ERROR: Parsing DNS IP failed from /submit."));
                }
            }
            else {
                // TODO bogus argument.
            }
        }

        
    }

    // TODO Validate connection config stuff. If we can't connect to WiFi,
    // then mark it as a failure and fall back to default values. Then return root.
    // If all is well though, save config changes and reboot.
    if (!result) {
        handleFileRead("/saveError.html");
        return;
    }

    handleFileRead("/saveSuccess.html");
}

void handleRestoreNav() {
    String restorePage = server.uri();
    if (restorePage.indexOf("restore") > 0 && !restorePage.endsWith(".html")) {
        restorePage += ".html";
    }

    if (!handleFileRead(restorePage)) {
        server.send(404, "text/plain", "404: Not Found");
    }
}

void handleFactoryRestore() {
    Serial.println(F("INFO: Recevied factory restore request from web client."));
    doFactoryRestore(true);
    reboot();
}

/**
 * Initializes the RS232 serial interface.
 */
void initSerial() {
    Serial.setDebugOutput(true);
    Serial.begin(SERIAL_BAUD, SERIAL_8N1);
    Serial.println();
    Serial.println();
    Serial.print(F("INIT: CyGarage v"));
    Serial.print(FIRMWARE_VERSION);
    Serial.println(F(" booting ..."));
    Serial.println();
}

/**
 * Initializes output components.
 */
void initOutputs() {
    Serial.print(F("INIT: Initializing components... "));
    heartbeatLED.init();
    heartbeatLED.on();
    delay(1000);
    Serial.println(F("DONE"));
}

/**
 * Initializes the required input pins.
 */
void initInputs() {
    Serial.print(F("INIT: Initializing sensors... "));
    adc.begin();
    bme.begin(BME280_ADDRESS);
    digitalWrite(PIN_MQ2_SENSOR, LOW);
    pinMode(PIN_MQ2_SENSOR, INPUT);
    Serial.println(F("DONE"));
}

/**
 * Initialize the SPIFFS filesystem.
 */
void initFilesystem() {
    Serial.print(F("INIT: Initializing SPIFFS and mounting filesystem... "));
    if (!SPIFFS.begin()) {
        Serial.println(F("FAIL"));
        Serial.println(F("ERROR: Unable to mount filesystem."));
        return;
    }

    filesystemMounted = true;
    Serial.println(F("DONE"));
    loadConfiguration();
}

void initWiFi() {
    Serial.println(F("INIT: Initializing WiFi... "));
    if (needsConfig) {
        WiFi.mode(WIFI_STA);
        WiFi.softAP(DEFAULT_SSID, DEFAULT_PASSWORD);

    }
    else {
        getAvailableNetworks();
    
        Serial.print(F("INFO: Connecting to SSID: "));
        Serial.print(ssid);
        Serial.println(F("..."));
    
        connectWifi();
    }
}

void setup() {
    initSerial();
    initOutputs();
    initInputs();
    initFilesystem();
}

void loop() {
    
}