/*
  eSense Rubino
  Environmental Sensing Device based on Espressif ESP8266, Bosch BME680
  and ROHM BH1750.

  Copyright (C) 2020  MOVASIM (https://movasim.com/)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  ================= 
  = Sensor BME680 =
  =================
  The BME680 is an environmental digital sensor that measures VOC gases, pressure, 
  humidity and temperature.
  connections:
  - VCC -> 3V3
  - GND -> GND
  - SCL -> SCL
  - SDA -> SDA
  - SDO -> N/C
  - CS -> N/C

  =================
  = Sensor BH1750 =
  =================
  The BH1750 is a digital ambient light level sensor.
  connections:
  - VCC -> 3V3
  - GND -> GND
  - SCL -> SCL
  - SDA -> SDA
  - ADD -> GND
    
  ADD pin is used to set sensor I2C address. If it has voltage greater or equal to
  0.7VCC voltage (e.g. you've connected it to VCC) the sensor address will be
  0x5C. In other case (if ADD voltage less than 0.7 * VCC) the sensor address will
  be 0x23 (by default).

  =======================
  = NeoPixel LED =
  =======================
  The NeoPixel WS2812b is a RGB LED used give local eSense state feedback.
  connections:
  - VCC -> 3V3
  - GND -> GND
  - DI -> GPIO14
  - DO -> N/C

  Color Codes:
    - AccesPoint Mode | WHITE
    Once Connected to WiFi:
      - Calibrating | BLUE
      - Excellent (IAQ = 0-50) | GREEN
      - Good (IAQ = 51-100) | LIGHT-GREEN
      - Lightly Polluted (IAQ = 101-150) | YELLOW
      - Moderately Polluted (IAQ = 151-200) | ORANGE
      - Heavily Polluted (IAQ = 201-250) | RED
      - Severely Polluted (IAQ = 251-350) | MAGENTA
      - Extremely Polluted (IAQ > 351) | PINK
    
  =======================
  = Jumper =
  =======================
  The Juper (JP1) is used to switch enviroSense into Debug/Production mode.
  connections:
  - D1 -> GND: Debug Mode
  - D1 -> N/C: Production Mode 

  Debug Mode: Useful information will be published in the Serial Port.
  Production Mode. Few infromation will be published to the Serial Port.

*/

#include <Arduino.h>                // https://github.com/arduino/ArduinoCore-avr
#include <ESP8266WiFi.h>            // https://github.com/esp8266/Arduino
#include <WiFiManager.h>            // https://github.com/tzapu/WiFiManager/tree/development
#include <PubSubClient.h>           // https://github.com/knolleary/pubsubclient
#include <ArduinoJson.h>            // https://github.com/bblanchon/ArduinoJson
#include "bsec.h"                   // https://github.com/BoschSensortec/BSEC-Arduino-library
#include <BH1750.h>                 // https://github.com/claws/BH1750
#include <Adafruit_NeoPixel.h>      // https://github.com/adafruit/Adafruit_NeoPixel
#include "mqtt.configuration.h"     // MQTT file configuration.

// ========== Start Device User Parametrization ================================================================

unsigned long mqttSensorsReportPeriod = 60000; // Sensors Report Period (Miliseconds).
unsigned int mqttDeviceReportPeriod = 10; // Device Report Period (times) based on MQTTSensorsReportPeriod.
uint16_t brightness = 100; //NeoPixel Brightness. [0-255]
int resetPortal = 180; //Number of seconds until the WiFiManager resests ESP8266.
#define AP_Password "eSense.movasim"

// ========== Start Device Development Parametrization (ONLY MODIFY WHEN NEW HW/FW VERSION IS RELASED) =========

#define DeviceType "eSense" 
#define DeviceModel "Rubino"
#define DeviceVersion "1.1.1"
#define FirmwareFlavor "Basic-Auth"
#define FirmwareVersion "1.0.0"
#define BME680_ADDR 0X77
#define BH1750_ADDR 0X23
#define NeoPixel 14
#define JUMPER 10
#define SCL D1
#define SDA D2

// ========== End Device Parametrization =======================================================================

const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;
const char* mqtt_server = MQTT_SERVER;
IPAddress mqtt_server_ip(MQTT_SERVER_IP);
int mqtt_server_port = MQTT_SERVER_PORT;
const char* mqtt_clientId;
const char* mqtt_publish_topic = MQTT_PUBLISH_TOPIC;
const char* mqtt_subscribe_topic = MQTT_SUBSCRIBE_TOPIC;
unsigned long currentTime;
unsigned long previousTime1 = 0;
unsigned long previousTime2 = 0;
unsigned int counter1 = 0;
float rawTemperature, pressure, rawHumidity, gasResistance, iaq, temperature, humidity, staticIaq, co2Equivalent, breathVocEquivalent;
byte iaqAccuracy;
unsigned int lux;
String output, strClientId; 
bool debug;

// Helper functions declarations
String GetDeviceName(void);
String GetMyMACAddress(void);
void reconnectMqttBroker(void);
void messageReceived(char* p_topic, byte* p_payload, unsigned int p_length);
void checkIaqSensorStatus(void);
void errLeds(void);
void RGB_color(int red_light_value, int green_light_value, int blue_light_value, uint16_t brightness);
void publishSensorsData(float raw_t, float t, float p, float raw_h, float h, float iaq, float s_iaq, byte iaq_acy, float gas_rst, float co2_eq, float bvoc_eq, unsigned int lux);
void publishDeviceData(String dev_t, String dev_m, String dev_v, String fw_f, String fw_v, String mac, String ip, byte s_qty, unsigned long up, String rst_r, unsigned int free_heap, byte heap_frg);

// Create  object of the class.
Bsec iaqSensor;
BH1750 lightMeter(BH1750_ADDR);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, NeoPixel, NEO_GRB + NEO_KHZ800);
WiFiClient espClient;
PubSubClient client(espClient);

void setup(void)
{
  Serial.begin(115200);
  Wire.begin(SDA, SCL);
  pixels.begin(); 

  // Check JUMPER Status.
  pinMode(JUMPER, INPUT_PULLUP);
  Serial.println();
  if (digitalRead(JUMPER) == LOW)
  {
    debug = true;
    Serial.println("DEBUG Mode ON");
  } else
  {
    debug = false;
    Serial.println("DEBUG Mode OFF");
  }
    
  // Generate ClientID with Device name and MAC.
  strClientId = GetDeviceName();
  mqtt_clientId = strClientId.c_str();
 
  RGB_color(255, 255, 255, brightness); // Connecting to WiFi | WHITE

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
 
  if (debug == true)
  {
    wifiManager.setDebugOutput(true); // Send debugging info to serial port.
    //wifiManager.resetSettings(); //Reset settings - wipe credentials for testing
  } else
  {
    wifiManager.setDebugOutput(false);
  }

  wifiManager.setConfigPortalTimeout(resetPortal); // auto close configportal after n seconds

  // Automatically connect using saved credentials,
  // if connection fails, it starts an access point with the specified name ( "ES-XXXX with the last 4 digits of the MAC Address"),
  // then goes into a blocking loop awaiting configuration and will return success result
  String APName = GetDeviceName();
  bool res;
  res = wifiManager.autoConnect((const char*)APName.c_str(),AP_Password); // password protected ap
  if(!res)
  {
    Serial.println("Failed to connect");
    ESP.restart();
  } 
  else
  {
    // If you get here you have connected to the WiFi    
    Serial.println("Connected to the WiFi Network!");
  }

  // Setup the MQTT Client
  //client.setServer(mqtt_server_ip, mqtt_server_port); // Connect to the MQTT Broker using IP
  client.setServer(mqtt_server, mqtt_server_port); // Connect to the MQTT Broker using URL
  client.setBufferSize(2048);
  client.setCallback(messageReceived);

  // Initialize BH1750
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 Started OK in CONTINUOUS_HIGH_RES_MODE"));
  }
  else {
    Serial.println(F("Error initialising BH1750"));
  }
 
  // Initializes BME680
  iaqSensor.begin(BME680_ADDR, Wire);
  output = "BSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();

  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
}

// Function that is looped forever
void loop(void)
{
  currentTime = millis();
  
  // Check if MQTT connection is active. Otherwise reconnect.
  if (!client.connected())
    {
      reconnectMqttBroker();
    }
  client.loop();

  if (iaqSensor.run()) 
  { // If new data is available
      rawTemperature = iaqSensor.rawTemperature;
      temperature = iaqSensor.temperature;
      pressure = iaqSensor.pressure / 100.0F;;
      rawHumidity = iaqSensor.rawHumidity;
      humidity = iaqSensor.humidity;
      iaq = iaqSensor.iaq;
      staticIaq = iaqSensor.staticIaq;
      iaqAccuracy = iaqSensor.iaqAccuracy;
      gasResistance = iaqSensor.gasResistance;
      co2Equivalent = iaqSensor.co2Equivalent;
      breathVocEquivalent = iaqSensor.breathVocEquivalent;
      
      if (iaqAccuracy == 0)
      RGB_color(0, 0, 255, brightness); // Callibrating | BLUE
      else if (staticIaq < 50)
        RGB_color(0, 255, 0, brightness); // Excellent (IAQ = 0-50) | GREEN
      else if ((staticIaq >= 50) & (staticIaq < 100))
        RGB_color(50, 255, 50, brightness); // Good (IAQ = 51-100) | LIGHT-GREEN
      else if ((staticIaq >= 100) & (staticIaq < 150))
        RGB_color(255, 170, 0, brightness); // Lightly Polluted (IAQ = 101-150) | YELLOW
      else if ((staticIaq >= 150) & (staticIaq < 200))
        RGB_color(255, 34, 0, brightness); // Moderately Polluted (IAQ = 151-200) | ORANGE
      else if ((staticIaq >= 200) & (staticIaq < 250))
        RGB_color(255, 0, 0, brightness); // Heavily Polluted (IAQ = 201-250) | RED
      else if ((staticIaq >= 250) & (staticIaq < 350))
        RGB_color(255, 0, 51, brightness); // Severely Polluted (IAQ = 251-350) | MAGENTA
      else
        RGB_color(255, 51, 119, brightness); // Extremely Polluted (IAQ > 351) | PINK
  } else 
  {
    checkIaqSensorStatus();
  }

   lux = lightMeter.readLightLevel(); // Reads BH1750.

    if (currentTime - previousTime1 >= mqttSensorsReportPeriod)
    {
      previousTime1 = currentTime;
      previousTime2 = currentTime;
      counter1++;

      publishSensorsData(rawTemperature, temperature, pressure, rawHumidity, humidity, iaq, staticIaq, iaqAccuracy, gasResistance, co2Equivalent, breathVocEquivalent, lux);
    }

    if ((counter1 == mqttDeviceReportPeriod) && (currentTime - previousTime2 >= (mqttSensorsReportPeriod/2)))
    {
      counter1 = 0;

      String deviceType = DeviceType;
      String deviceModel = DeviceModel;
      String deviceVersion = DeviceVersion;
      String firmwareFlavor = FirmwareFlavor;
      String firmwareVersion = FirmwareVersion;
      String deviceMAC = GetMyMACAddress();
      String deviceIP = WiFi.localIP().toString();

      // Device WiFi Signal Quality
      byte signalQuality = 0; 
      long rssi = WiFi.RSSI();
      // dBm to Signal Quality [%]:
        if(rssi <= -100)
            signalQuality = 0;
        else if(rssi >= -50)
            signalQuality = 100;
        else
            signalQuality = 2 * (rssi + 100);

      unsigned long deviceUptime = millis(); // Device Uptime
      String deviceResetReason = ESP.getResetReason(); // Returns a String containing the last reset reason in human readable format.
      unsigned int deviceFreeHeap = ESP.getFreeHeap(); // Returns the free heap size.
      byte deviceHeapFragmentation = ESP.getHeapFragmentation(); // Returns the fragmentation metric (0% is clean, more than ~50% is not harmless)

      publishDeviceData(deviceType, deviceModel, deviceVersion, firmwareFlavor, firmwareVersion, deviceMAC, deviceIP, signalQuality, deviceUptime, deviceResetReason, deviceFreeHeap, deviceHeapFragmentation);
    }
}

// Helper function definitions

// Function to get the Device Name. Device name is DeviceType + the last 4 MAC characters.
// This Device Name is used for WiFi SSID and MQTT clientId.
String GetDeviceName()
{
  String ssid1 = DeviceType;
  uint8_t mac[6];
  char macStr[6] = {0};
  String ssidDeviceName;
  WiFi.macAddress(mac);
  sprintf(macStr, "%02X%02X", mac[4], mac[5]);
  ssidDeviceName = ssid1 + String(macStr);
  return  ssidDeviceName;  
}

// Function to get the Device MAC Address.
String GetMyMACAddress()
{
  uint8_t mac[6];
  char macStr[18] = {0};
  WiFi.macAddress(mac);
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],  mac[1], mac[2], mac[3], mac[4], mac[5]); 
  return  String(macStr);
}

//Function to connect to MQTT Broker.
void reconnectMqttBroker()
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection");
    // Attempt to connect
      if (client.connect(mqtt_clientId, mqtt_user, mqtt_password))
      {
      if (debug == true)
      {
        Serial.print(" with clientID ");
        Serial.print(mqtt_clientId);
      } 
      Serial.println("...Connected!");
      // Subscribe to topics
      client.subscribe(mqtt_subscribe_topic);
      if (debug == true)
      {
        Serial.print("Subscribed to topic: ");
        Serial.println(mqtt_subscribe_topic);
      }
    } else
    {
      Serial.print("...failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// function called when a MQTT message arrived.
void messageReceived(char* topic, byte* payload, unsigned int length)
{
  if (debug == true)
  {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i=0;i<length;i++) 
    {
      Serial.print((char)payload[i]);
    }
    Serial.println();
  }

  // Deserialize the JSON document
  DynamicJsonDocument jsonReceivedCommand(2048);
  DeserializationError error = deserializeJson(jsonReceivedCommand, payload);

  // Test if parsing succeeds.
  if (error)
  {
    if (debug == true)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }
    return;
  }
  
  // Fetch values.
  if(jsonReceivedCommand["value"].as<String>() == "restart")
  {
    if (debug == true)
    {
      Serial.print("Disconnecting from ");
      Serial.println(mqtt_server);
    }
    client.disconnect();
    if (debug == true)
    {
      Serial.println("ESP8266 is going to be restarted");
    }
    delay (1000);
    ESP.restart();
  }
}

// Function to check BME680 status.
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }
  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

// Function that dislays ERROR with the NodeMCU builtin LED.
void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

// Function to set the NeoPixel color.
void RGB_color(int red_light_value, int green_light_value, int blue_light_value, uint16_t brightness)
 {
  pixels.setPixelColor(0, pixels.Color((brightness*red_light_value/255), (brightness*blue_light_value/255), (brightness*green_light_value/255)));
  pixels.show(); // This sends the updated pixel color to the hardware.
}

// function called to publish Sensors data (Temperature, Pressure, Humidity, IAQ and Lux).
void publishSensorsData(float raw_t, float t, float p, float raw_h, float h, float iaq, float s_iaq, byte iaq_acy, float gas_rst, float co2_eq, float bvoc_eq, unsigned int lux)
{
  StaticJsonDocument<256> jsonSensorsData;
  jsonSensorsData["msg_type"] = "srs";
  // BME680
  jsonSensorsData["raw_t"] = raw_t;
  jsonSensorsData["t"] = t;
  jsonSensorsData["p"] = p;
  jsonSensorsData["raw_h"] = raw_h;
  jsonSensorsData["h"] = h;
  jsonSensorsData["iaq"] = iaq;
  jsonSensorsData["s_iaq"] = s_iaq;
  jsonSensorsData["iaq_acy"] = iaq_acy;
  jsonSensorsData["gas_rst"] = gas_rst;
  jsonSensorsData["co2_eq"] = co2_eq;
  jsonSensorsData["bvoc_eq"] = bvoc_eq;
  // BH1750
  jsonSensorsData["lux"] = lux;
  
  char buffer[256];
  serializeJson(jsonSensorsData, buffer);
  if (debug == true)
  {
    Serial.println(buffer);
  }
  client.publish(mqtt_publish_topic, buffer, true);
  yield();
}

// function called to publish Device information (Type, Model, Version, Firmware Flavor, Firmware version, MAC, IP, WiFi Signal Quality, Uptime, etc.).
void publishDeviceData(String dev_t, String dev_m, String dev_v, String fw_f, String fw_v, String mac, String ip, byte s_qty, unsigned long up, String rst_r, unsigned int free_heap, byte heap_frg)
{
  StaticJsonDocument<512> jsonDeviceData;
  jsonDeviceData["msg_type"] = "dev";
  // Device
  jsonDeviceData["dev_t"] = dev_t;
  jsonDeviceData["dev_m"] = dev_m;
  jsonDeviceData["dev_v"] = dev_v;
  jsonDeviceData["fw_f"] = fw_f;
  jsonDeviceData["fw_v"] = fw_v;
  jsonDeviceData["mac"] = mac;
  jsonDeviceData["ip"] = ip;
  jsonDeviceData["s_qty"] = s_qty;
  jsonDeviceData["up"] = up;
  jsonDeviceData["rst_r"] = rst_r;
  jsonDeviceData["free_heap"] = free_heap;
  jsonDeviceData["heap_frg"] = heap_frg;

  char buffer[512];
  serializeJson(jsonDeviceData, buffer);
  if (debug == true)
  {
    Serial.println(buffer);
  }
  client.publish(mqtt_publish_topic, buffer, true);
  yield();
}
