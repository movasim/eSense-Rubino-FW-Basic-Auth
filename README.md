# eSense-Rubino-FW-Basic-Auth
### Basic-Authentication *(User/Password)* Firmware flavor for [eSense Rubino](https://github.com/movasim/eSense-Rubino).

This is the Basic-Authentication Firmware flavor of the eSense Rubino device, providing Client-Authentication by means of User-Password defined at compilation time. Its security features are described in the following table.

| Client Authentication | Server Authentication | Data Encryption | Secure Element |
| :-------------------- | :-------------------- | :-------------- | :------------- |
| Yes *(User/Pasword)*  | No                    | No              | No             |

The Firmware reads sensors and device status and periodically reports two types of MQTT messages to [**MOVASIM**](https://movasim.com/) **IoTek** Platform, both in JSON format.

The **Sensors Data** message contains the following information:

```json
{"msg_type":"srs","raw_t":27.45,"t":27.38531,"p":1019.54,"raw_h":48.041,"h":48.21092,"iaq":16.27032,"s_iaq":21.18662,"iaq_acy":1,"gas_rst":145526,"co2_eq":484.7465,"bvoc_eq":0.471993,"lux":234,"alias":"Production" }
```

Where:

| Key        | Key Description       | Value        | Value Description                                            |
| ---------- | --------------------- | ------------ | ------------------------------------------------------------ |
| `Msg_type` | Message Type          | `srs`        | Sensors Data                                                 |
| `raw_t`    | Raw Temperature       | `27.45`      | [ºC] Celsius Degrees                                         |
| `t`        | Temperature           | `27.38531`   | [ºC] Celsius Degrees                                         |
| `p`        | Pressure              | `1019.54`    | [hPa] Hectopascal                                            |
| `raw_h`    | Raw Humidity          | `48.041`     | [%] Percentage                                               |
| `h`        | Humidity              | `48.21092`   | [%] Percentage                                               |
| `iaq`      | IAQ Index             | `16.27032`   | [Index defined by Bosch Library](https://github.com/movasim/eSense-Rubino#Feature-Description) |
| `s_iaq`    | Static IAQ Index      | `21.18662`   | [Index defined by Bosch Library](https://github.com/movasim/eSense-Rubino#Feature-Description) |
| `iaq_acy`  | IAQ Accuracy          | `1`          | [State of the BME680 sensor calibration](https://github.com/movasim/eSense-Rubino#gas-sensor-measuring-relative-humidity-barometric-pressure-ambient-temperature-and-gas-voc). |
| `gas_rst`  | Gas Resistance        | `145526`     | Internal sensor gas resistance used by algotith to calculate IAQ. |
| `co2_eq`   | Co2 Equivalent        | `84.7465`    | Estimates a CO2-equivalent concentration [ppm]               |
| `bvoc_eq`  | Breath VOC Equivalent | `0.471993`   | Breath VOC equivalent estimates total VOC concentration in [ppm] |
| `lux`      | Illuminance           | `234`        | LUX                                                          |
| `alias`    | Alias                 | `Production` | Free Text *(Normally the device physical location)*          |

And the **Device Data** message contains the following information:

```json
{"msg_type":"dev","dev_t":"eSense","dev_m":"Rubino","dev_v":"1.1.1","fw_f":"Basic-Auth","fw_v":"1.0.0","mac":"2C:F4:32:7A:10:EE","ip":"192.168.4.125","s_qty":100,"up":630001,"rst_r":"External System","free_heap":41376,"heap_frg":2,"debug":true}
```

Where:

| Key         | Key Description           | Value               | Value Description                                            |
| ----------- | ------------------------- | ------------------- | ------------------------------------------------------------ |
| `Msg_type`  | Message Type              | `dev`               | Device Data                                                  |
| `dev_t`     | Device Type               | `eSense`            | -                                                            |
| `dev_m`     | Device Model              | `Rubino`            | -                                                            |
| `dev_v`     | Device Version            | `1.1.1`             | Hardware Version                                             |
| `fw_f`      | Firmware Flavor           | `Basic-Auth`        | -                                                            |
| `fw_v`      | Firmware Version          | `1.0.0`             | -                                                            |
| `mac`       | MAC                       | `2C:F4:32:7A:10:EE` | -                                                            |
| `ip`        | IP                        | `192.168.4.125`     | -                                                            |
| `s_qty`     | Signal Quality            | `100`               | Quality of the WiFi signal measured by ESP8266.              |
| `up`        | Uptime                    | `630001`            | Milliseconds since power-up.                                 |
| `rst_r`     | Reseat Reason             | `External System`   | Reason of the last Device Reset.                             |
| `free_heap` | Free Memory Heap          | `41376`             | free heap size in bytes.                                     |
| `heap_frg`  | Memory Heap Fragmentation | `2`                 | [%] Percentage. *(0% is clean, more than ~50% is not harmless)* |
| `debug`     | Debug Mode                | `true`              | Indicates whether Jumper (JP1) is in "Debug" or "Production" position [true:false] |

Periodicity of both reports to **IoTek** can be configured in the *"Device User Parametrization"* section, described in the section below.

## Firmware Configuration and Compilation

This project has been developed in [VSCODE/PlatformIO](VSCODE/PlatformIO), and includes all the necessary libraries for its compilation in the `/lib` directory. That is, by just downloading *(or cloning)* the project and editing one file name, it should compile without errors.

Once in the project's local folder, rename the file that is inside the `/include` directory from `mqtt.configuration.h` to `mqtt.configuration.example.h` *( just delete "example")*.

The content of the mqtt.configuration.example.h file is as follows:

```
// Replace with your credentials

#define MQTT_USER "user"
#define MQTT_PASSWORD "password"
#define MQTT_SERVER_IP 192, 168, 1, 1
#define MQTT_SERVER "mqtt.server.io"
#define MQTT_SERVER_PORT 1883
#define MQTT_PUBLISH_TOPIC  "t"
#define MQTT_SUBSCRIBE_TOPIC "command///req/#"
```

It is neccesary to edit the file contents with the MQTT data that should be provided by **IoTek Service** when the device is provisioned in the platform.

In summary, the steps to follow to compile and install the **eSense Rubino Basic-Authentication Firmware** on the microcontroller board are the following:

1. Download the project code *(or clone it in the desired directory)*.
2. Rename the file inside the `/include` directory from `mqtt.configuration.example.h` to `mqtt.configuration.h` *(just delete "example")*.
3. Edit the file giving it the MQTT information provided by **IoTek service** at the moment of provisioning your device.
4. Compile the project.
5. Download it to the eSense Rubino board via USB cable.

Only for those cases where is neccesary to modify pre-defined behaviuor of the Firmware, those modifications should be done in the "**Device User Parametrization**" section at the begining of the `src/main.cpp` file, shown below:

```c++
// ========== Start Device User Parametrization ================================================================

#define Alias "Production";                     // Friendly name for the Device location.
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
#define FirmwareVersion "1.1.0"
#define BME680_ADDR 0X77
#define BH1750_ADDR 0X23
#define NeoPixel 14
#define JUMPER 10
#define SCL D1
#define SDA D2

// ========== End Device Parametrization =======================================================================

```

instead, the "**Device Development Parametrization**" section should only be modified when a new device hardware or firmware versions are released.

## License

This project by [**MOVASIM**](https://movasim.com/) is licensed under the [MIT License](https://github.com/movasim/eSense-Rubino-FW-Basic-Auth/blob/main/LICENSE).

![](images/MIT.svg)



