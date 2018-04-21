#ifndef MKTHERMOMETER_CONFIG_H
#define MKTHERMOMETER_CONFIG_H

#include <c_types.h>

// Uncomment to enable LDR brightmess measurements.
// The LDR must be connected to the ADC.
// #define MKTHERMOMETER_LDR

// Uncomment to enable RCWL-0516 microwave sensor.
// #define MKTHERMOMETER_RCWL_0516

// Uncomment to enable SHT30 measurements.
#define MKTHERMOMETER_SHT30

namespace config {
  /// The WiFi AP SSID to connect to.
  static const char* WIFI_SSID = "secret";
  /// The WiFi password.
  static const char* WIFI_PASSWORD = "secret";
  /// Human-readable sensor ID. Must be unique on the network.
  static const char* SENSOR_ID = "living-room-sensor";

  /// The MQTT host to connect to.
  static const char* MQTT_HOST = "10.0.0.202";
  /// The MQTT port the host is listening on.
  static const uint16_t MQTT_PORT = 1883;
  /// The MQTT device ID.
  static const char* MQTT_CLIENT_ID = SENSOR_ID;
  /// The MQTT topic to publish to.
  static const char* MQTT_TOPIC = "living-room/sensor";

#ifdef MKTHERMOMETER_RCWL_0516
  /// The PGIO pin that the RCWL-0516 is connected to.
  static const uint8_t RCWL_0516_PIN = 5;
#endif

#ifdef MKTHERMOMETER_DHT22
  /// The GPIO pin the DHT22 is connected to.
  static const uint8_t DHT_PIN = 14;
#endif

#ifdef MKTHERMOMETER_SHT30
  /// The I2C address of the SHT30 sensor.
  static const uint16_t SHT30_ADDR = 0x45;
#endif

  /// The duration in µs that the ESP8266 will enter deep sleep before
  /// publishing sensor information again.
  static const uint64_t POWER_DEEP_SLEEP_DURATION = (5 * 60) * 1e6;
  /// The duration in µs that the ESP8266 wil sleep after failing while
  /// trying to publish sensor details before trying again.
  static const uint64_t POWER_DEEP_SLEEP_FAILURE_DURATION = (1 * 60) * 1e6;
}

#endif
