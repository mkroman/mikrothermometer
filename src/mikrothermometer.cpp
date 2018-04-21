#include "Config.h"

#ifdef MKTHERMOMETER_DHT22
# include <DHT.h>
#elif defined(MKTHERMOMETER_SHT30)
# include "SHT3X.hpp"
#endif
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

extern "C" {
  #include "user_interface.h"
}

#ifdef MKTHERMOMETER_DHT22
static DHT g_dht(config::DHT_PIN, DHT22);
#endif
static WiFiClient g_wifi_client;
static PubSubClient g_mqtt_client(g_wifi_client);

uint8_t crc8(const uint8_t* data, int length);
void failure_deep_sleep(const char* message);
//   // Extracted from the datasheet:
//   // Width: 8-bit
//   // Polynomial: 0x31 (x⁸ + x⁵ + x⁴ + 1)
//   // Initialization: 0xFF
//   // Final XOR: 0x00
//   const uint8_t POLYNOMIAL = 0x31;
//   uint8_t crc = 0xFF;
//
//   for (int j = length; j; --j) {
//     crc ^= *data++;
//
//     for (int i = 8; i; --i) {
//       crc = (crc & 0x80)
//         ? (crc << 1) ^ POLYNOMIAL
//         : (crc << 1);
//     }
//   }
//
//   return crc;
// }

typedef struct measurement {
  double temperature;
  double humidity;
} measurement_t;

// The amount of milliseconds after program start until we tried establishing connection.
static unsigned long g_wifi_start = 0;

// The amount of milliseconds after program start until we have a wifi connection.
static unsigned long g_wifi_finish = 0;

// The amount of milliseconds after program start and try connecting to mqtt.
static unsigned long g_mqtt_start = 0;

// The amount of milliseconds after program start until we successfully connect to mqtt.
static unsigned long g_mqtt_finish = 0;

void setup_wifi(void) {
  unsigned int connection_attempts = 0;

  g_wifi_start = millis();

  // Enter station mode.
  WiFi.mode(WIFI_STA);

  Serial.print("Connecting to ");
  Serial.print(config::WIFI_SSID);

  WiFi.hostname(config::SENSOR_ID);
  WiFi.begin(config::WIFI_SSID, config::WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED && connection_attempts < 20) {
    delay(500);
    Serial.print(".");
    connection_attempts++;
  }

  // Unable to connect. Go directly to deep sleep.
  if (WiFi.status() != WL_CONNECTED) {
    failure_deep_sleep("Could not connect to WiFi");
  }

  Serial.println(" OK.");

  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("NM: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("GW: ");
  Serial.println(WiFi.gatewayIP());

#ifdef MKTHERMOMETER_RCWL_0516
  // Set the RCWL-0516 pin as input.
  pinMode(config::RCWL_0516_PIN, INPUT);
#endif

  g_wifi_finish = millis();
}

bool get_measurement(measurement_t* measurement) {
#if defined(MKTHERMOMETER_SHT30)
  SHT3X sht30(config::SHT30_ADDR);
  sht30.reset();

  double t = 0, h = 0;
  unsigned int res = sht30.getMeasurement(&t, &h);

  if (res != SHT3X_OK) {
    Serial.print("SHT30 failed: ");
    Serial.println(SHT3X::GetErrorMessage(res));

    return false;
  } else {
    measurement->temperature = t;
    measurement->humidity = h;

    return true;
  }
#else
  return false;
#endif
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nµthermometer v0.4");

  g_mqtt_client.setServer(config::MQTT_HOST, config::MQTT_PORT);

#ifdef MKTHERMOMETER_DHT22
  g_dht.begin();
#endif
}

/// Enters deep sleep right after printing the provided error message.
void failure_deep_sleep(const char* message) {
  Serial.print("ERROR: ");
  Serial.println(message);
  Serial.print("Going to enter deep sleep for ");
  Serial.print((unsigned long)config::POWER_DEEP_SLEEP_FAILURE_DURATION / 1000);
  Serial.println("ms");

  ESP.deepSleep(config::POWER_DEEP_SLEEP_FAILURE_DURATION);
}

/// Connects to the MQTT host. Will try 5 times.
///
/// Returns true on connection success, false otherwise.
bool mqtt_reconnect(void) {
  for (int i = 0; i < 5; i++) {
    if (g_mqtt_client.connect(config::MQTT_CLIENT_ID)) {
      return true;
    } else {
      delay(200);
    }
  }

  return false;
}

void publish_measurement(measurement_t* measurement) {
  // Build the JSON payload
  StaticJsonBuffer<200> json_buffer;
  JsonObject& root = json_buffer.createObject();

  root["temperature"] = measurement->temperature;
  root["humidity"] = measurement->humidity;

  root["wifi_conn_time_ms"] = (unsigned int)(g_wifi_finish - g_wifi_start);
  root["mqtt_conn_time_ms"] = (unsigned int)(g_mqtt_finish - g_mqtt_start);

  String payload;
  root.printTo(payload);

  Serial.println(payload);

  for (int i = 0; i < 5; i++) {
    if (g_mqtt_client.publish(config::MQTT_TOPIC, payload.c_str(), payload.length())) {
      return;
    } else {
      delay(200);
    }
  }

  Serial.println("Could not publish sensor information!");
}

void loop() {
  // Get the power reset cause.
  struct rst_info* rtc_info = system_get_rst_info();

  if (rtc_info == NULL) {
    failure_deep_sleep("Could not retrieve reset info!");
    ESP.restart();
  }

  measurement_t measurement;

  if (!get_measurement(&measurement)) {
    failure_deep_sleep("Measurement failed");
    return;
  }

  measurement.temperature = round(measurement.temperature * 10) / 10.0;
  measurement.humidity = round(measurement.humidity * 10) / 10.0;

  if (rtc_info->reason == REASON_DEEP_SLEEP_AWAKE) {
    // The previous temperature measurement should be saved in RTC memory.
    measurement_t old_measurement;
    ESP.rtcUserMemoryRead(0, (uint32_t*)&old_measurement, sizeof(measurement_t));

    if (old_measurement.temperature == measurement.temperature) {
      Serial.println("The temperature hasn't changed.");
      ESP.deepSleep(config::POWER_DEEP_SLEEP_DURATION / 2);
    }
  }

  ESP.rtcUserMemoryWrite(0, (uint32_t*)&measurement, sizeof(measurement_t));

  setup_wifi();

  if (!g_mqtt_client.connected() && !mqtt_reconnect()) {
    failure_deep_sleep("mqtt_reconnect: could not connect");
    return;
  }

  g_mqtt_finish = millis();

  publish_measurement(&measurement);

  Serial.println("Going to sleep");

  g_mqtt_client.disconnect();

  delay(100);

  ESP.deepSleep(config::POWER_DEEP_SLEEP_DURATION);
}
