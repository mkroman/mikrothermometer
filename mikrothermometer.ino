#include "Config.h"

#ifdef MKTHERMOMETER_DHT22
# include <DHT.h>
#elif defined(MKTHERMOMETER_SHT30)
# include "SHT3X.hpp"
#endif
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#ifdef MKTHERMOMETER_DHT22
static DHT g_dht(config::DHT_PIN, DHT22);
#endif
static WiFiClient g_wifi_client;
static PubSubClient g_mqtt_client(g_wifi_client);

// The amount of milliseconds after program start until we tried establishing connection.
static unsigned long g_wifi_start = 0;

// The amount of milliseconds after program start until we have a wifi connection.
static unsigned long g_wifi_finish = 0;

// The amount of milliseconds after program start and try connecting to mqtt.
static unsigned long g_mqtt_start = 0;

// The amount of milliseconds after program start until we successfully connect to mqtt.
static unsigned long g_mqtt_finish = 0;

void setup_wifi(void) {
  // Enter station mode.
  WiFi.mode(WIFI_STA);

  Serial.print("Connecting to ");
  Serial.print(config::WIFI_SSID);

  WiFi.hostname(config::SENSOR_ID);
  WiFi.begin(config::WIFI_SSID, config::WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
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
}

void setup() {
  Serial.begin(115200);
  
  Serial.println("\nµthermometer v0.3");

  g_wifi_start = millis();
  setup_wifi();
  g_wifi_finish = millis();
  
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

void publish_temp(void) {
  // Build the JSON payload
  StaticJsonBuffer<200> json_buffer;
  JsonObject& root = json_buffer.createObject();

#ifdef MKTHERMOMETER_DHT22
  float t = g_dht.readTemperature();
  float h = g_dht.readHumidity();

  if (!isnan(t)) {
    root["temperature"] = t;
    root["humidity"] = h;
  }
#elif defined(MKTHERMOMETER_SHT30)
  SHT3X sht30(config::SHT30_ADDR);

  sht30.reset();

  float t = 0, h = 0;
  unsigned int res = sht30.getMeasurement(&t, &h);

  if (res != SHT3X_OK) {
    Serial.print("SHT30 failed: ");
    Serial.println(SHT3X::GetErrorMessage(res));

    return;
  }

  root["temperature"] = t;
  root["humidity"] = h;
#endif

  root["wifi_conn_time_ms"] = (unsigned int)(g_wifi_finish - g_wifi_start);
  root["mqtt_conn_time_ms"] = (unsigned int)(g_mqtt_finish - g_mqtt_start);

#ifdef MKTHERMOMETER_LDR
  int ldr = analogRead(A0);
  uint8_t brightness = map(ldr, 0, 1024, 0, 100);

  root["brightness"] = brightness;
#endif

#ifdef MKTHERMOMETER_RCWL_0516
  root["motion"] = digitalRead(MKTHERMOMETER_RCWL_0516_PIN);
#endif

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
  g_mqtt_start = millis();
  
  if (!g_mqtt_client.connected() && !mqtt_reconnect()) {
    failure_deep_sleep("mqtt_reconnect: could not connect");
    return;
  }

  g_mqtt_finish = millis();

  publish_temp();
  Serial.println("Going to sleep");

  g_mqtt_client.disconnect();

  delay(100);

  ESP.deepSleep(config::POWER_DEEP_SLEEP_DURATION);
}
