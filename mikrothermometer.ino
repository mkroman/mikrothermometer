#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "Config.h"

static DHT g_dht(config::DHT_PIN, DHT22);
static WiFiClient g_wifi_client;
static PubSubClient g_mqtt_client(g_wifi_client);

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
  
  Serial.print(" OK. (IPv4: ");
  Serial.print(WiFi.localIP());
  Serial.println(")");


#ifdef MKTHERMOMETER_RCWL_0516
  // Set the RCWL-0516 pin as input.
  pinMode(MKTHERMOMETER_RCWL_0516_PIN, INPUT);
#endif
}

void setup() {
  Serial.begin(115200);
  
  Serial.println();
  Serial.println("µthermometer v0.1");
  
  setup_wifi();
  g_mqtt_client.setServer(config::MQTT_HOST, config::MQTT_PORT);
  g_dht.begin();
}

/// Enters deep sleep right after printing the provided error message.
void failure_deep_sleep(const char* message) {
  Serial.print("ERROR: ");
  Serial.println(message);
  Serial.print("Going to enter deep sleep for ");
  Serial.print(config::POWER_DEEP_SLEEP_FAILURE_DURATION / 1e6);
  Serial.println("s");

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

String build_mqtt_payload() {
  StaticJsonBuffer<200> json_buffer;
  JsonObject& root = json_buffer.createObject();

  float t = g_dht.readTemperature();
  float h = g_dht.readHumidity();

  if (!isnan(t)) {
    root["temperature"] = t;
    root["humidity"] = h;
  }

#ifdef MKTHERMOMETER_LDR
  int ldr = analogRead(A0);
  uint8_t brightness = map(ldr, 0, 1024, 0, 100);

  root["brightness"] = brightness;
#endif

#ifdef MKTHERMOMETER_RCWL_0516
  root["motion"] = digitalRead(MKTHERMOMETER_RCWL_0516_PIN);
#endif

  String result;
  root.printTo(result);
  return result;
}

void publish_temp(void) {
  String payload = build_mqtt_payload();

  Serial.println(payload);

  for (int i = 0; i < 5; i++) {
    if (g_mqtt_client.publish(config::MQTT_TOPIC, payload.c_str(), true)) {
      return;
    } else {
      delay(200);
    }
  }

  Serial.println("Could not publish sensor information!");
}

void loop() {
  if (!g_mqtt_client.connected() && !mqtt_reconnect()) {
    failure_deep_sleep("mqtt_reconnect: could not connect");
    return;
  }

  publish_temp();
  Serial.println("Going to sleep");

  ESP.deepSleep(config::POWER_DEEP_SLEEP_DURATION);
}
