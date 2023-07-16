#include "credentials.h"
#include "mqtt.h"
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>

// https://tutorials-raspberrypi.de/esp32-ph-sensor-automatische-messung-pool-hydroponik-esphome/

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// these are known aproximative values. google for the pH of vinnegar and soap.
float pHSoap = 9.5;
float phVinnegar = 2.5;

// dip the probe in vinnegar and in soap and put the read values here (in V)
float readingSoapInV = 2.15; // V
float readingVinnegarInV = 2.96; // V

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicString = topic;
  Serial.print("Message arrived [");
  Serial.print(topicString);
  Serial.print("] ");
  payload[length] = 0;
  String payloadString = String((char *)payload);
  Serial.print(payloadString);
  Serial.println();
  if (topicString.equals(MQTT_CALIBRATION_SOAP_SET_TOPIC)) {
    Serial.print("Set soap calibration to ");
    float payloadFloat = payloadString.toFloat();
    Serial.println(payloadFloat);
    readingSoapInV = payloadFloat;
  }
  if (topicString.equals(MQTT_CALIBRATION_VINEGAR_SET_TOPIC)) {
    Serial.print("Set vinegar calibration to ");\
    float payloadFloat = payloadString.toFloat();
    Serial.println(payloadFloat);
    readingVinnegarInV = payloadFloat;
  }
}

void setup()
{
  Serial.begin(115200);
  WiFi.begin(STASSID,STAPSK);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.macAddress());
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(MQTT_NAME)) {
      Serial.println("connected");
      sendMQTTDiscoveryMsg();
      mqttClient.subscribe(MQTT_CALIBRATION_SOAP_SET_TOPIC);
      mqttClient.subscribe(MQTT_CALIBRATION_VINEGAR_SET_TOPIC);
      updateStatus();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void sendMQTTDiscoveryMsg() {
  DynamicJsonDocument doc(1024);
  char buffer[256];
  doc["name"] = MQTT_NAME;
  doc["state_topic"] = MQTT_STATE_TOPIC;
  doc["stat_t"]   = MQTT_STATE_TOPIC;
  size_t n = serializeJson(doc, buffer);
  mqttClient.publish(MQTT_DISCOVERY_TOPIC, buffer, n);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  updateStatus();
  delay(1000);
}

const int pHPin = GPIO32;
const int temperaturePin = GPIO33;

unsigned long int avgValue;
int buf[10],temp;
void updateStatus() {
    for(int i=0;i<10;i++)  {
    buf[i]=analogRead(pHPin);
    delay(30);
  }
  for(int i=0;i<9;i++)  {
    for(int j=i+1;j<10;j++)    {
      if(buf[i]>buf[j])      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++) {
    avgValue+=buf[i];
  }
  float volts=(float)avgValue * 5.0 / 1024 / 6;
  float factor = (phVinnegar - pHSoap) / (readingVinnegarInV - readingSoapInV);
  float calibration = pHSoap - readingSoapInV * factor; // 23; //change this value to calibrate
  float phValue = factor * volts + calibration;
  Serial.print("V= ");
  Serial.print(volts);
  Serial.print(", pH= ");
  Serial.println(phValue);
  char cstr[16];
  dtostrf(phValue, 4, 6, cstr);
  mqttClient.publish(MQTT_STATE_TOPIC, cstr);
  dtostrf(volts, 4, 6, cstr);
  mqttClient.publish(MQTT_VOLTAGE_TOPIC, cstr);
  dtostrf(readingVinnegarInV, 4, 6, cstr);
  mqttClient.publish(MQTT_CALIBRATION_VINEGAR_TOPIC, cstr);
  dtostrf(readingSoapInV, 4, 6, cstr);
  mqttClient.publish(MQTT_CALIBRATION_SOAP_TOPIC, cstr);

}
