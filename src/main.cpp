#include <Arduino.h>

#include <ESP8266WebServer.h>
#include <FS.h>
#include <memory>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiManager.h>

#include <Adafruit_NeoPixel.h>
#include <CCS811.h>
#include <SI7021.h>
#include <Wire.h>

#include <ESP8266LLMNR.h>
#include <ArduinoOTA.h>

#include "i2c_util.h"

#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(...) DEBUG_ESP_PORT.printf( __VA_ARGS__ )
#define DEBUG_MSG_PP(msg, ...) DEBUG_ESP_PORT.printf_P(PSTR(msg), ##__VA_ARGS__)
#else
#define DEBUG_MSG(...)
#define DEBUG_MSG_PP(msg, ...)
#endif

static const char config_topic[] PROGMEM = "test/co2config";
static const char light_topic[] PROGMEM = "test/co2light";
static const char host_name[] = "esp-co2";

const int ccs811Addr = 0x5A;
const int sclPin = D1;
const int sdaPin = D2;
const int intPin = D3;
const int rstPin = D4;
const int ledPin = D8;

char mqtt_server[32];
int mqtt_port = 1883;
char mqtt_user[32];
char mqtt_password[32];

int nb_loop_without_value = 0;
int nb_restart = 0;

float temp_correction = 0;
bool light_on = true;
CCS811 co2sensor;

float humidity = 0;
float temp = 0;

SI7021 sensor;

Adafruit_NeoPixel leds = Adafruit_NeoPixel(3, ledPin, NEO_RGB + NEO_KHZ800);

WiFiClient mqttTcpClient;

class PubSubClientPlus : public PubSubClient {
  public:
    PubSubClientPlus(Client& client) : PubSubClient(client) {

  };

  boolean subscribe(const __FlashStringHelper* topic) {
    std::unique_ptr<char[]> buf(new char[strlen_P((PGM_P)topic) + 1]);
    strcpy_P(buf.get(), (PGM_P)topic);

    return PubSubClient::subscribe(buf.get(), 0);
  }

};

PubSubClientPlus mqttClient(mqttTcpClient);




// flag for saving data
bool shouldSaveConfig = false;

// callback notifying us of the need to save config
void saveConfigCallback() { shouldSaveConfig = true; }

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
  std::unique_ptr<char[]> buf(new char[length + 1]);
  memcpy(buf.get(), payload, length);
  buf[length] = 0;

  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.parseObject(buf.get());

  if (json.success()) {
    if(strcmp_P(topic, config_topic) == 0) {
      temp_correction = json[F("temp_correction")];
    } else if (strcmp_P(topic, light_topic) == 0) {
      light_on = json[F("light_on")];
    }
    
  }
}

void setup_wifi() {

  if (SPIFFS.begin()) {
    DEBUG_MSG_PP("mounted file system");

    if (SPIFFS.exists("/config.json")) {
      // file exists, reading and loading
      DEBUG_MSG_PP("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject &json = jsonBuffer.parseObject(buf.get());

        if (json.success()) {
          String toto;
          strcpy(mqtt_server, json[F("mqtt_server")]);
          mqtt_port = json[F("mqtt_port")];
          strcpy(mqtt_user, json[F("mqtt_user")]);
          strcpy(mqtt_password, json[F("mqtt_password")]);
        } else {
          DEBUG_MSG_PP("failed to load json config");
        }
      }
    }
  } else {
    DEBUG_MSG_PP("failed to mount FS");
  }

  leds.setPixelColor(0, 0x10, 0x00, 0x00);
  leds.setPixelColor(1, 0x00, 0x00, 0x00);
  leds.show();

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server,
                                          32);
  char mqtt_port_str[5];
  sprintf(mqtt_port_str, "%i", mqtt_port);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port_str, 5);
  WiFiManagerParameter custom_mqtt_login("login", "mqtt login", mqtt_user, 32);
  WiFiManagerParameter custom_mqtt_password("password", "mqtt password",
                                            mqtt_password, 32);

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep
  // it around
  WiFiManager wifiManager;

  // set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_login);
  wifiManager.addParameter(&custom_mqtt_password);

  leds.setPixelColor(0, 0x00, 0x00, 0x00);
  leds.setPixelColor(1, 0x10, 0x00, 0x00);
  leds.show();

  if (!wifiManager.autoConnect("AutoConnectAP")) {
    DEBUG_MSG_PP("failed to connect and hit timeout\n");
    delay(3000);
    // reset and try again
    ESP.reset();
  }

  // read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  mqtt_port = atoi(custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_login.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());

  // save the custom parameters to FS
  if (shouldSaveConfig) {
    DEBUG_MSG_PP("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_password"] = mqtt_password;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      DEBUG_MSG_PP("failed to open config file for writing");
    }

    // json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
  }
  DEBUG_MSG_PP("local ip : %s \n", WiFi.localIP().toString().c_str());
  DEBUG_MSG_PP("Set server : %s:%i, username : %s \n", mqtt_server, mqtt_port,
            mqtt_user);

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_callback);
}

void mqtt_reconnect() {
  while (!mqttClient.connected()) {
    DEBUG_MSG_PP("Attempting MQTT connection...\n");
    if (mqttClient.connect("co2sensor", mqtt_user, mqtt_password)) {
      DEBUG_MSG_PP("connected\n");
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      mqttClient.subscribe(FPSTR (config_topic));
      mqttClient.subscribe(FPSTR (light_topic));
    } else {
      DEBUG_MSG_PP("failed, rc=%i\n", mqttClient.state());
      DEBUG_MSG_PP(" try again in 5 seconds\n");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void init_co2_sensor() {
  // reset sensor
  pinMode(rstPin, OUTPUT);
  digitalWrite(rstPin, 0);
  delay(200);
  digitalWrite(rstPin, 1);
  delay(500);

  while (!co2sensor.begin(ccs811Addr)) {
    Serial.print(".begin() returned with an error.\n");
    leds.setPixelColor(0, 0x05, 0x00, 0x00);
    leds.setPixelColor(1, 0x05, 0x00, 0x00);
    leds.show();
    delay(1000);
    Serial.println("I2C Status : ");
    Serial.println(Wire.status());
    Serial.println("Reinit I2C");
    Wire.begin(sdaPin, sclPin);

    Serial.println("Reset sensor");

    // reset sensor
    pinMode(rstPin, OUTPUT);
    digitalWrite(rstPin, 0);
    delay(200);
    digitalWrite(rstPin, 1);
    delay(500);
    leds.setPixelColor(0, 0x00, 0x00, 0x00);
    leds.setPixelColor(1, 0x00, 0x00, 0x00);
    leds.show();
    delay(4000);
  }

  co2sensor.setDriveMode(Ccs811Drive::mode_1sec);
  co2sensor.disableInterrupt();
}

void setup() {
  Serial.begin(74880);

  Serial.print(F("Starting\n"));
  DEBUG_MSG_PP("Debug Message\n");

  leds.begin();
  leds.setPixelColor(0, 0x00, 0x00, 0x00);
  leds.setPixelColor(1, 0x00, 0x00, 0x00);
  leds.show();

  setup_wifi();

  leds.setPixelColor(0, 0x10, 0x10, 0x10);
  leds.setPixelColor(1, 0x00, 0x00, 0x00);
  leds.show();

  ArduinoOTA.setHostname(host_name);
  ArduinoOTA.onStart([]() { // show
    leds.setPixelColor(0, 0x00, 0x00, 0x20);
    leds.setPixelColor(1, 0x00, 0x00, 0x20);
    leds.show();
  });

  ArduinoOTA.onEnd([]() {
    leds.setPixelColor(1, 0x00, 0x10, 0x10);
    leds.show();
  });

  ArduinoOTA.onError([](ota_error_t error) { 
    leds.setPixelColor(0, 0xFF, 0x00, 0x00);
    leds.show();
    ESP.restart(); 
  });

  /* setup the OTA server */
  ArduinoOTA.begin();
  LLMNR.begin(host_name);  

  Wire.begin(sdaPin, sclPin);
  Wire.setClock(100000);

  init_co2_sensor();
  // Initialize the I2C sensors and ping them
  sensor.begin(sdaPin, sclPin);
}

uint32_t getColorFromValue(uint16_t valuePPM) {

  if (valuePPM < 200) {
    return Adafruit_NeoPixel::Color(0, 1, 0);
  } else if (valuePPM < 500) {
    return Adafruit_NeoPixel::Color(0, 5, 0);
  } else if (valuePPM < 1000) {
    return Adafruit_NeoPixel::Color(0, 10, 10);
  }
  return Adafruit_NeoPixel::Color(15, 0, 0);
}

void publish_enviroment_data(uint16_t co2, uint16_t tvoc, float humidity,
                             float temperature) {
  DynamicJsonBuffer jsonBuffer;
  JsonObject &json = jsonBuffer.createObject();
  json[F("co2")] = co2;
  json[F("tvoc")] = tvoc;
  json[F("humidity")] = humidity;
  json[F("temperature")] = temperature;
  char message[256];
  json.printTo(message);
  mqttClient.publish("test/co2", message, true);
}

//---------------------------------------------------------------
void getWeather() {
  auto measure = sensor.getTempAndRH();
  humidity = measure.humidityPercent;
  temp = (measure.celsiusHundredths / 100.0) + temp_correction;
}

void printInfo() {
  // This function prints the weather data out to the default Serial Port

  Serial.printf_P(PSTR("Temp: %fC, Humidity: %f%%"), temp, humidity);
}

const int nb_values_for_avg = 10;
int nb_values = 0;
const int nb_values_for_env_correction = 3;
int nb_values_moy = 0;
uint32_t sum_values_co2 = 0;
uint32_t sum_values_tvoc = 0;

void loop() {
  ArduinoOTA.handle();
  // handle incomming mqtt message
  if (!mqttClient.connected()) {
    mqtt_reconnect();
  }
  mqttClient.loop();

  nb_loop_without_value++;

  if (nb_loop_without_value > 50) {
    // try to restart i2c
    DEBUG_MSG_PP("I2C recovery need loop without value : %i\n", nb_loop_without_value);
    i2c_util::i2crecovery(sdaPin, sclPin);
  }

  if (nb_loop_without_value > 55) {
    init_co2_sensor();
    // give time to startup again
    nb_loop_without_value = 0;
    nb_restart++;
  }

  // too much restart of I2C, restart all.
  if (nb_restart > 4) {
    ESP.restart();
  }

  // Readdata return true if data available
  if (co2sensor.available() && co2sensor.readData()) {
    nb_loop_without_value = 0;
    nb_restart = 0;
    Serial.print("CO2[");
    // Returns calculated CO2 reading
    Serial.print(co2sensor.geteCO2());
    Serial.print("] tVOC[");
    // Returns calculated TVOC reading
    Serial.print(co2sensor.getTVOC());
    Serial.print("] millis[");
    // Simply the time since program start
    Serial.print(millis());
    Serial.print("]");
    Serial.println();

    if(light_on) {
      leds.setPixelColor(0, getColorFromValue(co2sensor.geteCO2()));
      leds.setPixelColor(1, getColorFromValue(co2sensor.getTVOC()));
    } else {
      leds.setPixelColor(0, 0);
      leds.setPixelColor(1, 0);
    }
    leds.show();

    nb_values++;
    sum_values_co2 += co2sensor.geteCO2();
    sum_values_tvoc += co2sensor.getTVOC();

    if (nb_values >= nb_values_for_avg) {
      nb_values_moy++;
      getWeather();

      if (nb_values_moy > nb_values_for_env_correction) {
        nb_values_moy = 0;
        co2sensor.setEnvironmentalData(humidity, temp);
        printInfo();
        delay(100);
      }

      publish_enviroment_data(sum_values_co2 / nb_values,
                              sum_values_tvoc / nb_values, humidity, temp);

      nb_values = 0;
      sum_values_co2 = 0;
      sum_values_tvoc = 0;
    }
  } else {
    if(co2sensor.isI2CError()) {
      DEBUG_MSG_PP("I2C error : loop : %i\n", nb_loop_without_value);
    }
  }

  delay(500); // Don't spam the I2C bus
}
