#include <FS.h>
#include <memory>
#include <ESP8266WebServer.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>


#include <SparkFun_Si7021_Breakout_Library.h>
#include <SparkFunCCS811.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

#ifdef DEBUG_ESP_PORT
#define DEBUG_MSG(...) DEBUG_ESP_PORT.printf( __VA_ARGS__ )
#else
#define DEBUG_MSG(...)
#endif

#define CCS811_ADDR 0x5A 
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

CCS811 co2sensor(CCS811_ADDR);

float humidity = 0;
float temp = 0;

Weather sensor;

Adafruit_NeoPixel leds = Adafruit_NeoPixel(2, ledPin, NEO_RGB + NEO_KHZ800);

WiFiClient mqttTcpClient;
PubSubClient mqttClient(mqttTcpClient);

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  shouldSaveConfig = true;
}


void setup_wifi() {

  if (SPIFFS.begin()) {
    DEBUG_MSG("mounted file system");

    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      DEBUG_MSG("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());

        if (json.success()) {
          String toto;
          strcpy(mqtt_server, json["mqtt_server"]);
          mqtt_port=json["mqtt_port"];
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_password, json["mqtt_password"]);

        } else {
          DEBUG_MSG("failed to load json config");
        }
      }
    }
  } else {
    DEBUG_MSG("failed to mount FS");
  }


  leds.setPixelColor(0, 0x10, 0x00, 0x00);
  leds.setPixelColor(1, 0x00, 0x00, 0x00);
  leds.show();
  

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 32);
  char mqtt_port_str[5];
  sprintf(mqtt_port_str,"%i", mqtt_port);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port_str, 5);
  WiFiManagerParameter custom_mqtt_login("login", "mqtt login", mqtt_user, 32);
  WiFiManagerParameter custom_mqtt_password("password", "mqtt password", mqtt_password, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
    
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_login);
  wifiManager.addParameter(&custom_mqtt_password);
  

  leds.setPixelColor(0, 0x00, 0x00, 0x00);
  leds.setPixelColor(1, 0x10, 0x00, 0x00);
  leds.show();

  if (!wifiManager.autoConnect("AutoConnectAP")) {
    DEBUG_MSG("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }


  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  mqtt_port= atoi(custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_login.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    DEBUG_MSG("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_password"] = mqtt_password;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      DEBUG_MSG("failed to open config file for writing");
    }

    //json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
  }
  DEBUG_MSG("local ip : %s \n",WiFi.localIP().toString().c_str());
  DEBUG_MSG("Set server : %s:%i, username : %s \n",mqtt_server, mqtt_port, mqtt_user);
  
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqtt_callback);
}

void mqtt_reconnect() {
  while (!mqttClient.connected()) {
    DEBUG_MSG("Attempting MQTT connection...\n");
    if (mqttClient.connect("co2sensor", mqtt_user, mqtt_password)) {
      DEBUG_MSG("connected\n");
      // Once connected, publish an announcement...
      // client.publish("outTopic", "hello world");
      // ... and resubscribe
      mqttClient.subscribe("test/co2config");
    } else {
      DEBUG_MSG("failed, rc=%i\n",mqttClient.state());
      DEBUG_MSG(" try again in 5 seconds\n");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
/*  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }
*/
  std::unique_ptr<char[]> buf(new char[length+1]);
  memcpy(buf.get(),payload,length);
  buf[length] = 0;

  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf.get());
  if (json.success()) {
    temp_correction = json["temp_correction"];
  }
}

void init_co2_sensor() {
  // reset sensor
  pinMode(rstPin, OUTPUT);
  digitalWrite(rstPin, 0);
  delay(200);
  digitalWrite(rstPin, 1);
  delay(500);

  //It is recommended to check return status on .begin(), but it is not
  //required.
  CCS811Core::status returnCode = co2sensor.begin();
  while (returnCode != CCS811Core::SENSOR_SUCCESS)
  {
    Serial.print(".begin() returned with an error: ");
    Serial.println(returnCode);
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
    CCS811Core::status returnCode = co2sensor.begin();  
    
  }

 // co2sensor.setDriveMode(2);
  co2sensor.disableInterrupts();

}


void setup()
{
  Serial.begin(74880);

  Serial.print(F("Starting"));
  DEBUG_MSG("Debug Message\n");

  leds.begin();
  leds.setPixelColor(0, 0x00, 0x00, 0x00);
  leds.setPixelColor(1, 0x00, 0x00, 0x00);
  leds.show();

  setup_wifi();

  
  leds.setPixelColor(0, 0x10, 0x10, 0x10);
  leds.setPixelColor(1, 0x00, 0x00, 0x00);
  leds.show();
  
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(100000);

  init_co2_sensor();
//Initialize the I2C sensors and ping them
  sensor.begin();
  sensor.reset();
}

uint32_t getColorFromValue(uint16_t valuePPM) {
  
  if(valuePPM < 200) {
    return Adafruit_NeoPixel::Color(0,1,0);
  } else if (valuePPM < 500) {
    return Adafruit_NeoPixel::Color(0,5,0);    
  } else if (valuePPM < 1000) {
    return Adafruit_NeoPixel::Color(0,10,10);    
  }
  return Adafruit_NeoPixel::Color(15,0,0);    
}

void publish_enviroment_data(uint16_t co2, uint16_t tvoc, float humidity, float temperature){
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["co2"] = co2;
  json["tvoc"] = tvoc;
  json["humidity"] = humidity;
  json["temperature"] = temperature;
  char message[256];
  json.printTo(message);
  mqttClient.publish("test/co2", message, true);
}

void i2crecovery() {
  pinMode(sdaPin, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(sclPin, INPUT_PULLUP);
  delay(2500);
  boolean slc_low = (digitalRead(sclPin) == LOW); // Check is SCL is Low.
  if (slc_low) { //If it is held low Arduno cannot become the I2C master.
    DEBUG_MSG("SCL held low, can not recover\n");
    return; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean sda_low = (digitalRead(sdaPin) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (sda_low && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(sclPin, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(sclPin, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(sclPin, INPUT); // release SCL LOW
    pinMode(slc_low, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    slc_low = (digitalRead(sclPin) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (slc_low && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      slc_low = (digitalRead(sclPin) == LOW);
    }
    if (slc_low) { // still low after 2 sec error
    DEBUG_MSG("SCL held low, can not recover by slave clock stretch for >2sec\n");
      return; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    sda_low = (digitalRead(sdaPin) == LOW); //   and check SDA input again and loop
  }
  if (sda_low) { // still low
    DEBUG_MSG("Could not clear. SDA data line held low\n");
    return; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(sdaPin, INPUT); // remove pullup.
  pinMode(sdaPin, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(sdaPin, INPUT); // remove output low
  pinMode(sdaPin, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS

  DEBUG_MSG("bus recovery done, restart communication in 2s");
  //return to power up mode
  pinMode(sdaPin, INPUT);
  pinMode(sclPin, INPUT);
  delay(2000);
  Wire.begin(sdaPin, sclPin);

}

int nb_values = 0;
uint16_t sum_values_co2 = 0;
uint16_t sum_values_tvoc = 0;

void loop()
{
  // handle incomming mqtt message
  if (!mqttClient.connected()) {
    mqtt_reconnect();
  }
  mqttClient.loop();

  nb_loop_without_value++;

  if(nb_loop_without_value > 30) {
    // try to restart i2c
    i2crecovery();
  }

  if(nb_loop_without_value > 35) {
    init_co2_sensor();
    // give time to startup again
    nb_loop_without_value = 0;
    nb_restart++;
  }

  if(nb_restart > 4) {
    ESP.restart();
  }

  //Check to see if data is ready with .dataAvailable()
  if  (co2sensor.dataAvailable())
  {
    //If so, have the sensor read and calculate the results.
    //Get them later
    if(co2sensor.readAlgorithmResults() == CCS811Core::SENSOR_SUCCESS) {
      nb_loop_without_value = 0;
      nb_restart = 0;
      Serial.print("CO2[");
      //Returns calculated CO2 reading
      Serial.print (co2sensor.getCO2());
      Serial.print("] tVOC[");
      //Returns calculated TVOC reading
      Serial.print (co2sensor.getTVOC());
      Serial.print("] millis[");
      //Simply the time since program start
      Serial.print(millis());
      Serial.print("]");
      Serial.println();
  
      leds.setPixelColor(0, getColorFromValue(co2sensor.getCO2()));
      leds.setPixelColor(1, getColorFromValue(co2sensor.getTVOC()));
      leds.show();
  
      nb_values++;
      sum_values_co2+=co2sensor.getCO2();
      sum_values_tvoc+=co2sensor.getTVOC();

      if(nb_values >=10) {
        getWeather();
        co2sensor.setEnvironmentalData(humidity,temp);
        printInfo();
    
        publish_enviroment_data(sum_values_co2 / nb_values,
                                sum_values_tvoc / nb_values,humidity, temp);

        nb_values = 0;
        sum_values_co2 = 0;
        sum_values_tvoc = 0;
   
      }
      
    }
  } 

  delay(200); //Don't spam the I2C bus
}

//---------------------------------------------------------------
void getWeather()
{
  // Measure Relative Humidity from the HTU21D or Si7021
  humidity = sensor.getRH();

  // Measure Temperature from the HTU21D or Si7021
  temp = sensor.getTemp() + temp_correction;
  // Temperature is measured every time RH is requested.
  // It is faster, therefore, to read it from previous RH
  // measurement with getTemp() instead with readTemp()
}

void printInfo()
{
//This function prints the weather data out to the default Serial Port

  Serial.print("Temp:");
  Serial.print(temp);
  Serial.print("C, ");

  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.println("%");
}
