// Use this sketch for cabling test

#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_BME280.h>
#include <DallasTemperature.h>

#include <WiFiClient.h>
#include <PubSubClient.h>

#include "securities.h"

#define delay2 1000
#define maxRetr 20

#define fanPin D13
#define batAnalogPin A0
#define batReadings 10 // numero ripetizioni della lettura fra cui fare la media
#define readDelay 10   // in ms, attesa fra una lettura e l'altra
#define ONE_WIRE_BUS D8

#define TSL2561_INTEGRATIONTIME TSL2561_INTEGRATIONTIME_402MS  /* 16-bit data but slowest conversions */

#define mqttClientId "smaldinoHomeTerrace"
#define mqttTopic "announcement/clientid"
#define mqttTopicCmds   "it/smaldino/home/terrace/wemosd1/cmds"

void setup() {
  // put your setup code here, to run once:
  int i;
  Serial.begin(19200);
  Serial.println("==========================:");
  Serial.print("LED_BUILTIN : "); Serial.println(LED_BUILTIN);
  Serial.print("D0          : "); Serial.println(D0);
  Serial.print("D1          : "); Serial.println(D1);
  Serial.print("D2          : "); Serial.println(D2);
  Serial.print("D3          : "); Serial.println(D3);
  Serial.print("D4          : "); Serial.println(D4);
  Serial.print("D5          : "); Serial.println(D5);
  Serial.print("D6          : "); Serial.println(D6);
  Serial.print("D7          : "); Serial.println(D7);
  Serial.print("D8          : "); Serial.println(D8);
  Serial.print("D9          : "); Serial.println(D9);
  Serial.print("D10         : "); Serial.println(D10);
  Serial.print("D11         : "); Serial.println(D11);
  Serial.print("D12         : "); Serial.println(D12);
  Serial.print("D13         : "); Serial.println(D13);
  Serial.print("D14         : "); Serial.println(D14);
  Serial.print("D15         : "); Serial.println(D15);
  pinMode(LED_BUILTIN, OUTPUT);
  testFanDigit();
  // testFanPwm();
  delay(1000);
  Serial.print("Batteria: ");
  Serial.println(readBat(batAnalogPin));
  delay(1000);
  testDallasTemp();
  delay(1000);
  testLightSensor();
  delay(1000);
  testBME();
  delay(1000);
  testConnect();
} // setup

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
} // setup


float readBat(int batPin) {
  unsigned long rawVolt;
  float volt;
  float voltBat;
  int i;

  //letture multiple, fa la media
  rawVolt=0;
  // Serial.println("Letture cumulative batteria: ");
  for (i=0; i<batReadings; i++) {
    rawVolt+=analogRead(batPin);
    // Serial.print("** ");
    // Serial.println(rawVolt);
    delay(readDelay);
  }
  rawVolt=rawVolt/batReadings;
  // il partitore è fatto con 82k + 22k sulla batteria
  // la lettura appare instabile, potrebbe dipendere dalla qualità delle resistenze, 
  // dalla qualità dei cablaggi, dalla qualità del mio tester e dalla qualità del 
  // convertitore A/D del wemos
  // la contemporanea presenza del collegamento USB sembra influenzare negativamente la lettura
  volt=rawVolt*3.07/1024.0; // non modificare, serve per tarare la lettura su 3.3V max e 10bit
  voltBat=volt*(20.6+81.5)/20.6;
  // Serial.print("Tensione: ");
  // Serial.println(volt);
  // Serial.print(" ");
  // Serial.println(voltBat);
  return voltBat;
} // readBat

void testFanPwm () {
  pinMode(fanPin, OUTPUT);
  analogWrite(fanPin,0);
  Serial.println("Fan:  0%");
  delay(4000);
  analogWrite(fanPin, 512);
  Serial.println("Fan: 50%");
  delay(10000);
  analogWrite(fanPin, 870);
  Serial.println("Fan: 85%");
  delay(10000);
  pinMode(fanPin, OUTPUT);
  analogWrite(fanPin,0);
  Serial.println("Fan:  0%");
  delay(5000);
} // testFanPwm

void testFanDigit() {
  int i;
  
  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin,LOW);
  Serial.println("Start LOW");
  for (i=0;i<5;++i) {
    delay(8000);
    Serial.println("Adesso LOW");
    digitalWrite(fanPin, LOW);
    delay(8000);
    Serial.println("Adesso HIGH");
    digitalWrite(fanPin, HIGH);
  } // for
  Serial.println("Adesso LOW");
  digitalWrite(fanPin, LOW);
} // testFanDigit


void testDallasTemp() {
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature sensors(&oneWire);
  float floatDataVal;

  sensors.begin(); // Start up the onewire bus for DS18B20

  // Send the command to get temperatures
  sensors.requestTemperatures();
  floatDataVal=sensors.getTempCByIndex(0);
  Serial.print("Temper. (dallas) = ");
  Serial.print(floatDataVal);
  Serial.println("*C");
} // testDallasTemp


void testLightSensor() {
  int intDataVal;
  Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

  if(!tsl.begin()) {
    /* There was a problem detecting the sensor ... check your connections */
    Serial.print("Could not find a valid TSL2561 sensor, check wiring!");
    return;
  }
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("--------- Light Sensor Info ---------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  

  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME);  /* definito tra i parametri */

  /* Update these values depending on what you've set above! */  
  // Serial.println("------------ Light ------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Integr. Time: ");
  if (TSL2561_INTEGRATIONTIME == TSL2561_INTEGRATIONTIME_13MS) {
    Serial.println("13 ms");
  }
  else {
    if (TSL2561_INTEGRATIONTIME == TSL2561_INTEGRATIONTIME_101MS) {
      Serial.println("101 ms");
    }
    else {
      if (TSL2561_INTEGRATIONTIME == TSL2561_INTEGRATIONTIME_402MS) {
        Serial.println("402 ms");
      }
      else {
        Serial.println("Unknown");
      }
    }
  }
  Serial.println("-------------------------------------------");
  /* Get a new TSL sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  intDataVal=event.light;
  if (intDataVal) {
    Serial.print("Lux              = ");
    Serial.println(intDataVal); 
  }
  else {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }
} // testLightSensor

void testBME() {
  Adafruit_BME280 bme;
  int intDataVal;
  float floatDataVal;

  if (bme.begin(0x76)) {
    // weather/climate scenario fore BME280
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_X1, // temperature
    Adafruit_BME280::SAMPLING_X1, // pressure
    Adafruit_BME280::SAMPLING_X1, // humidity
    Adafruit_BME280::FILTER_OFF);
  }
  else {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    return;
  }

  bme.takeForcedMeasurement();
  floatDataVal=bme.readTemperature();
  Serial.print("Temperature (bme) = ");
  Serial.print(floatDataVal);
  Serial.println("*C");

  floatDataVal=bme.readHumidity();
  Serial.print("Humidity         = ");
  Serial.print(floatDataVal);
  Serial.println("%");

  floatDataVal=bme.readPressure();
  Serial.print("Pressure         = ");
  Serial.print(floatDataVal / 100.0F);
  Serial.println("hPa");

} // testBME

void testConnect() {
   int retr;
   char textAnnounce[100];

   WiFiClient espClient;
   PubSubClient client(espClient);
   
   WiFi.mode(WIFI_STA); // importante, altrimenti va in modo sta+ap e crea un ssid di nome FaryLink
   WiFi.begin(ssid, password);
   Serial.println();
   Serial.println();
   Serial.print("Connecting to WiFi Network ");
   Serial.print(ssid);
   Serial.print(" ");
   retr=0;
   while ((WiFi.status() != WL_CONNECTED) && (retr<maxRetr)) {
      delay(delay2);
      Serial.print(".");
      retr++;
   }
   if (WiFi.status() != WL_CONNECTED) {
      Serial.print(" !!!Failed!!!");
      return;
   }
   Serial.println(" connected"); 
   Serial.print("IP address: ");
   Serial.println(WiFi.localIP());
   
   Serial.println("Waiting few seconds for network to be up ...");
   delay(delay2*3);
   client.setServer(mqttServer, mqttPort);
   Serial.print("Connecting to MQTT server ");
   Serial.print(mqttServer);
   Serial.print(" as clientid '");
   Serial.print(mqttClientId);
   Serial.print("' ");
   retr=0;
   while ((!client.connected()) && (retr<maxRetr)) {
      if(client.connect(mqttClientId, mqttUser, mqttPassword )) {
         Serial.println("connected");
      }
      else {
        Serial.print(".");
        delay(delay2*2);
      }
      retr++;
   }
   if (!client.connected()) {
      Serial.println("!!!Failed to connect to MQTT broker!!!");
      Serial.print("Failed state ");
      Serial.println(client.state());
      return;
   }
   sprintf(textAnnounce,"Hello, here %s", mqttClientId);
   client.publish(mqttTopic,textAnnounce);
   client.subscribe(mqttTopicCmds);
   delay(delay2);

   client.disconnect();
   delay(delay2);
   WiFi.disconnect();
} // testConnect
