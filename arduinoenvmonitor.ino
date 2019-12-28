// Attenzione che i pin SCL e SDA cambiano a seconda della scheda utilizzata Arduino UNO/Wemos D1R1/Wemos D1R2
//
// Attenzione BME280 e TSL2561 alimentati a 3.3V
// Attenzione Wemos D1 ha max V su A0 pari a 3.3 / 10bit
//
/*
 * I sensori non possono essere tenuti spenti per evitare fenomeni 
 * di auto riscaldamento in quanto, almeno con il BME, potrebbero
 * danneggiarsi se arrivano segnali sul BUS
 * 
 */
#include <OneWire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_BME280.h>
#include <DallasTemperature.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>

#include "securities.h"
/*
 ******* securities.h ********
 * 
#define ssid "myssid"
#define password "mypass"
#define mqttServer "mymqttserver"
#define mqttPort myport
#define mqttUser "myusername"
#define mqttPassword "mypassword"
#define otahost "myotahost"
#define otaport myotaport
#define otapath "myotapath"
*/

#define maxRetr 20
#define mqttClientId "smaldinoHomeTerrace"
#define mqttTopic "announcement/clientid"
/*
 * ******************************************************
 * cambiare wemosd1 aggiungendo il chipid ESP.getChipId *
 * ******************************************************
*/
#define mqttTopicOut    "it/smaldino/home/terrace/wemosd1/out"
#define mqttTopicCmds   "it/smaldino/home/terrace/wemosd1/cmds"
// ######## comandi ################
#define FORCEREAD "READNOW"
// #################################

// ######## stati ################
#define statusOK 0
#define statusErrNoNet 1
#define statusErrNoMq 2
#define statusErrTslLux 3
#define statusErrBmeThp 4
#define statusErrGen 5
// ###############################

// ######## tsl   ################
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
// #define TSL2561_INTEGRATIONTIME TSL2561_INTEGRATIONTIME_13MS      /* fast but low resolution */
// #define TSL2561_INTEGRATIONTIME TSL2561_INTEGRATIONTIME_101MS  /* medium resolution and speed   */
#define TSL2561_INTEGRATIONTIME TSL2561_INTEGRATIONTIME_402MS  /* 16-bit data but slowest conversions */
// ###############################

#define checkMqttClient 80 // in ms intervallo per il controllo dell'arrivo di messaggi
#define leaseDuration 5 // lesase del DHCP in hour 

#define readingSensorsInterval 5 // in minuti, ogni quanti minuti legge i sensori
#define washTime 60              // in sec, tempo di lavaggio dell'aria interna
#define washTimePreOff 5         // in sec, tempo di pre-spegnimento delle ventole

#define ledSigPin LED_BUILTIN
#define myOFF HIGH // LED_BUILTIN funziona al contrario
#define myON  LOW  // LED_BUILTIN funziona al contrario

#define batAnalogPin A0
#define batReadings 50 // numero ripetizioni della lettura fra cui fare la media
#define readDelay 10   // in ms, attesa fra una lettura e l'altra
#define fanPin D13
#define ONE_WIRE_BUS D8

#define NumberOfErrorSignal 5
#define delay1 300
#define delay2 1000
#define delay3 10

#define SEALEVELPRESSURE_HPA (1013.25)

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   I2C Address
   ===========
   The address will be different depending on whether you leave
   the ADDR pin floating (addr 0x39), or tie it to ground or vcc. 
   The default addess is 0x39, which assumes the ADDR pin is floating
   (not connected to anything).  If you set the ADDR pin high
   or low, use TSL2561_ADDR_HIGH (0x49) or TSL2561_ADDR_LOW
   (0x29) respectively.
*/
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
Adafruit_BME280 bme;
// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(ONE_WIRE_BUS);
// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);

int status; // stato del sistema

#define heartBeatTime 6 // in sec, ogni quanto accende la luce di heartbeat
                        // l'accensione dura circa 1/5 della pausa
boolean heartBeatStatus=false;
unsigned long loopHeartBeat; // cicli per il battito del cuore
unsigned long loopCount4HeartBeat=0;
unsigned long loopReadingSensors; // cicli prima di leggere i sensori
unsigned long loopCount4ReadingSensors=0;
unsigned long loopWash;           // cicli per il lavaggio dell'aria interna
unsigned long loopWashPreOff; // cicli di spegnimento anticipato delle ventole
unsigned long loopLeaseDuration;
unsigned long loopCount4LeaseDuration=0; 
unsigned long loop4client; // ogni quanti cicli controlla l'arrivo di messaggi
unsigned long loopCount4client=0; // contatore per il controllo dell'arrivo dei messaggi
                           // usato anche per la segnazione in condizione di errore
boolean forceReadSensors=false;
boolean readingMessageSent=false;

WiFiClient espClient;
PubSubClient client(espClient);


int myconnect() {
   int retr;
   char textAnnounce[100];

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
      return statusErrNoNet;
   }
   Serial.println(" connected"); 
   Serial.print("IP address: ");
   Serial.println(WiFi.localIP());
   
   Serial.println("Waiting few seconds for network to be up ...");
   delay(delay2*3);
   client.setCallback(callback);
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
      return statusErrNoMq;
   }
   sprintf(textAnnounce,"Hello, here %s", mqttClientId);
   client.publish(mqttTopic,textAnnounce);
   client.subscribe(mqttTopicCmds);
   return statusOK;
} // myconnect


void mydisconnect () {
   client.disconnect();
   delay(delay2);
   WiFi.disconnect();
} // mydisconnect


void setup() {
   
   pinMode(ledSigPin, OUTPUT);
   digitalWrite(ledSigPin, myON);

   pinMode(fanPin, OUTPUT);
   digitalWrite(fanPin, LOW);
   
   Serial.begin(19200);
   
   status=statusErrGen;
   
   loopCount4LeaseDuration=0;
   loopLeaseDuration=leaseDuration*3600*1000/delay3;
   
   loopCount4client=0;
   loop4client=checkMqttClient/delay3;
   
   loopCount4ReadingSensors=0;
   loopReadingSensors=readingSensorsInterval*60*990 /delay3; // 1% di compensazione tempo

   loopHeartBeat=(heartBeatTime * 1000) /delay3;
   loopCount4HeartBeat=0;
   heartBeatStatus=false;

   loopWash=(washTime - washTimePreOff) * 1000 / delay3;
   loopWashPreOff=washTimePreOff * 1000 / delay3;
   
   forceReadSensors=false;
   
   readingMessageSent=false;
    
   status=myconnect();

   if (status <= statusOK) {
      status=initSensors();
   } // if (status <= statusOK)
   
} // setup


void analyzePayload(char *payload) {
  // client.publish(mqttTopic,"Light on");
  if (strstr(payload, FORCEREAD)) {
     // arrivata la richiesta di fare una lettura
     forceReadSensors=true;
     Serial.println("Richiesta di lettura");
  }
} // analyzePayload


void callback(char* topic, byte* payload, unsigned int length) {
   Serial.print("Message received in topic: ");
   Serial.println(topic);
   Serial.print("Message: ");
   for(int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
   }
   Serial.println();
   
   analyzePayload((char *) payload);
   delay(delay1);
   // readBat(A0);
} // callback


void ledSignal(int lamps) {
  int i;

  // spegne il led
  digitalWrite(ledSigPin, myOFF);
  delay(delay2);
  for(i=0;i<lamps;i++) {
    digitalWrite(ledSigPin, myON);
    delay(delay1);
    digitalWrite(ledSigPin, myOFF);
    delay(delay1);
  }
} // ledSignal


float readBat(int batPin) {
  unsigned long rawVolt;
  float volt;
  float voltBat;
  int i;

  //letture multiple, fa la media
  rawVolt=0;
  for (i=0; i<batReadings; i++) {
    rawVolt+=analogRead(batPin);
    delay(readDelay);
  }
  rawVolt=rawVolt/batReadings;
  // il partitore è fatto con 82k + 22k sulla batteria
  // la lettura appare instabile, potrebbe dipendere dalla qualità delle resistenze, 
  // dalla qualità dei cablaggi, dalla qualità del mio tester e dalla qualità del 
  // convertitore A/D del wemos
  // la contemporanea presenza del collegamento USB sembra influenzare negativamente la lettura
  volt=rawVolt*2.9/1024.0; // non modificare, serve per tarare la lettura su 3.3V max e 10bit
  voltBat=volt*(20.6+81.5)/20.6;
  // Serial.print("Tensione: ");
  // Serial.println(volt);
  // Serial.print(" ");
  // Serial.println(voltBat);
  

  return voltBat;
} // readBat


void loop () {    
   int swVal;

   loopCount4client++;
   if (status <= statusOK) {
      if (loopCount4client > loop4client) { 
        loopCount4client=0;
        // client.loop serve a inviare e recuperare i messaggi 
        // se il tempo fra 2 loop è troppo lungo, si perdono i
        // i messaggi, soprattutto quelli in ingresso
        client.loop();
      }
      
      loopCount4LeaseDuration++;
      if (loopCount4LeaseDuration > loopLeaseDuration) {
        loopCount4LeaseDuration=0;
        mydisconnect();
        ESP.restart();
      }
      
      loopCount4HeartBeat++;
      if (((loopCount4HeartBeat>(loopHeartBeat/5)) && (heartBeatStatus)) ||
          ((loopCount4HeartBeat>loopHeartBeat) && (!heartBeatStatus)))
      {
        loopCount4HeartBeat=0;
        if (heartBeatStatus) {
          digitalWrite(ledSigPin, myOFF);
          heartBeatStatus=false;
        }
        else {
          digitalWrite(ledSigPin, myON);
          heartBeatStatus=true;
        }
      }

      manageSensorReading();
      
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("No Wifi!!");
        mydisconnect();
        ESP.restart();
      }
      if (!client.connected()) {
        Serial.println("No MQTT broker!!!");
        mydisconnect();
        ESP.restart();
      }
      
      delay(delay3);
   }
   else {
      // condizione di errore
      ledSignal(status+2);
      delay(delay2*5);
      Serial.print("Condizione di errore, stato: ");
      Serial.println(status);
      // Serial.print(loopCount4client);
      // Serial.print(" ");
      // Serial.println(NumberOfErrorSignal);
      if (loopCount4client > NumberOfErrorSignal) {
        mydisconnect();
        ESP.restart();
      }
   }
} // loop

void readSensors() {
  char message[100];
  char topic[200];
  float floatDataVal;
  int  intDataVal;
  
  // mqttTopicOut
  
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
  strcpy(topic, mqttTopicOut);
  strcat(topic,"/light");
  sprintf(message,"%5d",intDataVal);
  // Serial.println(topic);
  // Serial.println(message);
  client.publish(topic,message);

  bme.takeForcedMeasurement();
  floatDataVal=bme.readTemperature();
  Serial.print("Temperature (bme) = ");
  Serial.print(floatDataVal);
  Serial.println("*C");
  strcpy(topic, mqttTopicOut);
  strcat(topic,"/tempbme");
  sprintf(message,"%3.1f",floatDataVal);
  client.publish(topic,message);

  floatDataVal=bme.readHumidity();
  Serial.print("Humidity         = ");
  Serial.print(floatDataVal);
  Serial.println("%");
  strcpy(topic, mqttTopicOut);
  strcat(topic,"/humbme");
  sprintf(message,"%3.1f",floatDataVal);
  client.publish(topic,message);

  floatDataVal=bme.readPressure();
  Serial.print("Pressure         = ");
  Serial.print(floatDataVal / 100.0F);
  Serial.println("hPa");
  strcpy(topic, mqttTopicOut);
  strcat(topic,"/pressbme");
  sprintf(message,"%3.1f",floatDataVal);
  client.publish(topic,message);

  Serial.print("Approx. Altitude = ");
  floatDataVal=bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print(floatDataVal);
  Serial.println("m");
  strcpy(topic, mqttTopicOut);
  strcat(topic,"/altitude");
  sprintf(message,"%4.0f",floatDataVal);
  client.publish(topic,message);

  floatDataVal=readBat(batAnalogPin);
  Serial.print("Battery          = ");
  Serial.println(floatDataVal);
  strcpy(topic, mqttTopicOut);
  strcat(topic,"/bat");
  sprintf(message,"%3.1f",floatDataVal);
  client.publish(topic,message);

  // Send the command to get temperatures
  sensors.requestTemperatures();
  floatDataVal=sensors.getTempCByIndex(0);
  Serial.print("Temper. (dallas) = ");
  Serial.print(floatDataVal);
  Serial.println("*C");
  strcpy(topic, mqttTopicOut);
  strcat(topic,"/tempdallas");
  sprintf(message,"%3.1f",floatDataVal);
  client.publish(topic,message);

  Serial.println();
} // readSensors


int initSensors() {
   int retCode;

   sensors.begin(); // Start up the onewire bus for DS18B20
   retCode=statusOK;
   /* Initialize the sensors */
   if(!tsl.begin()) {
      /* There was a problem detecting the sensor ... check your connections */
      Serial.print("Could not find a valid TSL2561 sensor, check wiring!");
      retCode = statusErrTslLux;
   }
   else {
      displaySensorDetails();
      delay(delay1);
      configureLightSensor();
      delay(delay1);
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
         retCode = statusErrBmeThp;
      }
   }
   
   return retCode;
} // initSensors


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails() {
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("--------- Light Sensor Info ---------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  // Serial.println("-------------------------------------------------");
  // Serial.println("");
} // displaySensorDetails


/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureLightSensor() {
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
} // configureLightSensor

void manageSensorReading() {
  // DA RIVEDERE HO IL SOSPETTO CHE IN ACCENSIONE VENTOLE CI ENTRI MOLTE VOLTE
  char topic[200];
  loopCount4ReadingSensors++;
  if (loopCount4ReadingSensors > (loopReadingSensors - loopWash - loopWashPreOff)) {
    if (loopCount4ReadingSensors < (loopReadingSensors - loopWashPreOff)) {
      // accensione ventole
      // Serial.println("Accensione ventole");
      digitalWrite(fanPin, HIGH);
      // avvisa che è partita la sequenza di lettura
      if (!readingMessageSent) {
         strcpy(topic, mqttTopicOut);
         strcat(topic,"/reading");
         client.publish(topic,"READINGON");
         readingMessageSent=true;
      }
    }
    else {
      // spegnimento ventole
      // Serial.println("Spegnimento ventole");
      digitalWrite(fanPin, LOW);
      if (loopCount4ReadingSensors > loopReadingSensors) {
        loopCount4ReadingSensors=0;
        // esegue la lettura dei sensori
        readSensors();
        // avvisa che è conclusa la sequenza di lettura
        strcpy(topic, mqttTopicOut);
        strcat(topic,"/reading");
        client.publish(topic,"READINGOFF");
        readingMessageSent=false;
      }
    }
  }
  else {
     if (forceReadSensors) {
        // è arrivata una richiesta di lettura e quindi si forza il contatore
        // per evaderla
        loopCount4ReadingSensors = loopReadingSensors - loopWash - loopWashPreOff;
     }
  }
  forceReadSensors=false; // disattiva la richiesta
} // manageSensorReading
