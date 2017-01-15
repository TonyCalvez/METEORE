/*
DHT22 - Pin6
GPS - Rx->Pin4 et Tx->pPin3
Centrale Inertielle : SDA->A5 et SCL->A6
TEMT 6000: A2
UV: A1
rf69

*/


#include "DHT.h"
#define DHTPIN 6
#define DHTTYPE DHT22
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
DHT dht(DHTPIN, DHTTYPE);

/* GPS */
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(3, 4);

Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

boolean usingInterrupt = false;
void useInterrupt(boolean);

/* Module Emission */
#include <RFM69.h> 
#include <SPI.h>

#define NETWORKID     100  
#define NODEID        2   
#define RECEIVER      1 
 

#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" 
#define IS_RFM69HCW   true
 
//*********************************************************************************************
#define SERIAL_BAUD   115200
 
#define RFM69_CS      10
#define RFM69_IRQ     2
#define RFM69_IRQN    0  
#define RFM69_RST     9
 
#define LED           13  
 
char radiopacket[100]; //paquet de 100 char pour emission
int16_t packetnum = 0;  
 
RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

/* fin module emission*/

/* VARIABLE GLOBAL */
  float h = 0;
  float t = 0;
  float hic = 0;
  float lux = 0;
  float UV = 0;
  float temperaturebmp;
  float altitudeBMP;
  float p;
  String message;
  String dataRFM69HCW="0";
  String temperature;
  String humidite;
  String temperatureressentie;
  String indexUV;
  String luminosite;
  String pression;
  String gpslongitude;
  String gpslatitude;
  String gpsaltitude;
  String gpsvitesse;

void setup() {
  Serial.begin(115200);
  dht.begin();
  bmp.begin();
  /* GPS*/ 
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
  mySerial.println(PMTK_Q_RELEASE);
  /* Fin Gps */
  /* module emission*/
  Serial.begin(SERIAL_BAUD);
 
  Serial.println("Arduino RFM69HCW Transmitter");
  
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);
 
  // Initialize radio
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  if (IS_RFM69HCW) {
    radio.setHighPower();    // Only for RFM69HCW & HW!
  }
  radio.setPowerLevel(31); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  radio.encrypt(ENCRYPTKEY);
  
  pinMode(LED, OUTPUT);
  Serial.print("\nTransmitting at ");
  Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(" MHz");
  /* fin module emission*/
}

void loop(void){
  importdht22();
  importUV();
  importluminosite();
  importBMP();
  importGPS();
  Serial.print("\n");
  message = dataRFM69HCW + ";" + temperature + ";" + humidite + ";" + temperatureressentie + ";" + indexUV + ";" + luminosite + ";" + pression + ";";
  Serial.print(message);
  message.toCharArray(radiopacket,100);
  moduleemission();
  message = dataRFM69HCW + ";"+ gpslongitude + ";" + gpslatitude + ";" + gpsaltitude + ";" + gpsvitesse + "\n";
  Serial.print(message);
  message.toCharArray(radiopacket,100);
  moduleemission();
  delay (1000);
}

void importdht22() {

  h = dht.readHumidity();
  t = dht.readTemperature();
  float hic = dht.computeHeatIndex(t, h, false);
  humidite = String(h,2);
  Serial.print(humidite);
  temperature = String(t, 2);
  Serial.print(temperature);
  temperatureressentie = String(hic,2);
  Serial.print(temperatureressentie);

}

void importUV() {
  indexUV = String((analogRead(A1) * 5.0 / 1023.0), 2);
  Serial.print(indexUV);
}

void importluminosite() {
  luminosite = String(analogRead(A2) * 0.9765625, 2);
  Serial.print(luminosite);
}

void importBMP() {
  sensors_event_t event;

  bmp.getEvent(&event);

  if (event.pressure)
  {
    p=event.pressure;
    pression = String(p,2);
    Serial.print(pression);

    bmp.getTemperature(&temperaturebmp);


    altitudeBMP = SENSORS_PRESSURE_SEALEVELHPA;
  }
  else
  {
    Serial.println("BMP - Erreur");
  }
  delay(1000);
}


/* GPS */

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c; 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void importGPS()
{
  if (! usingInterrupt) {
    char c = GPS.read();
    if (GPSECHO)
      if (c);
  }
  
  if (GPS.newNMEAreceived()) {
  
    if (!GPS.parse(GPS.lastNMEA()))  
      return; 
  }

  if (timer > millis())  timer = millis();

  if (millis() - timer > 2000) { 
    timer = millis();
      gpslongitude = String(GPS.longitudeDegrees, 4);
      Serial.print(gpslongitude);
      gpslatitude = String(GPS.latitudeDegrees, 4);
      Serial.print(gpslatitude);
      gpsvitesse = String(GPS.speed, DEC);
      Serial.print(gpsvitesse);
      gpsaltitude = String(GPS.altitude, DEC);
      Serial.print(gpsaltitude);
    
  }
}

void moduleemission() {
  Serial.print("Sending "); Serial.println(radiopacket);
  if (radio.sendWithRetry(RECEIVER, radiopacket, strlen(radiopacket))) { //target node Id, message as string or byte array, message length
    Serial.println("OK");
    Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
  }
  radio.receiveDone(); //put radio in RX mode
  Serial.flush(); //make sure all serial data is clocked out before sleeping the MCU
}
 
void Blink(byte PIN, byte DELAY_MS, byte loops)
{
  for (byte i=0; i<loops; i++)
  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}