/**The MIT License (MIT)

Copyright (c) 2015 by Daniel Eichhorn

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

See more at http://blog.squix.ch
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <ESP8266WiFi.h>
#include <WiFiConnector.h>
WiFiConnector *wifi;

extern "C" {
uint16 readvdd33(void);
  #include "user_interface.h"
}
#include "DHT.h"


/***************************
 * Begin Settings
 **************************/

/*
  WIFI INFO
  DELETE ALL IF YOU WANT TO USE FULL FUNCTION OF SMARTCONFIG
*/
#define WIFI_SSID        ""
#define WIFI_PASSPHARSE  ""
#include "init_wifi.h"

const char* host = "api.thingspeak.com";

const char* THINGSPEAK_API_KEY = "xxxxxxxxxxxxxxx";

// Pin Settings
/*
    Connections for BMP085 i2c
   ===========
   Connect SCL to D1
   Connect SDA to D2
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
 */
#define DHT_PIN D5     // from Data DHT pin2
#define DHT_PWR D6 //Power to DHT pin1
#define VANE_PIN A0 //AnalogRead WindVane
#define VANE_PWR 9 //Power for WindVane
#define ANEMOMETER_PIN 10 // from Anemometer
#define RAIN_GAUGE_PIN D4 //from RainGuage
#define Dust_PIN D7 // from Dust Sensor
#define Dust_PWR D8 //Power to Dust Sensor
//#define UV_PWR D3 // Power for UV sensor

// diameter of anemometer
float diameter = 2.75; //inches from center pin to middle of cup
float mph;
 
// read RPM
int half_revolutions = 0;
int rpm = 0;
unsigned long lastmillis = 0;

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

const boolean IS_METRIC = true;

// Update every 600 seconds = 10 minutes. Min with Thingspeak is ~20 seconds
const int UPDATE_INTERVAL_SECONDS = 600;

/*
 Interface to Shinyei Model PPD42NS Particle Sensor
 Program by Christopher Nafis 
 Written April 2012
 
 http://www.seeedstudio.com/depot/grove-dust-sensor-p-1050.html
 http://www.sca-shinyei.com/pdf/PPD42NS.pdf
 
 JST Pin 1 (Black Wire)  => GND
 JST Pin 3 (Red wire)    => 5VDC
 JST Pin 4 (Yellow wire) => nodemcu Digital Pin D7
 */
//Dust sensor valiables
unsigned long duration, starttime, sampletime_ms = 2000, lowpulseoccupancy = 0;
float ratio = 0, concentrations = 0;
int i = 0;

//BMP085
float barometer=0;
//UV Index
int uv=0;


/***************************
 * End Settings
 **************************/

/***************************
 * Initialize Things
 **************************/

// Initialize the temperature/ humidity sensor
DHT dht(DHT_PIN, DHTTYPE);
//init BMP085
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


#define WIND_FACTOR 2.4
#define TEST_PAUSE 10000 //60000
 
volatile unsigned long anem_count=0;
volatile unsigned long anem_last=0;
volatile unsigned long anem_min=0xffffffff;
 
double getUnitWind()
{
  unsigned long reading=anem_count;
  anem_count=0;
  return (WIND_FACTOR*reading)/(TEST_PAUSE/1000);
}
 
double getGust()
{
  unsigned long reading=anem_min;
  anem_min=0xffffffff;
  double time=reading/1000000.0;
 
  return (1/(reading/1000000.0))*WIND_FACTOR;
}
 
void anemometerClick()
{
  long thisTime=micros()-anem_last;
  anem_last=micros();
  if(thisTime>500)
  {
    anem_count++;
    if(thisTime<anem_min)
    {
      anem_min=thisTime;
    }
  }
}
//static int vaneValues[] PROGMEM={66,84,92,127,184,244,287,406,461,600,631,702,786,827,889,946};
//static int vaneDirections[] PROGMEM={1125,675,900,1575,1350,2025,1800,225,450,2475,2250,3375,0,2925,3150,2700};

int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);
}
int get_wind_direction()
// read the wind direction sensor, return heading in degrees
{
  digitalWrite(VANE_PWR,HIGH);//turn WindVane ON
  delay(200);
  unsigned int adc;

  adc = averageAnalogRead(VANE_PIN); // get the current reading from the sensor

  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.
  Serial.println(adc);
  
  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
  
  digitalWrite(VANE_PWR,LOW); //turn WindVane OFF
}

#define RAIN_FACTOR 0.2794
 
volatile unsigned long rain_count=0;
volatile unsigned long rain_last=0;
 
double getUnitRain()
{
  unsigned long reading=rain_count;
  rain_count=0;
  double unit_rain=reading*RAIN_FACTOR;
 
  return unit_rain;
}
 
void rainGageClick()
{
    long thisTime=micros()-rain_last;
    rain_last=micros();
    if(thisTime>500)
    {
      rain_count++;
    }
}

//dust sensor read
float read_dust(uint8_t pin)
{
  digitalWrite(Dust_PWR,HIGH); //turn DustSensor ON
  duration = pulseIn(Dust_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;
 
  if ((millis()-starttime) >= sampletime_ms)//if the sample time == 30s
  {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=&gt;100
    concentrations = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    Serial.print(lowpulseoccupancy);
    Serial.print(",");
    Serial.print(ratio);
    Serial.print(",");
    Serial.print("concentration (size >1um)= ");
    Serial.println(concentrations);
    Serial.println(" pcs/0.01cf (283ml)");
    Serial.println("\n");
     
    lowpulseoccupancy = 0;
    starttime = millis();
  }
  digitalWrite(Dust_PWR,LOW); //turn DustSensor OFF
  return concentrations;
}

//BMP085 sensor 
float read_bmp085()
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);

  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    Serial.print("Pressure:    ");
    Serial.print(event.pressure);
    Serial.println(" hPa");
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude:    "); 
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,event.pressure)); 
    Serial.println(" m");
    Serial.println("");
  }
  else
  {
    Serial.println("BMP085 error");
  }
  delay(1000);

  return event.pressure;
}

void read_uv() 
{
  digitalWrite(VANE_PWR,LOW); //Turn WindVane OFF to Hook A0 for UV Sensor
  //digitalWrite(UV_PWR,HIGH);
  delay(200);
  int sensorValue;
  sensorValue = analogRead(A0);//connect UV sensors to Analog 0
  
  Serial.println(sensorValue);//print the value to serial  
  if (sensorValue < 10) {uv=0; Serial.println("UV Index: 0"); }
  else if (sensorValue < 46) {uv=1; Serial.println("UV Index: 1"); }
  else if (sensorValue < 65) {uv=2; Serial.println("UV Index: 2"); }
  else if (sensorValue < 83) {uv=3; Serial.println("UV Index: 3"); }
  else if (sensorValue < 103) {uv=4; Serial.println("UV Index: 4"); }
  else if (sensorValue < 124) {uv=5; Serial.println("UV Index: 5"); }
  else if (sensorValue < 142) {uv=6; Serial.println("UV Index: 6"); }
  else if (sensorValue < 162) {uv=7; Serial.println("UV Index: 7"); }
  else if (sensorValue < 180) {uv=8; Serial.println("UV Index: 8"); }
  else if (sensorValue < 200) {uv=9; Serial.println("UV Index: 9"); }
  else if (sensorValue < 221) {uv=10; Serial.println("UV Index: 10");} 
  else if (sensorValue > 240) {uv=11; Serial.println("UV Index: 11"); }
  else {uv=-1; Serial.println("UV sensor error");}
  //digitalWrite(UV_PWR,LOW);
  delay(200);      
}

void init_hardware()
{
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("BEGIN");
  //BMP085
  if (!bmp.begin())
    {
      Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
      while (1);
    }else {
     Serial.println("BMP085 ready.");
    }
    //displaySensorDetails();
  pinMode(DHT_PIN, INPUT);
  pinMode(DHT_PWR,OUTPUT);
  digitalWrite(DHT_PWR,LOW);
  pinMode(Dust_PIN, INPUT);
  pinMode(Dust_PWR,OUTPUT);
  digitalWrite(Dust_PWR,LOW);
  pinMode(D3,INPUT_PULLUP);
  //digitalWrite(UV_PWR,LOW);
  pinMode(ANEMOMETER_PIN,INPUT_PULLUP);
  pinMode(RAIN_GAUGE_PIN,INPUT_PULLUP);
  pinMode(VANE_PIN, INPUT);
  pinMode(VANE_PWR,OUTPUT);
  digitalWrite(VANE_PWR,LOW);
  attachInterrupt(ANEMOMETER_PIN,anemometerClick,FALLING);
  attachInterrupt(RAIN_GAUGE_PIN,rainGageClick,FALLING);
  digitalWrite(DHT_PWR,HIGH);
}

void setup() 
{
  uint8_t SMARTCONFIG_PIN = 0;
  init_hardware();
  init_wifi(SMARTCONFIG_PIN);
  Serial.print("CONNECTING TO ");
  Serial.println(wifi->SSID() + ", " + wifi->psk());
  wifi->on_connecting([&](const void* message)
  {
    char buffer[70];
    sprintf(buffer, "[%d] connecting -> %s ", wifi->counter);
    Serial.print(buffer);
    Serial.println((char*) message);
    delay(500);
  });

  wifi->on_connected([&](const void* message)
  {
    // Print the IP address
    Serial.print("WIFI CONNECTED");
    Serial.println(WiFi.localIP());
  });

  wifi->connect();

  starttime = millis();//get the current time;
}

void loop() {
  starttime = millis();//get the current time;
  wifi->loop();

  //read dust
  //delay(2000);
  float dust = read_dust(Dust_PIN);
  if (dust <= 75) Serial.print("Excellent");
  else if (dust <= 150) Serial.print("Very Good");
  else if (dust <= 300) Serial.print("Good");
  else if (dust <= 1050) Serial.print("FAIR");
  else if (dust <= 3000) Serial.print("POOR");
  else Serial.print("VERY POOR");

  //read bmp085
  barometer = read_bmp085();
  Serial.print("WindSP:");
  Serial.println(getUnitWind()*1.609);
  Serial.print("WindDirec:");
  Serial.println(get_wind_direction());
  Serial.print("Rain:");
  Serial.println(getUnitRain());
  
  //read DHT
  delay(500);
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature(!IS_METRIC);
  Serial.print("Humid:");
  Serial.println(humidity);
  Serial.print("Temp:");
  Serial.println(temperature);
  
  //read Sparkfun unit
  Serial.print("\t KMph=\t");
  Serial.println(getUnitWind()*1.609);
  Serial.print("\t Wind Direction=\t");
  //Serial.println(get_wind_direction());
  //Serial.print(" Deg");
  if((get_wind_direction()<22.5)||(get_wind_direction()>337.5))
    Serial.print("South");
  if((get_wind_direction()>22.5)&&(get_wind_direction()<67.5))
    Serial.print("South-West");
  if((get_wind_direction()>67.5)&&(get_wind_direction()<112.5))
    Serial.print("West");
  if((get_wind_direction()>112.5)&&(get_wind_direction()<157.5))
    Serial.print("North-West");
  if((get_wind_direction()>157.5)&&(get_wind_direction()<202.5))
    Serial.print("North");
  if((get_wind_direction()>202.5)&&(get_wind_direction()<247.5))
    Serial.print("North-East");
  if((get_wind_direction()>247.5)&&(get_wind_direction()<292.5))
    Serial.print("East");
  if((get_wind_direction()>292.5)&&(get_wind_direction()<337.5))
    Serial.print("South-East");
  Serial.print("\t Rain Unit=\t");
  Serial.println(getUnitRain());
  
  //read UV
  read_uv();
  
  //Upload all to THINGSPEAK
  uploadThingsSpeak();
  //Sleep ESP
  ESP.deepSleep(50000000, WAKE_RF_DEFAULT); // Sleep for 50 seconds
}

 void uploadThingsSpeak(){
  int loopCount = 0;
    Serial.print("connecting to ");
    Serial.println(host);
    
    // Use WiFiClient class to create TCP connections
    WiFiClient client;
    const int httpPort = 80;
    if (!client.connect(host, httpPort)) {
      Serial.println("connection failed");
      return;
    }
    // read values from the sensor
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature(!IS_METRIC);
 
    // We now create a URI for the request
    String url = "/update?api_key=";
    url += THINGSPEAK_API_KEY;
    url += "&field1=";
    url += String(temperature);
    url += "&field2=";
    url += String(humidity);
    url += "&field3=";
    url += String(getUnitWind()*1.609);
    url += "&field4=";
    url += String(get_wind_direction());
    url += "&field5=";
    url += concentrations;
    url += "&field6=";
    url += barometer;
    url += "&field7=";
    url += String(getUnitRain());
    url += "&field8=";
    url += uv;
    
    Serial.print("Requesting URL: ");
    Serial.println(url);
    
    // This will send the request to the server
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" + 
                 "Connection: close\r\n\r\n");
    delay(10);
    while(!client.available()){
      delay(100);
      Serial.print(".");
      loopCount++;
      Serial.print(loopCount);
      // if nothing received for 10 seconds, timeout
      if(loopCount > 1000) {
      client.stop();
      Serial.println("TCP send Timeout");
      break;
      }
    }
    // Read all the lines of the reply from server and print them to Serial
    while(client.available()){
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
    
    Serial.println();
    Serial.println("closing connection");

 }

