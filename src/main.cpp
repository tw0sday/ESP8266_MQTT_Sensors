// Program for ESP8266 MQTT + Sensors                KLH V0.92/8.1.2023
// serial connection (via USB Bridge), setup WIFI
// reading Chip-Data, MAC-Adress
// sending (coloured) output via serial Port (115200 Baud)
// scanning for WIFI-Networks, Ping, starting NTP client (UDP), 
// scanning OneWire BUS for Dallas Temperature sensor (DS18B20)
// implementing simple MQTT-Client

#include <Arduino.h>
#include "Wire.h"
#include <ESP8266WiFi.h>
#include <ESP8266Ping.h>
#include <PubSubClient.h>       // MQTT
#include <ArduinoJson.h>        // https://github.com/bblanchon/ArduinoJson.git
#include <NTPClient.h>          // https://github.com/taranais/NTPClient
#include <WiFiUdp.h>
#include <OneWire.h>            // OneWire bus for Sensors
#include <DallasTemperature.h>  // read Dallas Temp. Sensors

#include "hw_settings.h"        // WiFi settings see hw_settings.template

//const char* ssid     = "xxxxxxx";     hw_settings.h
//const char* password = "yyyyyyy";

#define Version_SW "ESP8266-AZ-D1-0.92"
#define Version_HW "ESP8266-AZ-D1-BR-0.9"

const int LED = 5;              // IO5 = D1 (AZ)!!!
const int oneWireBus = 4;       // IO4 for OneWire devices           
const int Ain = A0;             // 
int aVal = 0;                   // analog read Value 

const IPAddress remote_ip(10, 4, 0, 106);
String client_Id = "ESP8266-";   
String client_MAC = "";

OneWire OW_ds(oneWireBus);          // OneWire Datastructure
DallasTemperature ow_sensors(&OW_ds);  // Dallas Sensor Data
char *str_temp0 = "17";       // = ow_sensors.getTempCByIndex(0);

// non blocking Timer - millis() function returns unsigned long
unsigned long start_time;       // settings for non blocking timer
unsigned long timer1 = 30000;   // time in ms
unsigned long current_time;     // millis() 

// MQTT Broker

// const char *mqtt_broker = "10.4.0.106";   // Local Broker on Pi2
const char  *sensor_topic = "sensors/esp32/esp8266-01";
const char *room_topic = "home/DG/elab/light_01";
const char *mqtt_username = "esp32";
const char *mqtt_password = "esp32";
const char *mqtt_mess = "";
const int mqtt_port = 1883;

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
String NTP_time = "";

// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;

WiFiClient espClient;  
PubSubClient client(espClient); 

// JSON Document
//StaticJsonDocument<300> odoc;  // 200 Byte RAM allocation
DynamicJsonDocument idoc(400);
DynamicJsonDocument odoc(400);

// ANSI ESCAPE Sequences for coloured output
#define ESC_RED "\e[1;31m"      // red 
#define ESC_GRN "\e[1;32m"      // green
#define ESC_YEL "\e[1;33m"      // yellow
#define ESC_BLU "\e[1;34m"      // blue
#define ESC_MAG "\e[1;35m"      // magenta
#define ESC_CYA "\e[1;36m"      // cyan
#define ESC_WHT "\e[1;37m"      // white
#define ESC_RES "\e[1;0m"       // reset

////////////////////////// MQTT-callback ///////////////////////////////

void callback(char *topic, unsigned char *payload, unsigned int length) 
{
  StaticJsonDocument<400> odoc; 
  String messageTemp;
  String output;
  if (timeClient.isTimeSet()) NTP_time = timeClient.getFormattedTime();
  // digitalWrite(LED, HIGH);    // LED blink for received MQTT
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (unsigned int i = 0; i < length; i++) {
      Serial.print((char) payload[i]);
      messageTemp += (char)payload[i];
  }
  // if ((String(topic) == "home/DG/elab/light_01"))
  if ((String(topic) == room_topic))
    {
    if(messageTemp == "status")
      { 
        // ow_temp0 = ow_sensors.getTempCByIndex(0);
        odoc["event"] = "STATUS";
        odoc["room"] = room_topic;
        odoc["sensor"] = sensor_topic;
        odoc["sensor_MAC"] = client_MAC.c_str();
        odoc["time"] = NTP_time;
        odoc["SSID"] = WiFi.SSID();
        odoc["RSSI"] = WiFi.RSSI();
        odoc["A0in"] = analogRead(Ain);
        // odoc["temp[0]"] = str_temp0;      
        odoc["LED"] = digitalRead(LED);
    
        serializeJsonPretty(odoc, output);
        client.publish("sensors", output.c_str());
      } // if "status"
    
      else if(messageTemp == "set_LED=1") 
      {
        digitalWrite(LED, HIGH);
        if(digitalRead(LED))
          {
          client.publish("sensors","LED = HIGH");
          Serial.print(ESC_RED);
          Serial.println("  LED = HIGH");
          Serial.print(ESC_RES);
          }
      }
      else if(messageTemp == "set_LED=0")
      { 
        digitalWrite(LED, LOW);
        if(! digitalRead(LED))
          {
          client.publish("sensors","LED = LOW");
          Serial.print(ESC_GRN);
          Serial.println("  LED = LOW");
          Serial.print(ESC_RES);
          }
      }
    } // if sensors/esp32
  Serial.println();
  // Serial.println(messageTemp);
  Serial.println("---------------------------------------");
  delay(10);     
} // callback

//////////////////////////   S E T U P   ////////////////////////////
void setup() {
  
  pinMode(LED,OUTPUT);                        //  Testing LED
  digitalWrite(LED, HIGH);
  Serial.begin(115200);     // Serial connection for debugging
  Serial.println();

// Searching for One-Wire Sensors connected at GPIO "oneWireBUS" 
// Setup a oneWire instance to communicate with any OneWire devices
    // OneWire OW_ds(oneWireBus);
    Serial.print(ESC_MAG);
    Serial.print("\nScanning for OneWire devices on GPIO ");
    Serial.println(oneWireBus);

    byte addr[8]; byte i; int k=0;   // ROM address , index , number of devices
    while (OW_ds.search(addr)) 
    {
    Serial.print("ID =");           // e.g. ROM = 28 ED EE 8E 54 22 8 BE
    for (i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    } // for
    k++;                              // number of devices
  Serial.println();  
  } // while
  OW_ds.reset_search();
  Serial.print(ESC_RES);

// Pass oneWire reference to Dallas Temperature sensor (DS18B20)
  // DallasTemperature sensors(&OW_ds);  // Dallas Sensor Data
  for (i = 0; i < k; i++)             // read sensors
  {
    ow_sensors.begin();
    ow_sensors.requestTemperatures(); 
    float temp0 = ow_sensors.getTempCByIndex(i);  // first sensor
    // sprintf(str_temp0, "%f", temp0);
    if (temp0 != -127.00)           // read error (R pullup ?)
    {
      Serial.print("Temp. Sensor(");
      Serial.print(i); Serial.print(")    : ");
      Serial.print(temp0);
      Serial.println(" C");
    }
    else 
    { 
      Serial.print(ESC_RED);
      Serial.print("error reading Temp. Sensor(");
      Serial.print(i); Serial.print(") at GPIO ");
      Serial.println(oneWireBus);
      Serial.print(ESC_RES);
    }
  } // for
//////////////////////  Scanning WiFi-Network ///////////////////////
   // Set WiFi to station mode (and disconnect from an AP)
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    Serial.print(ESC_CYA);
    Serial.print("\nWIFI Staion Mode. ");
    Serial.print("Scanning for WiFi Networks ... ");
    // WiFi.scanNetworks will return the number of networks found.
    int n = WiFi.scanNetworks();
    Serial.println("done");
    if (n == 0) 
      {
        Serial.print(ESC_RED);
        Serial.println("no networks found");
        Serial.print(ESC_RES);
      } 
      else {
        Serial.print(ESC_CYA);
        Serial.print(n);
        Serial.println(" networks found");
        Serial.println("Nr | SSID                       | RSSI | CH | Encryption");
        for (int i = 0; i < n; ++i) 
        {
            // Print SSID and RSSI for each network found
            digitalWrite(LED, HIGH);           // LED blink for each network found
            Serial.printf("%2d",i + 1);
            Serial.print(" | ");
            Serial.printf("%-26.26s", WiFi.SSID(i).c_str());
            Serial.print(" | ");
            Serial.printf("%4d", WiFi.RSSI(i));
            Serial.print(" | ");
            Serial.printf("%2d", WiFi.channel(i));
            Serial.print(" | ");
            switch (WiFi.encryptionType(i))
            {
            case AUTH_OPEN: Serial.print("open"); break;
            case AUTH_WEP:   Serial.print("WEP"); break;
            case AUTH_WPA_PSK: Serial.print("WPA"); break;
            case AUTH_WPA2_PSK: Serial.print("WPA2"); break;
            case AUTH_WPA_WPA2_PSK: Serial.print("WPA+WPA2"); break;
            default:
                Serial.print("unknown");
            }
            Serial.println();
            delay(100);
            digitalWrite(LED, LOW);
            delay(200);
        }
        Serial.print(ESC_RES);
    }
    // Delete the scan result to free memory for code below.
    WiFi.scanDelete();

 ///////////////////// Connecting to WiFi ///////////////////////////
    Serial.print(ESC_YEL);
    Serial.print("\nConnecting to:        ");
    Serial.println(wifi_ssid); 
    // WiFi.setHostname(hostname.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
//   esp_task_wdt_delete(NULL);            // delete WDT when connected!
    Serial.print("\nRRSI:                 ");
    Serial.println(WiFi.RSSI()); 
  
   //////////////////////  setup NTP Client   ////////////////////////
    // Initialize a NTP Client to get time
    Serial.print(ESC_YEL);
    Serial.print("\nStarting (NTP) Network Time Protocoll-Client ");
    timeClient.begin();    
    timeClient.setTimeOffset(3600);      // GMT +1 = 3600 = CET (Vienna)
    timeClient.update();
    if(timeClient.isTimeSet()) 
      {
        Serial.println(timeClient.getFormattedTime());
      }
    else
      {
        Serial.print(ESC_RED);
        Serial.println("NTP failed!");
      }
    Serial.print(ESC_RES);
    delay(10);

//////////////////////  setup MQTT Client   ////////////////////////

  if(Ping.ping(mqtt_broker, 3))
  {
    Serial.print(ESC_GRN);
    Serial.print("Ping to MQTT-Broker ");
    Serial.print(mqtt_broker);
    Serial.println(" successful!");
    Serial.print(ESC_RES);
  }
  else
  {
    Serial.print(ESC_RED);
    Serial.print("Ping MQTT-Broker ");
    Serial.print(mqtt_broker);
    Serial.println(" failed!");
    Serial.print(ESC_RES);
    // return;
  }
  ////////////////  connecting to a MQTT Broker  /////////////////////
 
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  while (! client.connected()) {
      // clientId = "ESP32-";
      client_MAC += String(WiFi.macAddress());
      if (client.connect(client_MAC.c_str(), mqtt_username, mqtt_password))
      {
        Serial.print(ESC_MAG);
        Serial.printf("MQTT-client: %s connected\n", client_MAC.c_str());
        Serial.print(ESC_RES);
      } 
      else 
      {
        Serial.print(ESC_RED);
        Serial.print("failed with state ");
        Serial.print(client.state());
        Serial.print(ESC_RES);
        delay(2000);
      }
  }
  client.subscribe("sensors/#");        // default topic
  Serial.println("MQTT-Topic:'sensors/#' subscribed successfully!");
  if(! client.subscribe(sensor_topic))
    { 
      Serial.print(ESC_RED);
      Serial.print(sensor_topic);
      Serial.println(" subscribe failed!");
      Serial.print(ESC_RES);
    } 
    else
    {
      Serial.print(ESC_GRN);
      Serial.print(sensor_topic);
      Serial.println(" subscribed successfully!");
      Serial.print(ESC_RES);      
    }
  if(! client.subscribe(room_topic))
    { 
      Serial.print(ESC_RED);
      Serial.print(room_topic);
      Serial.println(" subscribe failed!");
      Serial.print(ESC_RES);
    } 
    else
    {
      Serial.print(ESC_GRN);
      Serial.print(room_topic);
      Serial.println(" subscribed successfully!");
      Serial.print(ESC_RES);      
    }

////////////////  MQTT Broker BOOT -Message /////////////////////

  StaticJsonDocument<400> odoc;  
  String output;
        // str_temp0 = ow_sensors.getTempCByIndex(0);
        odoc["event"] = "BOOT";
        odoc["room"] = room_topic;
        odoc["sensor"] = sensor_topic;
        odoc["sensor_MAC"] = client_MAC.c_str();
        odoc["time"] = NTP_time;
        odoc["SSID"] = WiFi.SSID();
        odoc["RSSI"] = WiFi.RSSI();
        odoc["A0in"] = analogRead(Ain);
        // odoc["temp[0]"] = str_temp0;      
        odoc["LED"] = digitalRead(LED);

  serializeJsonPretty(odoc, Serial);   // generate & print JSON to Serial
  Serial.println("");
  
  serializeJsonPretty(odoc, output);
  
  if (! client.publish("sensors/esp32",output.c_str()))
    { 
      Serial.print(ESC_RED);
      Serial.println("publish failed!");
      Serial.print(ESC_RES);
    } 
  client.publish("sensors", output.c_str()); // publish to sensor_topic
odoc.clear();

// Start Timer
  current_time = millis();
	start_time = current_time; 
} // End Setup

//////////////////////////  M A I N - L O O P ////////////////////////                      
void loop() 
{
  client.loop();
  if (millis() - start_time >= timer1) //repeating after timer (ms)
  {
    String output;
    StaticJsonDocument<400> odoc;
    timeClient.update();
    if (timeClient.isTimeSet()) NTP_time = timeClient.getFormattedTime();
    else
    {
    Serial.print(ESC_RED);
    Serial.println ("NTP failed!");
    Serial.print(ESC_RES);
    }
      //str_temp0 = ow_sensors.getTempCByIndex(0);
        odoc["event"] = "LOOP";
        odoc["room"] = room_topic;
        odoc["sensor"] = sensor_topic;
        odoc["sensor_MAC"] = client_MAC.c_str();
        odoc["time"] = NTP_time;
        odoc["SSID"] = WiFi.SSID();
        odoc["RSSI"] = WiFi.RSSI();
        odoc["A0in"] = analogRead(Ain);
        // odoc["temp[0]"] = str_temp0;      
        odoc["LED"] = digitalRead(LED);

    serializeJsonPretty(odoc, output);
    client.publish(room_topic, output.c_str());
    start_time = millis();            // reset the timer
    odoc.clear();
  }
}  // end loop
//////////////////////////////  E N D  ///////////////////////////////
