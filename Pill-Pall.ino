# include <Arduino.h>
# include <WiFi.h>
# include <PubSubClient.h>
# include <esp_wifi.h>
# include <Stepper.h>
# include "thermistor.h"

//LED pin 
# define RED_PIN 25
# define GREEN_PIN 26

// Distance Sensor Pin
# define DISTANCE_PIN 27 

float distanceReading; 

// ULN2003 Motor Driver Pins for Closing Mechanisme 
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 17

const int stepsPerRevolution = 2048/2;  // change this to fit the number of steps per revolution

// initialize the stepper library
Stepper myStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);

// 3-5V Motor Pins for pump 
#define pumpPin 7

// Network info

/*const char *ssid = "KranienerveVIII";
const char *password = "NervusVestibulocochlearis";*/
const char* ssid = "Mads iPhone SE";
const char* password = "MadsErSej";

// Maqiatto MQTT info
const char *mqtt_user = "s245033@dtu.dk";
const char *mqtt_pword = "SynthWars2025";
const char *mqtt_topic = "s245033@dtu.dk/Test";

int cnt = 0;
String clientID;

WiFiClient espcli;
PubSubClient client(espcli);

float distanceRead(int pin);

void setup() 
{
  Serial.begin(115200);

  pinMode(RED_PIN, OUTPUT); 
  pinMode(GREEN_PIN, OUTPUT); 

  pinMode(DISTANCE_PIN, INPUT);

  pinMode(pumpPin, OUTPUT);

  myStepper.setSpeed(9);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); // attempt to connect

  Serial.println();
  Serial.println("Wait for WiFi.. ");

  while (WiFi.status() != WL_CONNECTED) { //wait to connect
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(500);

  client.setServer("maqiatto.com", 1883);
  client.setCallback(c_bag);
  clientID += "ESP32Client_";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  for(int i = 0; i < 6; i++)
    clientID += String(mac[i], 16);
  Serial.println(clientID);
}

void loop() 
{ 
  if(!client.connected()) 
  {
    Serial.println("Connecting to MQTT...");
    
    if (client.connect(clientID.c_str(), mqtt_user, mqtt_pword)) 
    {
      Serial.println("Connected"); 
      client.subscribe(mqtt_topic);
    } 
    else 
    {
      Serial.print("failed with state ");
      Serial.print(client.state());
      Serial.println();
      delay(2000);
    }
  }
  else
  {
    if(cnt++ > 5000)
    {
      float tempFloat = ReadTemp(NTC_PIN);
      char tempString [5]; 
      dtostrf(tempFloat, 1, 2, tempString);
      client.publish(mqtt_topic,tempString);
      cnt = 0;

      if (20 <= tempFloat && tempFloat <= 25) {
        green = 255;
        blue = map(tempFloat, 20, 25, 20, 0);
        red = map(tempFloat,20, 25, 0, 20);
      }
      else if (tempFloat < 20) {
        green = map(tempFloat,15,20,0,255);
        blue = map(tempFloat, 15, 20, 255, 20);
        red = 0;
      }
      else {
        green = map(tempFloat,25,30,225,0);
        red = map(tempFloat, 25, 30, 20, 255);
        blue = 0;
      }
      ledcWrite(RED_PIN,red);
      ledcWrite(GREEN_PIN,green);
      ledcWrite(BLUE_PIN,blue);
      
    }
    client.loop();
    delay(1);
  }
}

void c_bag(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Topic: ");
  Serial.println(topic);

  if (length == 0) {
    Serial.println("Empty payload received!");
    return;
  }

  char messageBuffer[length + 1];
  memcpy(messageBuffer, payload, length);
  messageBuffer[length] = '\0'; // null-terminate string

  Serial.print("Payload: ");
  Serial.println(messageBuffer);
  Serial.println("==");

  float value = atof(messageBuffer);

  // Topic check
  if (strcmp(topic, "s245033@dtu.dk/Test") == 0) {
    if (strcmp(messageBuffer, "ShutOff") == 0) {
      Serial.println("Executing shut-off protocol...");
      myStepper.step(stepsPerRevolution);
      delay(1000); //Button most be pressed for 1 sec
      myStepper.step(stepsPerRevolution);
    } 
    else {
      Serial.print("Temperature received: ");
      Serial.println(value);
    }
  }
}
