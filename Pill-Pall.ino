# include <Arduino.h>
# include <WiFi.h>
# include <PubSubClient.h>
# include <esp_wifi.h>
# include <Stepper.h>

//LED pin 
# define RED_PIN 25
# define GREEN_PIN 26

// Distance Sensor Pin
# define TRIG_PIN 27
# define ECHO_PIN 34 

int boxWidth = 10; 
int boxLength = 20; 
int boxHeight = 20; 

/*
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
*/

// Network info

/*const char *ssid = "KranienerveVIII";
const char *password = "NervusVestibulocochlearis";*/
const char* ssid = "Mads iPhone SE";
const char* password = "MadsErSej";

// Maqiatto MQTT info
const char *mqtt_user = "s245033@dtu.dk";
const char *mqtt_pword = "SynthWars2025";
const char *mqtt_topic = "s245033@dtu.dk/PillPall";

int cnt = 0;
String clientID;

WiFiClient espcli;
PubSubClient client(espcli);

//Function prototypes
float volume(void)

void motorstart(int time);

void setup() 
{
  Serial.begin(115200);

  pinMode(RED_PIN, OUTPUT); 
  pinMode(GREEN_PIN, OUTPUT); 

  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN, INPUT);


  /* 
  pinMode(pumpPin, OUTPUT);

  myStepper.setSpeed(9);
  */

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
      float volFloat = volume();
      Serial.println(volFloat);
      char volString [5]; 
      dtostrf(volFloat, 1, 2, volString);
      client.publish(mqtt_topic,volString);
      cnt = 0;
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

  //float value = atof(messageBuffer); //Converts payload to float, doesn't really matter 

  // Topic check
  if (strcmp(topic, "s245033@dtu.dk/PillPall") == 0) {
    switch (messageBuffer[0]) {
      case 'A': 
        Serial.println("Dispensing 100 mL");
        motortime(100);
        break;
        //Dispensing 100 mL 
      case 'B': 
        Serial.println("Dispensing 150 mL");
        break;
        //Dispensing 150 mL 
      case 'C': 
        Serial.println("Dispensing 200 mL");
        break;
        //Dispensing 200 mL 
      case 'D': 
        //Opening infusion
        Serial.println("Opening valve and starting dispensing IV-solution");
        digitalWrite(GREEN_PIN,HIGH);
        digitalWrite(RED_PIN,LOW);
        break;
      case 'E': 
        //Closing infusion
        Serial.println("Closing valve and stopping dispensing IV-solution");
        digitalWrite(GREEN_PIN,LOW);
        digitalWrite(RED_PIN,HIGH); 
        break; 
      default: 
        Serial.println("Volume of doom-bag is:");
        Serial.println(messageBuffer);
        break;
    }
  }
}

//volume function 
float volume(void) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  int duration = pulseIn(ECHO_PIN, HIGH);
  Serial.println(duration);
  float distance = (duration*.0343)/2;
  Serial.println(distance);
  float volume = (boxWidth*boxLength*(boxHeight-distance)); 
  Serial.println(volume);
  return volume;
}

void motorstart(int time){
  
}
