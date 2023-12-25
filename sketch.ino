#include <DHT.h>
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define DHTPIN 2
#define DHTTYPE DHT22  
#define buzzer 15
#define AO_PIN 39

DHT dht(DHTPIN, DHTTYPE);
  
const float GAMMA = 0.7;
const float RL10 = 50;

int theta_off = 30;
float gammaValue = 0.75;
int on = 0;
int repeat = 0;
int delayT = 3;
int frequency = 256;

Servo myservo; 

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void setup() {

  Serial.begin(115200);
  setupWifi();
  setupMqtt();
  dht.begin();
  myservo.attach(13);
  pinMode(buzzer, OUTPUT);
  pinMode(AO_PIN, INPUT);
 
}

void loop() {

  if(!mqttClient.connected()){
    connectToBroker();
  }

  mqttClient.loop();

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  String payload1 = String(t); 
  mqttClient.publish("190680P-DATA/temp", payload1.c_str()); 
  String payload2 = String(h); 
  mqttClient.publish("190680P-DATA/humidity", payload2.c_str());
  
  int analogValue = analogRead(AO_PIN);
  float voltage = analogValue / 4096. * 5;
  float resistance = 2000 * voltage / (1 - voltage / 5);
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));
  float lux_scaled = min(lux/100000.0,1.0);

  String payload3 = String(lux_scaled); 
  mqttClient.publish("190680P-DATA/intensity", payload3.c_str());

  int theta = theta_off + (180-theta_off)*lux_scaled*gammaValue;

  myservo.write(theta); 
  
  delay(1000);

  
}

void setupWifi(){
  WiFi.begin("Wokwi-GUEST","");

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupMqtt(){
  mqttClient.setServer("test.mosquitto.org",1883);
  mqttClient.setCallback(receiveCallback);
}

void connectToBroker(){
  while(!mqttClient.connected()){
    Serial.print("Attempting MQTT connection...");
    if(mqttClient.connect("ESP32-465454454545")){
      Serial.println("connected");
      mqttClient.subscribe("190680P-OUT/#");
    }else{
      Serial.print("failed ");
      Serial.print(mqttClient.state());
      delay(5000);
    }
  }
}

void receiveCallback(char* topic, byte* payload, unsigned int length){
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char payloadCharAr[length];
  for(int i=0;i<length;i++){
    Serial.print((char)payload[i]);
    payloadCharAr[i] = (char)payload[i];
  }
  Serial.println();

  if (strcmp(topic,"190680P-OUT/minAngle") == 0){
    int Value1 = atoi(payloadCharAr); 
    theta_off = Value1;
  }

  if (strcmp(topic,"190680P-OUT/gamma") == 0){
    float Value2 = atof(payloadCharAr); 
    gammaValue = Value2;
  }

  if (strcmp(topic,"190680P-OUT/mainSwitch") == 0){
    int Value3 = atoi(payloadCharAr); 
    on = Value3;
  }

  if (strcmp(topic,"190680P-OUT/delay") == 0){
    int Value4 = atoi(payloadCharAr); 
    delayT = Value4;
  }

  if (strcmp(topic,"190680P-OUT/frequency") == 0){
    int Value5 = atoi(payloadCharAr); 
    frequency = Value5;
  }

  if (strcmp(topic,"190680P-OUT/repeat") == 0){
    int Value6 = atoi(payloadCharAr); 
    repeat = Value6;
  }

  if (strcmp(topic,"190680P-OUT/buzzer") == 0){
    int Value7 = atoi(payloadCharAr); 
    if (Value7 == 1){
      buzzerF();
    }
  }

}

void buzzerF(){
  if (on==1){
    if (repeat==1){
      for(int i=0;i<10;i++){
        tone(buzzer,frequency,2000);
        delay (delayT*1000);
      }
      
    }else{
        tone(buzzer,frequency,10000);
    }
  }    
}