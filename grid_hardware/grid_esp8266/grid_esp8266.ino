#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <Servo.h>
#define MAX_SRV_CLIENTS 1
Servo myservo;
WebSocketsServer webSocket = WebSocketsServer(8888);

const char* ssid = "JioFibreArnab4G";
const char* password = "arnabanup";
#define en_A D5
#define en_B D6
#define in_1 D3
#define in_2 D4
#define in_3 D7  //D1 for 1 bot n D7 for other
#define in_4 D8
#define servoPin 4 //D2

int lm,rm,ser;

void setup() {
  delay(10);
//Declaring l293d control pins as Output
  pinMode(en_A, OUTPUT);
  pinMode(en_B, OUTPUT);
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(in_3, OUTPUT);
  pinMode(in_4, OUTPUT);

  myservo.attach(servoPin);

    Serial.begin(115200);
    Serial.print("\n\n\nConnecting to ");
    Serial.print(ssid);
    WiFi.begin(ssid, password);
    while(WiFi.status() != WL_CONNECTED) {
        delay(100);
        Serial.print(".");

        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
    Serial.println("");
    Serial.println("WiFi connected");

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket Server started");
    Serial.print("\nUse this URL to connect: ");
    Serial.print("ws://");    //URL IP to be typed in mobile/desktop browser
    Serial.print(WiFi.localIP());
    Serial.println(":8888");

}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            moveBot("0","0","0");
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            }
            break;
        case WStype_TEXT:
        {
            Serial.printf("[%u] get Text: %s\n", num, payload);
            int k=0;
            String ls="",rs="",sm="";
            while(payload[k]!=',')
              ls+=(char)payload[k++];
            k++;
            while(payload[k]!=',')
              rs+=(char)payload[k++];
            k++;
            while(k<length)
              sm+=(char)payload[k++];
            moveBot(ls,rs,sm);
              webSocket.sendTXT(num, "ok");
       }
            break;
    }
}

void moveBot(String ls,String rs,String sm){
  lm = ls.toInt();
  rm = rs.toInt();
  ser = sm.toInt();

  Serial.println(lm);
  Serial.println(rm);
  Serial.println(ser);

                               //Right motor direction
  if(rm<0) {
   digitalWrite(in_1, HIGH);
   digitalWrite(in_2, LOW);
   }
  else {
   digitalWrite(in_1, LOW);
   digitalWrite(in_2, HIGH);
   }

                                //Left motor direction
  if(lm<0) {
   digitalWrite(in_3, HIGH);
   digitalWrite(in_4, LOW);
   }
  else {
   digitalWrite(in_3, LOW);
   digitalWrite(in_4, HIGH);
   }
                                //Speed of motors
   analogWrite(en_A, abs(rm));
   analogWrite(en_B, abs(lm));

   if(ser<0){
      myservo.write(-ser);
   }else if(ser==1){
     myservo.write(180);
   }
   else if (ser==0) {
     myservo.write(0);
   }else{
      Serial.println("delay..");
     delay(ser);//(-100,100,600)  (+90, -90, +180, +360)
     Serial.println("stop..");
     moveBot("0","0","0");
   }
}
void loop() {

   webSocket.loop();

}
