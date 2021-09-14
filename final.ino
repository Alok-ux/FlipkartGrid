#include <ESP8266WiFi.h>
#include <Servo.h>

WiFiClient client;
WiFiServer server(80);
Servo myservo;

const char* ssid = "JioFibreArnab4G";
const char* password = "arnabanup";

#define en_A D5
#define en_B D6
#define in_1 D3
#define in_2 D4
#define in_3 D7
#define in_4 D8
#define servoPin 5 //D2

int lm,rm,ser; 
  
void setup() {
  Serial.begin(115200);
  delay(10);
//Declaring l293d control pins as Output
  pinMode(en_A, OUTPUT);
  pinMode(en_B, OUTPUT);
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(in_3, OUTPUT);
  pinMode(in_4, OUTPUT);

  myservo.attach(servoPin); 
 
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  WiFi.begin(ssid, password);

while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  // Start the server
  server.begin();
  Serial.println("Server started");
 
  // Print the IP address on serial monitor
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");    //URL IP to be typed in mobile/desktop browser
  Serial.print(WiFi.localIP());
  Serial.println("/");
 
}

void loop() {
  // Check if a client has connected
  int a;
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
Serial.println("new client");
  while(!client.available()){
    delay(1);
  }

String request = client.readStringUntil('\r');
  Serial.println(request);
  client.flush();

for(int i=5 ; i<request.length();i++)
{
  if(request[i]==' ')
  {
    a=i;
  }
    
}
String sub = request.substring(5,a);
func(sub);

                             //Right motor direction
if(rm>0) {
 digitalWrite(in_1, HIGH);
 digitalWrite(in_2, LOW);
 }
 else {
  digitalWrite(in_1, LOW);
  digitalWrite(in_2, HIGH);
 }
 
                              //Left motor direction
 if(lm>0) {
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

                              //Servo
 if(ser==1){
   myservo.write(180);
 }
 else {
  myservo.write(0);
 }

   
}


void func(String sub)
{
  int k =0;
  
  int arr[4];
  
  for(int j=0;j<sub.length();j++)
  {
    if(sub[j]==',')
    {
       arr[k]=j;
       k =k+1;
    }
   }
  
  lm = (sub.substring(0,arr[0])).toInt();
  rm = (sub.substring(arr[0]+1,arr[1])).toInt();
  ser = (sub.substring(arr[1]+1)).toInt();
  
  Serial.println(lm);
  Serial.println(rm);
  Serial.println(ser);

}
  //GET /47,194,1,1,1 HTTP/1.1
