

// Load Wi-Fi library
#include <ESP8266WiFi.h>

// Replace with your network credentials
const char* ssid     = "Ajinkya";
const char* password = "Ajab9028";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output5State = "off";
String output6State = "off";
String output7State = "off";
// Assign output variables to GPIO pins
const int output5 = 14;
const int output6 = 12;
const int output7 = 13;
// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

#define TRIGGERPIN D2
#define ECHOPIN  D3
long duration, distance;  
void Ultrasonic(void);

namespace pin
{
const byte tds_sensor = A0;   // TDS Sensor
}
 
namespace device
{
float aref = 3.3;
}
 
namespace sensor
{
float ec = 0;
unsigned int tds = 0;

float ecCalibration = 1;
}


//#define SENSOR1  D8
int SENSOR1 = 3;   // rx pin
long currentMillis1 = 0;
long previousMillis1 = 0;
int interval1 = 1000;
float calibrationFactor1 = 4.5;
volatile byte pulseCount1;
byte pulse1Sec1 = 0;
float flowRate1;
unsigned int flowMilliLitres1;
unsigned long totalMilliLitres1;

#define SENSOR2  D1
long currentMillis2 = 0;
long previousMillis2 = 0;
int interval2 = 1000;
float calibrationFactor2 = 4.5;
volatile byte pulseCount2;
byte pulse1Sec2 = 0;
float flowRate2;
unsigned int flowMilliLitres2;
unsigned long totalMilliLitres2;
void IRAM_ATTR pulseCounter1()
{
  pulseCount1++;
}

void IRAM_ATTR pulseCounter2()
{
  pulseCount2++;
}

void setup() {
  Serial.begin(115200);
  // Initialize the output variables as outputs
  pinMode(output5, OUTPUT);
  pinMode(output6, OUTPUT);
  pinMode(output7, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output5, LOW);
  digitalWrite(output6, LOW);
  digitalWrite(output7, LOW);

 pinMode(TRIGGERPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);
   pinMode(SENSOR1, INPUT_PULLUP);
  pulseCount1 = 0;
  flowRate1 = 0.0;
  flowMilliLitres1 = 0;
  totalMilliLitres1 = 0;
  previousMillis1 = 0;
  attachInterrupt(digitalPinToInterrupt(SENSOR1), pulseCounter1, FALLING);
   pulseCount2 = 0;
  flowRate2 = 0.0;
  flowMilliLitres2 = 0;
  totalMilliLitres2 = 0;
  previousMillis2 = 0;
  attachInterrupt(digitalPinToInterrupt(SENSOR2), pulseCounter2, FALLING);

 
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
 
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();        
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
           
            // turns the GPIOs on and off
            if (header.indexOf("GET /14/on") >= 0) {
              Serial.println("GPIO 14 on");
              output5State = "on";
              digitalWrite(output5, HIGH);
            } else if (header.indexOf("GET /14/off") >= 0) {
              Serial.println("GPIO 14 off");
              output5State = "off";
              digitalWrite(output5, LOW);
            } else if (header.indexOf("GET /12/on") >= 0) {
              Serial.println("GPIO 12 on");
              output6State = "on";
              digitalWrite(output6, HIGH);
            } else if (header.indexOf("GET /12/off") >= 0) {
              Serial.println("GPIO 12 off");
              output6State = "off";
              digitalWrite(output6, LOW);
            }else if (header.indexOf("GET /13/on") >= 0) {
              Serial.println("GPIO 13 on");
              output7State = "on";
              digitalWrite(output7, HIGH);
            } else if (header.indexOf("GET /13/off") >= 0) {
              Serial.println("GPIO 13 off");
              output7State = "off";
              digitalWrite(output7, LOW);
            }
              Ultrasonic();
  flow1();
  flow2();
 // double waterTemp = TempRead();
 double waterTemp = 25;
  waterTemp  = waterTemp*0.0625;                 // conversion accuracy is 0.0625 / LSB
  float rawEc = analogRead(pin::tds_sensor) * device::aref / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  float temperatureCoefficient = 1.0 + 0.02 * (waterTemp - 25.0);  // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
 
  sensor::ec = (rawEc / temperatureCoefficient) * sensor::ecCalibration; // temperature and calibration compensation
  sensor::tds = (133.42 * pow(sensor::ec, 3) - 255.86 * sensor::ec * sensor::ec + 857.39 * sensor::ec) * 0.5; //convert voltage value to tds value
  int tds=(sensor::tds);
   int ecvalue=(sensor::ec);
  Serial.print(F("TDS:")); Serial.println(tds);
 
  Serial.print(F("EC:")); Serial.println(ecvalue, 2);
 
 // Serial.print(F("Temperature:")); Serial.println(waterTemp,2);  
  Serial.print(F(""));
 
 // Blynk.virtualWrite(V1, sensor::tds);
 // Blynk.virtualWrite(V2, sensor::ec);
//  Blynk.virtualWrite(V3, waterTemp);
  //Blynk.virtualWrite(V4, distance);
  delay(200);
           
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");
           
            // Web Page Heading
            client.println("<body><h1>Water Distribution System</h1>");
           
            // Display current state, and ON/OFF buttons for GPIO 5  
            client.println("<p>Valve1 - " + output5State + "</p>");
            // If the output5State is off, it displays the ON button      
            if (output5State=="off") {
              client.println("<p><a href=\"/14/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/14/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
               
            // Display current state, and ON/OFF buttons for GPIO 12
            client.println("<p>Valve2 - " + output6State + "</p>");
            // If the output6State is off, it displays the ON button      
            if (output6State=="off") {
              client.println("<p><a href=\"/12/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/12/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
             client.println("<p>Valve3 - " + output7State + "</p>");
            // If the output7State is off, it displays the ON button      
            if (output7State=="off") {
              client.println("<p><a href=\"/13/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/13/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

           client.println("<hr>Water TDS (PPM) - ""</hr>");
             client.println(tds);
           client.println("<hr>Water Tank Level(CM) - ""</hr>");
             client.println(distance);
           client.println("<hr>Inlet Water Flow (L/m) - ""</hr>");
            client.println(flowRate1);
           client.println("<hr>Outlet Water Flow (L/m)- ""</hr>");
             client.println(flowRate2);

             
    /* client.println( <table>
         <caption> Data Monitering </caption>
          <tr>
              <th> TDS</th>
              <th>Water EC</th>
              <th>Tank Level(cm)</th>
          </tr>
          <tr>
              <td> tds </td>
              <td> ecvalue </td>
              <td> distance </td>
          </tr>
     </table>);    

     */

     
            client.println("</body></html>");
           
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

void Ultrasonic()
{
 
 digitalWrite(TRIGGERPIN, LOW);  
 delayMicroseconds(2);  
 digitalWrite(TRIGGERPIN, HIGH);
 delayMicroseconds(10);
 
 digitalWrite(TRIGGERPIN, LOW);
 duration = pulseIn(ECHOPIN, HIGH);
 distance = (34-(duration*0.034/2));
 Serial.print(distance);
 Serial.println("Cm");
}

void flow1(){
  currentMillis1 = millis();
  if (currentMillis1 - previousMillis1 > interval1) {
    pulse1Sec1 = pulseCount1;
    pulseCount1 = 0;
    flowRate1 = ((1000.0 / (millis() - previousMillis1)) * pulse1Sec1) / calibrationFactor1;
    previousMillis1 = millis();
    flowMilliLitres1 = (flowRate1 / 60) * 1000;
    totalMilliLitres1 += flowMilliLitres1;
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate1: ");
    Serial.print(int(flowRate1));  // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t");       // Print tab space
    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity1: ");
    Serial.print(totalMilliLitres1);
    Serial.print("mL / ");
    Serial.print(totalMilliLitres1 / 1000);
    Serial.println("L");
    // Blynk.virtualWrite(V5, int(flowRate1));
  //   Blynk.virtualWrite(V6, totalMilliLitres1);
     delay(100);
  }
}

void flow2(){
  currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 > interval2) {
    pulse1Sec2 = pulseCount2;
    pulseCount2 = 0;
    flowRate2 = ((1000.0 / (millis() - previousMillis2)) * pulse1Sec2) / calibrationFactor2;
    previousMillis2 = millis();
    flowMilliLitres2 = (flowRate2 / 60) * 1000;
    totalMilliLitres2 += flowMilliLitres2;
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate2: ");
    Serial.print(int(flowRate2));  // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t");       // Print tab space
    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity2: ");
    Serial.print(totalMilliLitres2);
    Serial.print("mL / ");
    Serial.print(totalMilliLitres2 / 1000);
    Serial.println("L");
   
 // Blynk.virtualWrite(V7, int(flowRate2));
 // Blynk.virtualWrite(V8, totalMilliLitres2);
    delay(100);
  }
}
