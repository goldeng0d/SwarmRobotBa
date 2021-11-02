#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "Drive.h"
#include <iostream>
#include <sstream>
#include <Adafruit_NeoPixel.h>

#define LOCAL_DEBUG 1
#define TIMER_INTERRUPT_DEBUG 0
#define TIMER0_INTERVAL_MS 1
#define TOGGLEPIN1 21
#define TOGGLEPIN2 22

#define NUM_LEDS 4
#define LEDPIN 23
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, LEDPIN, NEO_GRB + NEO_KHZ800);
uint8_t redvalue = 0, greenvalue = 0, bluevalue = 0;
void showLEDs(int red, int green, int blue);

    unsigned long lastMillis = 0;
volatile unsigned long ganzezeit = 0;
volatile double rpmVorgabe;
volatile int32_t encValueLEFT = 0;
volatile int32_t encValueRIGHT = 0;
volatile int valueInt = 0;
volatile int speed = 0;
Drive drive;

double v;
double w;

volatile int32_t encoderValueleft;
volatile int32_t encoderValueright;
volatile int32_t Stellwert = 0;
volatile int32_t Sollwert = 0;

// Init ESP32 timer 0
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR TimerHandler0(void)
{
  static bool toggle0 = false;

#if (TIMER_INTERRUPT_DEBUG > 0)
  Serial.print("ITimer0 called, millis() = ");
  Serial.println(millis());
#endif

  //timer interrupt toggles pin LED_BUILTIN
  digitalWrite(TOGGLEPIN1, toggle0);
  toggle0 = !toggle0;

  // portENTER_CRITICAL_ISR(&timerMux0);
  encoderValueleft = drive.getEncoderValueLEFT();
  encoderValueright = drive.getEncoderValueRIGHT();
  // portEXIT_CRITICAL_ISR(&timerMux0);
}

//Websocket for Car Control
const char *ssid = "MyWiFiCar";
const char *password = "12345678";
AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");
volatile double dT = 0;

const char *htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>
  <head>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no"; charset="utf-8"/>
    <style>
    .arrows {
      font-size:40px;
      color:red;
    }
    td.button {
      background-color:black;
      border-radius:25%;
      box-shadow: 5px 5px #888888;
    }
    td.button:active {
      transform: translate(5px,5px);
      box-shadow: none; 
    }
    .noselect {
      -webkit-touch-callout: none; /* iOS Safari */
        -webkit-user-select: none; /* Safari */
         -khtml-user-select: none; /* Konqueror HTML */
           -moz-user-select: none; /* Firefox */
            -ms-user-select: none; /* Internet Explorer/Edge */
                user-select: none; /* Non-prefixed version, currently
                                      supported by Chrome and Opera */
    }
    .slidecontainer {
      width: 100%;
    }
    .slider {
      -webkit-appearance: none;
      width: 100%;
      height: 15px;
      border-radius: 5px;
      background: #d3d3d3;
      outline: none;
      opacity: 0.7;
      -webkit-transition: .2s;
      transition: opacity .2s;
    }
    .slider:hover {
      opacity: 1;
    }
  
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 25px;
      height: 25px;
      border-radius: 50%;
      background: red;
      cursor: pointer;
    }
    .slider::-moz-range-thumb {
      width: 25px;
      height: 25px;
      border-radius: 50%;
      background: red;
      cursor: pointer;
    }
    </style>
  
  </head>
  <body class="noselect" align="center" style="background-color:white">
     
    <!--h2 style="color: teal;text-align:center;">Wi-Fi Camera &#128663; Control</h2-->
    
    <table id="mainTable" style="width:400px;margin:auto;table-layout:fixed" CELLSPACING=10>
      <tr>
        <td></td>
        <td class="button" ontouchstart='sendButtonInput("MoveCar","1")' ontouchend='sendButtonInput("MoveCar","0")'><span class="arrows" >&#8679;</span></td>
        <td></td>
      </tr>
      <tr>
        <td class="button" ontouchstart='sendButtonInput("MoveCar","3")' ontouchend='sendButtonInput("MoveCar","0")'><span class="arrows" >&#8678;</span></td>
        <td class="button"></td>    
        <td class="button" ontouchstart='sendButtonInput("MoveCar","4")' ontouchend='sendButtonInput("MoveCar","0")'><span class="arrows" >&#8680;</span></td>
      </tr>
      <tr>
        <td></td>
        <td class="button" ontouchstart='sendButtonInput("MoveCar","2")' ontouchend='sendButtonInput("MoveCar","0")'><span class="arrows" >&#8681;</span></td>
        <td></td>
      </tr>
      <tr/><tr/>
      <tr>
        <td style="text-align:left"><b>Speed:</b></td>
        <td colspan=2>
         <div class="slidecontainer">
            <input type="range" min="0" max="100" value="80" class="slider" id="Speed" oninput='sendButtonInput("Speed",value)'>
          </div>
        </td>
      </tr>        
      <tr>
        <td style="text-align:left"><b>RED Light:</b></td>
        <td colspan=2>
          <div class="slidecontainer">
            <input type="range" min="0" max="255" value="0" class="slider" id="RED Light" oninput='sendButtonInput("RED Light",value)'>
          </div>
        </td>   
      </tr>
      <tr>
        <td style="text-align:left"><b>GREEN Light:</b></td>
        <td colspan=2>
          <div class="slidecontainer">
            <input type="range" min="0" max="255" value="0" class="slider" id="GREEN Light" oninput='sendButtonInput("GREEN Light",value)'>
          </div>
        </td>   
      </tr>
      <tr>
        <td style="text-align:left"><b>BLUE Light:</b></td>
        <td colspan=2>
          <div class="slidecontainer">
            <input type="range" min="0" max="255" value="0" class="slider" id="BLUE Light" oninput='sendButtonInput("BLUE Light",value)'>
          </div>
        </td>   
      </tr>
    </table>
  
    <script>
      var webSocketCarInputUrl = "ws:\/\/" + window.location.hostname + "/CarInput";
      var websocketCarInput;
      
      function initCarInputWebSocket() 
      {
        websocketCarInput = new WebSocket(webSocketCarInputUrl);
        websocketCarInput.onopen    = function(event)
        {
          var speedButton = document.getElementById("Speed");
          sendButtonInput("Speed", speedButton.value);
          var redlightButton = document.getElementById("RED Light");
          sendButtonInput("RED Light", redlightButton.value);
          var greenlightButton = document.getElementById("GREEN Light");
          sendButtonInput("GREEN Light", greenlightButton.value);
          var bluelightButton = document.getElementById("BLUE Light");
          sendButtonInput("BLUE Light", bluelightButton.value);
        };
        websocketCarInput.onclose   = function(event){setTimeout(initCarInputWebSocket, 2000);};
        websocketCarInput.onmessage = function(event){};        
      }
      
      function initWebSocket() 
      {
        initCarInputWebSocket();
      }
      function sendButtonInput(key, value) 
      {
        var data = key + "," + value;
        websocketCarInput.send(data);
      }
    
      window.onload = initWebSocket;
      document.getElementById("mainTable").addEventListener("touchend", function(event){
        event.preventDefault()
      });      
    </script>
  </body>    
</html>
)HTMLHOMEPAGE";

void moveCar(int inputValue)
{
  Serial.printf("Got value as %d\n", inputValue);
  drive.move(inputValue);
}

void handleRoot(AsyncWebServerRequest *request)
{
  request->send_P(200, "text/html", htmlHomePage);
}

void handleNotFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "File Not Found");
}



void onCarInputWebSocketEvent(AsyncWebSocket *server,
                              AsyncWebSocketClient *client,
                              AwsEventType type,
                              void *arg,
                              uint8_t *data,
                              size_t len)
{
  
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
    //moveCar(0);
    //ledcWrite(PWMLightChannel, 0);
    break;
  case WS_EVT_DATA:
    AwsFrameInfo *info;
    info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
      std::string myData = "";
      myData.assign((char *)data, len);
      std::istringstream ss(myData);
      std::string key, value;
      std::getline(ss, key, ',');
      std::getline(ss, value, ',');
      Serial.printf("Key [%s] Value[%s]\n", key.c_str(), value.c_str());
      valueInt = atoi(value.c_str());
      if (key == "MoveCar")
      {
        //moveCar(valueInt);
        drive.move(valueInt);
      }
      else if (key == "Speed")
      {
        //ledcWrite(PWMSpeedChannel, valueInt);
        speed = valueInt;
        drive.setspeed(valueInt);
      }
      else if (key == "RED Light")
      {
        redvalue = valueInt;
        showLEDs(redvalue, greenvalue, bluevalue);
      }
      else if (key == "GREEN Light")
      {
        greenvalue = valueInt;
        showLEDs(redvalue, greenvalue, bluevalue);
      }
      else if (key == "BLUE Light")
      {
        bluevalue = valueInt;
        showLEDs(redvalue, greenvalue, bluevalue);
      }
    }
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  default:
    break;
  }
}

void showLEDs(int red, int green, int blue)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
  }
  pixels.show();
}

void setup(){
  Serial.begin(9600);
  Serial.println("CPU Frequency in MHZ:");
  Serial.println(getCpuFrequencyMhz());

  //Timer setup
  pinMode(TOGGLEPIN1, OUTPUT);
  pinMode(TOGGLEPIN2, OUTPUT);
  timer0 = timerBegin(0, 80, true); // 12,5 ns * 80 = 1000ns = 1us
  timerAttachInterrupt(timer0, &TimerHandler0, true);
  timerAlarmWrite(timer0, 1000, true);
  timerAlarmEnable(timer0);

  //Drive setup
  drive.setup();
  drive.move(0);

  //WIFI/TCP Websocket setup
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("\nAP IP address: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);
  wsCarInput.onEvent(onCarInputWebSocketEvent);
  server.addHandler(&wsCarInput);

  server.begin();
  Serial.print("HTTP server started\n");

  //LED setup
  
  pixels.begin();
}

void loop()
{
  // drive.move(1);
  wsCarInput.cleanupClients();

  // Serial.printf("main loop encvalue = %l\n", enccounter);
  // Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
   rpmVorgabe = (speed * RPM_MAX) / 100.0;
  // Serial.printf("RPM Value vorgabe Haupschleife vor Ã¼bergabe  = %d \n", (int)rpmVorgabe);
  // drive.rpmcontrol((unsigned int)rpmVorgabe);
  //Serial.printf("ganzzeit = %ld \n", ganzezeit);

  double dT = (millis() - lastMillis) / 1000.0;
  lastMillis = millis();
  ganzezeit += (dT * 1000);
  if (ganzezeit >= 200)
  {
    //timer interrupt toggles pin LED_BUILTIN
    // digitalWrite(TOGGLEPIN2, toggle1);
    // toggle1 = !toggle1;

    // encValueLEFT = drive.getEncoderValueLEFT();
    // encValueRIGHT = drive.getEncoderValueRIGHT();

    // double leftrpmValue = (((double)encValueLEFT / ENCODER_COUNTS_PER_REVOLUTION_MOTORSIDE) / (double)ganzezeit / 1000) * SEC_IN_MIN;
    // double rightrpmValue = (((double)encValueRIGHT / ENCODER_COUNTS_PER_REVOLUTION_MOTORSIDE) / (double)ganzezeit / 1000) * SEC_IN_MIN;

    // Serial.printf("Encoder Value Left  = %d \n", encValueLEFT);
    // Serial.printf("Encoder Value Right = %d \n", encValueRIGHT);

    // Serial.printf("MotorDrehzahl Value Left  = %f \n", leftrpmValue);
    // Serial.printf("MotorDrehzahl Value Right = %f \n", rightrpmValue);
    // Serial.println("dT = ");
    // Serial.println(ganzezeit);
    // digitalWrite(TOGGLEPIN2, toggle1);
    // toggle1 = !toggle1;
    //valueInt = Speed from User Interface
    //   ganzezeit = 0;
    // }

    // if(rosSocket.client.connected()){
    v = valueInt;

    if (Sollwert > encValueRIGHT && Stellwert < 255)
    {
      Stellwert += 2;
    }
    else if (Sollwert < encValueRIGHT && Stellwert > 0){
      Stellwert -= 2;
    }
    drive.setDutyMotor(MOTORRIGHT, Stellwert);
    
    // Serial.println("Stellwert:");
    // Serial.println(Stellwert);
    // Serial.println("encValeRight");
    // Serial.println(encValueRIGHT);

    //   Serial.println("connected...");
    //   rosSocket.update(v, w);
    //drive.update(dT, v, w);

    // }else{

    //   Serial.println("not connected...");
    //   drive.update(dT, 0, 0);
    //   rosSocket.connectSocket();

   }
}
