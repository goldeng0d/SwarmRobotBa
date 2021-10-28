#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "Drive.h"
#include "Socket2ROS.h"
#include <iostream>
#include <sstream>
#include "FastLED.h"

#define NUM_LEDS 4
CRGB leds[NUM_LEDS];

const int ledPin = 2; 

unsigned long lastMillis = 0;
volatile unsigned long ganzezeit = 0;
int32_t encValueLEFT = 0;
int32_t encValueRIGHT = 0;
volatile int valueInt = 0;
volatile int speed = 0;
Drive drive;

uint8_t red = 0, green = 0, blue = 0;

double v;
double w;
volatile double rpmVorgabe;

const char *ssid = "MyWiFiCar";
const char *password = "12345678";
AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");
volatile double dT = 0;
//Socket2ROS rosSocket;

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
        red = valueInt;
        leds[0] = CRGB(red, green, blue);
        leds[1] = CRGB(red, green, blue);
        leds[2] = CRGB(red, green, blue);
        leds[3] = CRGB(red, green, blue);
        FastLED.show();
      }
      else if (key == "GREEN Light")
      {
        green = valueInt;
        leds[0] = CRGB(red, green, blue);
        leds[1] = CRGB(red, green, blue);
        leds[2] = CRGB(red, green, blue);
        leds[3] = CRGB(red, green, blue);
        FastLED.show();
      }
      else if (key == "BLUE Light")
      {
        blue = valueInt;
        leds[0] = CRGB(red, green, blue);
        leds[1] = CRGB(red, green, blue);
        leds[2] = CRGB(red, green, blue);
        leds[3] = CRGB(red, green, blue);
        FastLED.show();
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

void setup(){
  Serial.begin(9600);

  //rosSocket.setup();
  drive.setup();
  drive.move(0);

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
  FastLED.addLeds<WS2812B, 23, RGB>(leds, NUM_LEDS);
}


void loop(){

  wsCarInput.cleanupClients();
  
  // Serial.printf("main loop encvalue = %l\n", enccounter);
  // Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());

  //calculate loop time
  // if (valueInt != speed && valueInt > 4)
  // {
  //   speed = valueInt;
  // }
    double dT = (millis() - lastMillis) / 1000.0;
    lastMillis = millis();
  //  ganzezeit += (dT*1000);
  //  if(ganzezeit >= 50)
  //  {

  rpmVorgabe = (speed * RPM_MAX) / 100.0;
  //Serial.printf("RPM Value vorgabe Haupschleife vor übergabe  = %d \n", (int)rpmVorgabe);
  //drive.rpmcontrol((unsigned int)rpmVorgabe);
  //Serial.printf("ganzzeit = %ld \n", ganzezeit);
  //encValueLEFT = drive.getEncoderValueLEFT();
  //encValueRIGHT = drive.getEncoderValueRIGHT();
  //Serial.printf("Encoder Value Left  = %03d \n", encValueLEFT);
  //Serial.printf("Encoder Value Right = %03d \n", encValueRIGHT);
  //valueInt = Speed from User Interface
  //ganzezeit = 0;
  //  }
  v = speed;
  // if(rosSocket.client.connected()){

  //   Serial.println("connected...");
  //   rosSocket.update(v, w);
     drive.update(dT, v, w);

  // }else{

  //   Serial.println("not connected...");
  //   drive.update(dT, 0, 0);
  //   rosSocket.connectSocket();

  // }

}

