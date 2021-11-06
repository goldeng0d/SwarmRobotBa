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
#define TIMER0_INTERVAL_MS 20
#define TIMER0_INTERVAL_US 1000 * TIMER0_INTERVAL_MS
#define TOGGLEPIN1 21
#define TOGGLEPIN2 22

#define NUM_LEDS 4
#define LEDPIN 23
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, LEDPIN, NEO_GRB + NEO_KHZ800);
uint8_t redvalue = 0, greenvalue = 0, bluevalue = 0;
void showLEDs(int red, int green, int blue);

float lastMillis = 0;
volatile float min500mstimer = 0;
volatile float min5mstimer = 0;
// volatile double rpmVorgabe;
volatile int32_t encValueLEFT = 0;
volatile int32_t encValueRIGHT = 0;
volatile int valueInt = 0;
volatile int speed = 0;
volatile int movement = 0;
Drive drive;

volatile int32_t encoderValueleft = 0;
volatile int32_t encoderValueright = 0;
volatile float leftrpmValue = 0.0;
volatile float rightrpmValue = 0.0;
float dT = TIMER0_INTERVAL_MS;
volatile uint32_t Controlvalueleft = 0; // Theoretically the control value can be put into drive class
volatile uint32_t Controlvalueright = 0; // Theoretically the control value can be put into drive class
volatile uint32_t DesiredRPM = 0;

// Init ESP32 timer 0
hw_timer_t * timer0 = NULL;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR TimerHandler0(void)
{
   static bool toggle0 = false;

  //timer interrupt toggles pin LED_BUILTIN
  digitalWrite(TOGGLEPIN1, toggle0);
  toggle0 = !toggle0;
  
  encoderValueleft = drive.getEncoderValueLEFT();
  encoderValueright = drive.getEncoderValueRIGHT();

  // leftrpmValue = drive.CalculateRPMfromEncoderValue(encoderValueleft, dT); // floating point operation in ISR is bad
  // rightrpmValue = drive.CalculateRPMfromEncoderValue(encoderValueright, dT); // floating point operation in ISR is bad

  // Controlvalueleft = drive.IRegler(DesiredRPM, Controlvalueleft, (uint32_t)leftrpmValue);
  // Controlvalueright = drive.IRegler(DesiredRPM, Controlvalueright, (uint32_t)rightrpmValue);
}

//Websocket for Car Control
const char *ssid = "MyWiFiCar";
const char *password = "12345678";
AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");

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
        <td class="button" ontouchstart='sendButtonInput("MoveCar","1")' onclick='sendButtonInput("MoveCar","1")' ontouchend='sendButtonInput("MoveCar","0")' onmouseup='sendButtonInput("MoveCar","0")'><span class="arrows" >&#8679;</span></td>
        <td></td>
      </tr>
      <tr>
        <td class="button" ontouchstart='sendButtonInput("MoveCar","3")' onclick='sendButtonInput("MoveCar","3")' ontouchend='sendButtonInput("MoveCar","0")' onmouseup='sendButtonInput("MoveCar","0")'><span class="arrows" >&#8678;</span></td>
        <td class="button" ></td>    
        <td class="button" ontouchstart='sendButtonInput("MoveCar","4")' onclick='sendButtonInput("MoveCar","4")' ontouchend='sendButtonInput("MoveCar","0")' onmouseup='sendButtonInput("MoveCar","0")'><span class="arrows" >&#8680;</span></td>
      </tr>
      <tr>
        <td></td>
        <td class="button" ontouchstart='sendButtonInput("MoveCar","2")' onclick='sendButtonInput("MoveCar","2")' ontouchend='sendButtonInput("MoveCar","0")' onmouseup='sendButtonInput("MoveCar","0")'><span class="arrows" >&#8681;</span></td>
        <td></td>
      </tr>
      <tr/><tr/>
      <tr>
        <td style="text-align:left"><b>Speed:</b></td>
        <td colspan=2>
         <div class="slidecontainer">
            <input type="range" min="0" max="30000" value="14000" class="slider" id="Speed" oninput='sendButtonInput("Speed",value)'>
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
        // valueInt = Value from User Interface
        movement = valueInt;
        drive.move(valueInt);
        if (movement == 0)
        {
           drive.setDutyMotor(MOTORLEFT, 0);
           drive.setDutyMotor(MOTORRIGHT, 0);
           Controlvalueleft = 2.0 * drive.minPWMvaluestartturn;
           Controlvalueright = 1.45 * drive.minPWMvaluestartturn;
           leftrpmValue = 0;
           rightrpmValue = 0;
        }
      }
      else if (key == "Speed")
      {
        // valueInt = Value from User Interface
        speed = valueInt;
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
  Serial.begin(115200);
  // delay(500);
  Serial.println("CPU Frequency in MHZ:");
  Serial.println(getCpuFrequencyMhz());

  //Timer setup
  pinMode(TOGGLEPIN1, OUTPUT);
  pinMode(TOGGLEPIN2, OUTPUT);
  timer0 = timerBegin(0, 80, true); // 12,5 ns * 80 = 1000ns = 1us
  timerAttachInterrupt(timer0, &TimerHandler0, /*true*/ false); //edge interrupts do not work, use false
  timerAlarmWrite(timer0, TIMER0_INTERVAL_US, true);             // 1us * 1000 = 1ms
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
  
  //For Testing without connecting to WIFI
  speed = 14000;
  movement = 1;
  drive.move(1);

  //Adjust Motor differences and start values for Motors here and in onCarInputWebSocketEvent (movement == 0)
  Controlvalueleft = 2.0 * drive.minPWMvaluestartturn;  //~150 for resolution == 8bit, adjusting up for worse motor
  Controlvalueright = 1.45 * drive.minPWMvaluestartturn; //~90 for resolution == 8bit, adjusting to match initial speed of motorleft
  
  // The above Ratio is calibrated Manually it could be automated by running a calibration in setup() 
  // where the calibration is done at a fixed RPM -> when both Motors are at the same RPM both Stellwert values can be taken to get that ratio
  // set the Ratio for the next move of the car see (movement == 0) to this ratio where higher value is around 60% of maxPWMvalue to get a good Starting RPM where both Motors have close to equal RPM

}

void loop()
{
  // Clear Inactive Clients 
  wsCarInput.cleanupClients();

//   // Serial.printf("main loop encvalue = %l\n", enccounter);
//   // Serial.printf("SPIRam Total heap %d, SPIRam Free Heap %d\n", ESP.getPsramSize(), ESP.getFreePsram());
//   // Serial.printf("RPM Value vorgabe Haupschleife vor Uebergabe  = %d \n", speed);
//   //Serial.printf("ganzzeit = %ld \n", min500mstimer);
  DesiredRPM = speed;
  float timedifference = ((float)millis() - lastMillis) / (float)1000;
  lastMillis = millis();
  min500mstimer += (timedifference * 1000);
  min5mstimer += (timedifference * 1000);
  if (min5mstimer >= 200)
  {
    min5mstimer = 0;
    if (movement > 0) // while moving in any Direction
    {
      // Moving Direction of Motors can be obtained by evaluating the sign of encoder value here
      // For Calculating RPM, the encoder sign is irrelevant and makes the implementation of Controller unnecessary hard
      if (encoderValueleft < 0)
      {
        encoderValueleft = -encoderValueleft;
      }
      if (encoderValueright < 0)
      {
        encoderValueright = -encoderValueright;
      }

      // The Basic Idea is that every 20ms the Encoder value is read in TimerInterrupt0
      // and the RPM value Value for the left and right Motors is calculated form the Encoder Value and the time difference 20ms from the Timer.
      // Then Adjust the PWM Value for the Motors
      // Then Output the PWM Value for the Motors
      leftrpmValue = drive.CalculateRPMfromEncoderValue(encoderValueleft, dT);
      rightrpmValue = drive.CalculateRPMfromEncoderValue(encoderValueright, dT);
      Controlvalueleft = drive.IRegler(DesiredRPM, Controlvalueleft, (uint32_t)leftrpmValue);
      Controlvalueright = drive.IRegler(DesiredRPM, Controlvalueright, (uint32_t)rightrpmValue);
      drive.setDutyMotor(MOTORLEFT, Controlvalueleft);
      drive.setDutyMotor(MOTORRIGHT, Controlvalueright);
        
     }
     
  }
  if(min500mstimer >= 1000)
  {
      //timer interrupt toggles pin LED_BUILTIN
      // digitalWrite(TOGGLEPIN2, toggle1);
      // toggle1 = !toggle1;

      // encValueLEFT = drive.getEncoderValueLEFT();
      // encValueRIGHT = drive.getEncoderValueRIGHT();

      Serial.println("\n"); //here
      Serial.printf("Encoder Value Left Interrupt  = %d \n", encoderValueleft); //here
      Serial.printf("Encoder Value Right Interrupt = %d \n", encoderValueright); //here
      // Serial.printf("DesiredRPM = %d\n", DesiredRPM);

      // Serial.println("min500mstimer = ");
      // Serial.println(min500mstimer);
      // digitalWrite(TOGGLEPIN2, toggle1);
      // toggle1 = !toggle1;
      min500mstimer = 0;
      

      // =================================================Wagner Regler
      //  if (DesiredRPM > encValueRIGHT && Stellwert < 255)
      //  {
      //      Stellwert += 2;
      //  }
      //  else if (DesiredRPM < encValueRIGHT && Stellwert > 0){
      //    Stellwert -= 2;
      //  }
      //  drive.setDutyMotor(MOTORRIGHT, Stellwert);
      // =================================================
      // Serial.println(Controlvalueright);
      // drive.IRegler(DesiredRPM, Controlvalueright, rightrpmValue);

      Serial.printf("Controlvalueright(PWM): %d\n", Controlvalueright); //here
      Serial.printf("Controlvalueleft(PWM): %d\n", Controlvalueleft);   //here
      Serial.printf("MotorDrehzahl Value Right = %.0f \n", rightrpmValue); //here
      Serial.printf("MotorDrehzahl Value Left = %.0f \n", leftrpmValue);   //here

   }
}
