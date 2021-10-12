
#include "Socket2ROS.h"

Socket2ROS::Socket2ROS(){

}

Socket2ROS::~Socket2ROS(){
    client.stop();
    WiFi.disconnect();
}

void Socket2ROS::setup(){

  // WiFi client
  Serial.print("Connecting to Network: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  // wait for network connection
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());

  // Socket client (also waiting for connection)
  connectSocket();

}

void Socket2ROS::update(double & vel, double & omega){

  // enough bytes in buffer? -> read until is blocking... (avoided here)
  if(client.available() > 16){

    // msg:     #vvv.vvv:www.www  limited to +/-99.999
    // example: #001.500:-16.000
    // receive one full msg
    String msg = client.readStringUntil('#');

    // seperate into single msgs seperated by ":"
    int seperatorIndex = msg.indexOf(':');
    String v_msg = msg.substring(0, seperatorIndex);
    String w_msg = msg.substring(seperatorIndex+1, msg.length());

    // convert ot Double
    vel = v_msg.toDouble();
    omega = w_msg.toDouble();

    #ifdef DEBUG_VW
        Serial.println(v);
        Serial.println(w);
        Serial.println("");
    #endif

  }

}

void Socket2ROS::connectSocket(){

  while(!client.connect(host, port)){
    Serial.println("Connection to host failed... trying again in 5 sec");
    delay(5000);
  }

  // send name of client
  client.print(roboName);
  Serial.println("Connected to Server");

  // clear the socket receive buffer
  String msg = client.readStringUntil('#');

}