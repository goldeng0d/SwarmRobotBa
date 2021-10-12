
#ifndef socket2ros_h__
#define socket2ros_h__


#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>


// #define DEBUG_VW


class Socket2ROS{

    public:
        // Constructor has no actions in it
        Socket2ROS();
        // Disconnects from socket server and WiFi
        ~Socket2ROS();

        // WiFi client object to handle Socket communication
        WiFiClient client;

        /**
         * connect to WiFi network with SSID and password
         * connect to socket server with IP and port
         * retry both until a connection is established
        */
        void setup();

        /**
         * read msg from socket server if available
         * parse msg into doubles
         * 
         * @param[out] vel Linear Velocity received from socket server
         * @param[out] omega Angular Velocity received from socket server
        */
        void update(double & vel, double & omega);

        /**
         * connect to Socket Server
        */
        void connectSocket();


    private:

        // Network SSID and password
        const char* ssid = "*******";
        const char* password = "*********";

        // Socket server hostname/ip and port
        const char * host = "192.168.110.74";
        const uint16_t port = 2007;

        // name of robot in ROS
        const char * roboName = "robot1";
      





};

#endif