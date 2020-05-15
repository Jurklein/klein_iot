#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <ESP8266HTTPClient.h>

//#include <ArduinoJson.h>

#include <SocketIoClient.h>
#include <WebSocketsClient.h>


//#include <Hash.h>

#define USE_SERIAL Serial
#define SERVER_IP "192.168.0.101:3030"

//char host[] = "192.168.0.101"; // Socket.IO Server Address
//int port = 3030; // Socket.IO Port Address
//char path[] = "/users"; // Socket.IO Base Path


ESP8266WiFiMulti WiFiMulti;
SocketIoClient webSocket;

void event(const char * payload, size_t length) {
  //USE_SERIAL.printf("got message: %s\n", payload);
  USE_SERIAL.printf("got message: %s\n", length);
}


void setup() {
    USE_SERIAL.begin(115200);

    USE_SERIAL.setDebugOutput(true);

    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();

      for(uint8_t t = 4; t > 0; t--) {
          USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
          USE_SERIAL.flush();
          delay(1000);
      }

    WiFiMulti.addAP("Klein_Net", "Beterrabas012");

    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
    
   }
   USE_SERIAL.printf("connected...");
  if ((WiFi.status() == WL_CONNECTED)) {
USE_SERIAL.printf("CONNECTED!!!");
    WiFiClient client;
    HTTPClient http;

    USE_SERIAL.print("[HTTP] begin...\n");
    // configure traged server and url
    http.begin(client, "http://" SERVER_IP "/authentication"); //HTTP
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Accept", "application/json");

    USE_SERIAL.print("[HTTP] POST...\n");
    // start connection and send HTTP header and body
    //int httpCode = http.POST("{\"hello\":\"world\"}");
    int httpCode = http.POST("{\"strategy\":\"local\",\"email\":\"2@2.2\",\"password\":\"123\"}");

    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      USE_SERIAL.printf("[HTTP] POST... code: %d\n", httpCode);

      // file found at server
      //if (httpCode == HTTP_CODE_OK) {
        const String& payload = http.getString();
        USE_SERIAL.println("received payload:\n<<");
        USE_SERIAL.println(payload);
        USE_SERIAL.println(">>");
      //}
    } else {
      USE_SERIAL.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  }
    
    USE_SERIAL.printf("HTTP END!");
    webSocket.on("event", event);
    webSocket.on("connect", event);
    webSocket.on("connection", event);
  webSocket.on("login", event);
  webSocket.on("messages/created", event);
  //webSocket.on("state_change_request", event);*/
  USE_SERIAL.printf("before WS begin...");
    webSocket.begin("192.168.0.101",3030, "/socket.io/?transport=websocket"); ///socket.io/?transport=websocket
    //webSocket.begin(host, port, path);
    // use HTTP Basic Authorization this is optional remove if not needed
    USE_SERIAL.printf("before setAuthorization...");
    webSocket.setAuthorization("2@2.2", "123");
    USE_SERIAL.printf("entering loop...");
}

void loop() {
    webSocket.loop();
    delay(2000);
}
