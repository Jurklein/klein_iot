#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include <SocketIoClient.h>

#define USE_SERIAL Serial

ESP8266WiFiMulti WiFiMulti;
SocketIoClient webSocket;

void event(const char * payload, size_t length) {
  USE_SERIAL.printf("got message: %s\n", payload);
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
    USE_SERIAL.println("Connected to Wi-Fi.");
    webSocket.on("connect", event);
    webSocket.on("news", event);
    webSocket.on("customEvent", event);
    //webSocket.begin("my.socket-io.server");
    webSocket.begin("192.168.0.101",3030, "/socket.io/?transport=websocket"); ///socket.io/?transport=websocket
    // use HTTP Basic Authorization this is optional remove if not needed
    //webSocket.setAuthorization("username", "password");
    USE_SERIAL.println("entering loop...");
}

void loop() {
    webSocket.loop();
    //delay(5000);
}
