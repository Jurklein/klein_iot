//kurokaji

#include <Arduino.h>

#include <ESP8266WiFi.h>
// Begin: OTA libraries
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
// End: OTA libraries
#include <ESP8266WiFiMulti.h>

#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <Hash.h>

#include "FS.h"

//Begin: HTTP Client
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
//End: HTTP Client

ESP8266WiFiMulti WiFiMulti;
SocketIOclient socketIO;

#define USE_SERIAL Serial

/*
#ifndef STASSID
#define STASSID "Klein_Net"
#define STAPSK  "Beterrabas012"
#endif

const char* ssid = STASSID;
const char* password = STAPSK;
*/



/*
template <typename T, size_t N>
bool isArray(const T (&arr)[N]) {
    return true;
}

template <typename T>
bool isArray(const T& ) {
   return false;
}
*/

#define _DEFAULT "default"

#define ABRIR_PORTAL "abrir"
#define FECHAR_PORTAL "fechar"
#define PORTAL_ABERTO "portal aberto"
#define PORTAL_FECHADO "portal fechado"
#define PORTAL_SENSOR_CONFLICT "conflito de sinal nos sensores de fim de curso (ambos em 1)"
#define PORTAL_PARCIAL "portal parcialmente aberto"
#define PORTAL_ABRINDO "abrindo portal"
#define PORTAL_FECHANDO "fechando portal"
#define PORTAL_ACT_TOO_FAST "nao se afobe ao acionar o portal"
#define PORTAL_PIN  D7
#define PORTAL_ABERTO_PIN D1
#define PORTAL_FECHADO_PIN D6
#define PORTAL_FECHOU_LAST_DIR false
#define PORTAL_ABRIU_LAST_DIR true
#define TOO_LOW_DISPLACEMENT "o deslocamento solicitado eh muito proximo a posicao atual do portal"
#define NEGATIVE_ABSOLUTE_DISPLACEMENT "o valor de um deslocamento absoluto nao pode ser negativo"

#define REGAR_HORTA "regar"
#define REGA_INICIADA "rega iniciada"
#define REGA_FINALIZADA "rega finalizada"
#define REGANDO_HORTA "horta ja sendo regada"
#define FECHAR_TORNEIRA "parar"
#define TORNEIRA_FECHADA "torneira ja fechada"


#define REGA_PADRAO 10
#define HORTA_PIN D2
#define MAX_REGA_TIME 10000

boolean is_portal_open = false;
boolean is_portal_closed = false;
boolean portal_sensor_conflict = false;
boolean portal_command_conflict = false;
boolean portal_changed = false;
boolean is_portal_opening = false; //n usado
boolean is_portal_closing = false; //n usado
unsigned long portal_min_command_time = 2000; //2 seconds minimum between two consecutive gate_actuations
unsigned long portal_command_time = 0;

boolean portal_act_time_threshold_reached = false;
boolean portal_last_dir = PORTAL_FECHOU_LAST_DIR;
boolean portal_dir_shift_lock = false;
boolean portal_static = false;

// Portal_Act_Watchdog: Begin
boolean portal_act_time_recalc_eligible = false;
unsigned long portal_total_time = 10000;
unsigned long portal_curr_time = 0;
float portal_act_percentage = 1.0;
unsigned long portal_curr_displ = 0;
float portal_displ_threshold = 0.1;
boolean portal_act_watchdog_on = false; // Indica ao portalLoop() se ele deve monitorar o tempo de percurso do portal para acionar quando chegar no tempo pré-definido
// Portal_Act_Watchdog: End

const char* hora_rega = "15:45 de hoje";
boolean is_torneira_open = false;
boolean torneira_just_closed = false;
unsigned long rega_time = 0;
unsigned long rega_time_start = 0;
unsigned long rega_time_end = 0;
unsigned long last_rega_time = MAX_REGA_TIME;

boolean io_connected = false;


void checkPortalState() {
  if(is_portal_opening && !is_portal_closing) { 
    if(digitalRead(PORTAL_FECHADO_PIN)) { is_portal_open = false; is_portal_closed = true; }
    else {
      if(is_portal_closed) { is_portal_closed = false; }
      if(digitalRead(PORTAL_ABERTO_PIN)) { is_portal_open = true; is_portal_opening = false; portal_static = true; if(portal_last_dir != PORTAL_ABRIU_LAST_DIR) portal_dir_shift_lock = true; else portal_changed = true; portal_last_dir = PORTAL_ABRIU_LAST_DIR; }
    }
  } else if(is_portal_closing && !is_portal_opening) { 
    if(digitalRead(PORTAL_ABERTO_PIN)) { is_portal_closed = false; is_portal_open = true; }
    else {
      if(is_portal_open) { is_portal_open = false; } 
      if(digitalRead(PORTAL_FECHADO_PIN)) { is_portal_closed = true; is_portal_closing = false; portal_static = true; if(portal_last_dir != PORTAL_FECHOU_LAST_DIR) portal_dir_shift_lock = true; else portal_changed = true; portal_last_dir = PORTAL_FECHOU_LAST_DIR; }
    }
  } else if(is_portal_opening && is_portal_closing) {
    portal_command_conflict = true; is_portal_opening = false; is_portal_closing = false;
  } else { //!is_portal_opening && !is_portal_closing
    if(digitalRead(PORTAL_ABERTO_PIN)) {
      if(digitalRead(PORTAL_FECHADO_PIN)) portal_sensor_conflict = true;
      else { if(!is_portal_open) { is_portal_open = true; portal_last_dir = PORTAL_ABRIU_LAST_DIR; } is_portal_closed = false; portal_sensor_conflict = false; }
    } else if(digitalRead(PORTAL_FECHADO_PIN)) { if(!is_portal_closed) { is_portal_closed = true; portal_last_dir = PORTAL_FECHOU_LAST_DIR; } is_portal_open = false; portal_sensor_conflict = false; }
    else { is_portal_closed = false; is_portal_open = false; }
  }
}

//Portal_Act_Watchdog: Begin
void portalActWatchdog() {
  if(portal_act_percentage < 1.0) {
    //if(is_portalportal_curr_displ portal_total_time
  }
boolean portal_act_time_recalc_eligible = false;
unsigned long portal_total_time_displ = 10000;
unsigned long portal_curr_time_displ = 0;
unsigned long portal_desired_time_displ = 3.5;
float portal_displ_percentage_threshold = 0.1;
float portal_act_percentage = 1.0;
boolean portal_act_watchdog_on = false;
  
}
//Portal_Act_Watchdog: End

const char* portalState() {
  //checkPortalState();
  if(portal_changed)
    portal_changed = false;
  if(is_portal_opening)  //is_portal_closing / is_portal_opening flags must be checked before is_portal_closed / is_portal_open flags
    return PORTAL_ABRINDO;
  if(is_portal_closing)
    return PORTAL_FECHANDO;
  if(portal_sensor_conflict)
    return PORTAL_SENSOR_CONFLICT;
  if(is_portal_open)
    return PORTAL_ABERTO;
  if(is_portal_closed)
    return PORTAL_FECHADO;
  return PORTAL_PARCIAL;
}

//Portal_Act_Dealer: Begin
const char* portalInteract(const char* action, JsonObject& action_options) {
  const char* displ_reference;
  if(action_options["reference"]) displ_reference = action_options["reference"]; else displ_reference = "absolute";
  const char* displ_amount;
  if(action_options["amount"]) displ_amount = action_options["amount"]; else displ_amount = "1.0";
  portal_act_percentage = strtof(displ_amount, NULL);
  if(portal_act_percentage > 1.0) portal_act_percentage = 1.0;
  else if(portal_act_percentage < -1.0 && displ_reference == "relative") portal_act_percentage = -1.0;
  else if(portal_act_percentage < 0.0 && displ_reference == "absolute") portal_act_percentage = 0.0;
  if(!strcmp(displ_reference,"relative")) {
    if( portal_act_percentage < portal_displ_percentage_threshold ) return TOO_LOW_DISPLACEMENT;
    portal_desired_time_displ = portal_curr_time_displ;
    if(!strcmp(action,ABRIR_PORTAL)) portal_desired_time_displ += portal_act_percentage * portal_total_time_displ;
    else if(!strcmp(action,FECHAR_PORTAL)) portal_desired_time_displ -= portal_act_percentage * portal_total_time_displ;
  } else if(!strcmp(displ_reference,"absolute")) {
    if(portal_act_percentage < 0.0) return NEGATIVE_ABSOLUTE_DISPLACEMENT;
    float curr_percentage = portal_curr_time_displ / portal_total_time_displ;
    if(!strcmp(action,ABRIR_PORTAL)) {
      if( abs(portal_act_percentage - curr_percentage) < portal_displ_percentage_threshold ) return TOO_LOW_DISPLACEMENT;
      portal_desired_time_displ = portal_act_percentage * portal_total_time_displ;
    } else if(!strcmp(action,FECHAR_PORTAL)) {
      if( abs( ( 1.0 - portal_act_percentage ) - curr_percentage) < portal_displ_percentage_threshold ) return TOO_LOW_DISPLACEMENT;
      portal_desired_time_displ = ( 1.0 - portal_act_percentage ) * portal_total_time_displ;
    }
  }
  if( portal_desired_time_displ < portal_displ_percentage_threshold * portal_total_time_displ ) portal_desired_time_displ = 0;
  else if( portal_desired_time_displ > ( 1 - portal_displ_percentage_threshold * portal_total_time_displ ) ) portal_desired_time_displ = portal_total_time_displ;
  if(!strcmp(action,ABRIR_PORTAL)) {
    openPortal(action_options);
  } else if(!strcmp(action,FECHAR_PORTAL)) {
    closePortal(action_options);
  } else if(strcmp(action,_DEFAULT)) return "valor invalido para portal"; 
  return portalState();
}
//Portal_Act_Dealer: End
/*
const char* portalInteract(const char* action, JsonObject& action_options) {
  if(!strcmp(action,ABRIR_PORTAL)) {
    openPortal(action_options);
  } else if(!strcmp(action,FECHAR_PORTAL)) {
    closePortal(action_options);
  } else if(strcmp(action,_DEFAULT)) return "valor invalido para portal"; 
  return portalState();
}
*/

boolean portalActTimeThreshold() {
  if(millis() - portal_command_time > portal_min_command_time) { portal_act_time_threshold_reached = true; return true; }
  portal_act_time_threshold_reached = false;
  return false;
}
boolean openPortal(JsonObject& action_options) {
  if(is_portal_opening || is_portal_open)
    return true;
  if(!portalActTimeThreshold())
    return false;
  if(is_portal_closing) { is_portal_closing = false; portal_changed = true; }
  is_portal_opening = true;
  portal_changed = true;
  checkPortalState();
  if(portal_dir_shift_lock && !portal_static) { portal_dir_shift_lock = false; return true; }
  acionarPortal(action_options);
  return true;
}
boolean closePortal(JsonObject& action_options) {
  if(is_portal_closing || is_portal_closed)
    return true;
  if(!portalActTimeThreshold() || portal_dir_shift_lock)
    return false;
  if(is_portal_opening) { is_portal_opening = false; portal_changed = true; }
  is_portal_closing = true;
  portal_changed = true;
  checkPortalState();
  if(portal_dir_shift_lock && !portal_static) { portal_dir_shift_lock = false; return true; }
  acionarPortal(action_options);
  return true;
}
void acionarPortal(JsonObject& action_options) { // Cuidado: essa função só pode ser chamada quando os requisitos forem cumpridos (portalActTimeThreshold() retorna true)
  if(portal_static) {
    portal_static = false;
    if(portal_last_dir == PORTAL_FECHOU_LAST_DIR)
      portal_last_dir = PORTAL_ABRIU_LAST_DIR;
    else if(portal_last_dir == PORTAL_ABRIU_LAST_DIR)
      portal_last_dir = PORTAL_FECHOU_LAST_DIR;
    if(portal_dir_shift_lock) portal_dir_shift_lock = false;
  } else portal_static = true;
  if( ( is_portal_opening && portal_last_dir == PORTAL_FECHOU_LAST_DIR ) || ( is_portal_closing && portal_last_dir == PORTAL_ABRIU_LAST_DIR ) )
    portal_dir_shift_lock = true;
  if( portal_desired_time_displ > 0 && portal_desired_time_displ < portal_total_time_displ ) portal_act_watchdog_on = true; else portal_act_watchdog_on = false;
  portal_command_time = millis();
  digitalWrite(PORTAL_PIN, HIGH);
  delay(500);
  digitalWrite(PORTAL_PIN, LOW);
}
void portalLoop() {
  checkPortalState();
  if(portal_changed) postMessage(portalState());
  portal_changed = false;
  if(portal_dir_shift_lock && portalActTimeThreshold()) acionarPortal();
  if(portal_act_watchdog_on) portalActWatchdog();
}



void checkHortaState() {  //verifica se houve uma mudança de estado na torneira (para emitir um evento)
  if(digitalRead(HORTA_PIN)) { 
    if(!is_torneira_open) { is_torneira_open = true; /*horta_changed = true;*/ }
  } else if(is_torneira_open) { is_torneira_open = false; torneira_just_closed = true; }
}
const char* hortaState() { //retorna string: se torneira aberta, retorna REGANDO_HORTA; se torneira fechada, retorna hora da última rega
  checkHortaState();
  if(is_torneira_open) return REGANDO_HORTA;
  if(torneira_just_closed) { torneira_just_closed = false; return REGA_FINALIZADA; }
  return hora_rega;
}
void definirTempoRega(String time_interval = "") {
  if(time_interval == "") rega_time = last_rega_time;
  else { rega_time = atol(time_interval.c_str()); last_rega_time = rega_time; }
}
boolean regarHorta(const char* time_interval = "") {
  if(is_torneira_open)
    return false;
  definirTempoRega(time_interval);
  digitalWrite(HORTA_PIN, HIGH);
  rega_time_start = millis();
  checkHortaState();
  return true;
}
boolean terminarRega() {
  if(!is_torneira_open)
    return false;
  digitalWrite(HORTA_PIN, LOW);
  rega_time_end = millis();
  checkHortaState();
  return true;
}
boolean isTempoRegaAtingido() {
  if(!is_torneira_open) return false;
  unsigned long rega_time_interval = millis() - rega_time_start;
  if(rega_time_interval > MAX_REGA_TIME || rega_time_interval > rega_time) {
    terminarRega();
    return true;
  }
  return false;
}
/*
const char* unsLongToCharPointer(unsigned long number) {
  char* opa;
  ultoa(number, opa, 10);
  return opa;
}*/
const char* hortaInteract(const char* action, JsonObject& action_options) {
  if(!strcmp(action,REGAR_HORTA)) {
    if(regarHorta()) return REGA_INICIADA;
    else return REGANDO_HORTA;
  } else if(!strcmp(action,FECHAR_TORNEIRA)) {
    if(terminarRega()) return REGA_FINALIZADA;
    else return TORNEIRA_FECHADA;
  } else if(!strcmp(action,_DEFAULT)) { return hortaState();
  } else {
    if(regarHorta(action)) return REGA_INICIADA,", regando por 20 s";//,unsLongToCharPointer(rega_time);
    else return REGANDO_HORTA;
  }
  return "valor invalido para horta";
}
void regaLoop() {
  if(isTempoRegaAtingido() || torneira_just_closed) postMessage(hortaState());
}

void postMessage(const char* message) {
  // creat JSON message for Socket.IO (event)
        DynamicJsonDocument doc(1024);
        JsonArray array = doc.to<JsonArray>();
        // add evnet name
        // Hint: socket.on('event_name', ....
        array.add("create");
        array.add("messages");

        // add payload (parameters) for the event
        //JsonObject param0 = array.createNestedObject();
        //param0 = "authentication";
        JsonObject param1 = array.createNestedObject();
        param1["text"] = message;
        // JSON to String (serializion)
        String output;
        serializeJson(doc, output);
        // Send event        
        socketIO.sendEVENT(output);
        // Print JSON for debugging
        USE_SERIAL.println(output);
}

void socketIOAuthenticate() {
  // creat JSON message for Socket.IO (event)
        DynamicJsonDocument doc(1024);
        JsonArray array = doc.to<JsonArray>();
        // add evnet name
        // Hint: socket.on('event_name', ....
        array.add("create");
        array.add("authentication");
        // add payload (parameters) for the event
        //JsonObject param0 = array.createNestedObject();
        //param0 = "authentication";
        JsonObject param1 = array.createNestedObject();
        param1["strategy"] = "local";
        //sonObject param2 = array.createNestedObject();
        param1["email"] = "esp_garagem@iot";
        //JsonObject param3 = array.createNestedObject();
        param1["password"] = "123";
        
        // JSON to String (serializion)
        String output;
        serializeJson(doc, output);
        // Send event        
        socketIO.sendEVENT(output);
        // Print JSON for debugging
        USE_SERIAL.println(output);
}

boolean checkHost(const char* address, const char* port) {
  WiFiClient client;
  HTTPClient http;
  Serial.print("[HTTP] begin...\n");
  String host = "http://";
  host += address;
  host += ":";
  host += port;
  if (http.begin(client, host)) {  // HTTP
    //Serial.print("[HTTP] GET...\n");
    // start connection and send HTTP header
    int httpCode = http.GET();
    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] GET... code: %d\n", httpCode);
      // file found at server
      if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
        //String payload = http.getString();
        //Serial.println(payload);
        Serial.print("Server answered: ");
        Serial.println(host);
        http.end();
        return true;
      }
    } else
      Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  Serial.print("[HTTP] Couldn't connect to server ");
  Serial.println(host);
  http.end();
  return false;
}

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            USE_SERIAL.printf("[IOc] Disconnected!\n");
            io_connected = false;
            break;
        case sIOtype_CONNECT:
            USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);
            socketIOAuthenticate();
            io_connected = true;
            break;
        case sIOtype_EVENT:
            //USE_SERIAL.printf("[IOc] get event: %s\n", payload);
            //break;
            {
            //char * sptr = NULL;
            //int id = strtol((char *)payload, &sptr, 10);
            USE_SERIAL.printf("[IOc] get event: %s\n", payload);
            //USE_SERIAL.printf("[IOc] get event: %s id: %d\n", payload, id);
            //if(id) {
            //    payload = (uint8_t *)sptr;
            //}
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, payload, length);
            if(error) {
                USE_SERIAL.print(F("deserializeJson() failed: "));
                USE_SERIAL.println(error.c_str());
                return;
            }
            
            String eventName = doc[0];
            USE_SERIAL.printf("[IOc] event name: %s\n", eventName.c_str());
            // Message Includes a ID for a ACK (callback)
            /*
            if(id) {
                // creat JSON message for Socket.IO (ack)
                DynamicJsonDocument docOut(1024);
                JsonArray array = docOut.to<JsonArray>();
                // add payload (parameters) for the ack (callback function)
                JsonObject param1 = array.createNestedObject();
                param1["now"] = millis();
                // JSON to String (serializion)
                String output;
                output += id;
                serializeJson(docOut, output);
                // Send event        
                socketIO.send(sIOtype_ACK, output);
            }*/
            if(eventName == "commands created") {
              USE_SERIAL.println("AAAAAAAAAAAAAAAAAAAAAMULHEEEEEEEEEEEEEEEEEEEEEEEEKEEEEEEEEE");
              JsonObject data = doc[1];
              int user_id = data["user_id"];
              int target_user_id = data["target_user_id"];
              JsonObject object = data["object"];
              JsonObject horta;
              JsonObject portal;
              if(object["horta"]) { horta = object["horta"]; }
              if(object["portal"]) { portal = object["portal"]; }
              /*JsonObject action = data["action"];
              JsonObject action_detail = data["action_detail"];
              */
              const char* portal_action;
              const char* horta_action;
              if(portal["action"]) portal_action = portal["action"]; else portal_action = "invalid";
              if(horta["action"]) horta_action = horta["action"]; else horta_action = "invalid";
              JsonObject portal_action_options;
              JsonObject horta_action_options;
              if(portal["options"]) portal_action_options = portal["options"]; /*else portal_action_options = "";*/
              if(horta["options"]) horta_action_options = horta["options"]; /*else horta_action_options = "";*/
              /*if(portal != undefined) {
                if(portal == "abrir"
              }*/
              if(horta_action != "invalid") { postMessage(hortaInteract(horta_action, horta_action_options)); torneira_just_closed = false; }
              if(portal_action != "invalid") { postMessage(portalInteract(portal_action, portal_action_options)); portal_changed = false; }
              /*
              const char* current_host;
              int n_hosts = sizeof(doc["host"]);
              for (int i = 0;i <= n_hosts; i++) {
                if(checkHost(host[i],port)) {
                  current_host = doc["host"][i];
                  break;
                }
                if(i == n_hosts) {
                  USE_SERIAL.println("Couldn't connect to any of the hosts listed.");
                }
              }*/

              /*
              USE_SERIAL.println("checando sizeof object[name]:");
              USE_SERIAL.println(sizeof(action["regar"][0]));
              USE_SERIAL.println(sizeof(action["abrir portao"][0]));
              USE_SERIAL.println(sizeof(action["sidjhi"][0]));
              USE_SERIAL.println((sizeof(action["regar"]) / sizeof(action["regar"][0])));
              USE_SERIAL.println((sizeof(action["abrir portao"]) / sizeof(action["abrir portao"][0])));
              USE_SERIAL.println(sizeof((sizeof(action["adsds"]) / sizeof(action["iudhiu"][0]))));
              const char* regare[3] = {0,0,0};
              for (int i = 0; i < 3; i++) {
                regare[i] = action["regar"][i];
              }
              USE_SERIAL.println("REGANDO:");
              //USE_SERIAL.println(action["regar"][0]);
              //USE_SERIAL.printf("%s\n",action["regar"][1]);
              //USE_SERIAL.printf("%s\n",regare);
              //USE_SERIAL.printf("%s\n",regare[0]);
              //USE_SERIAL.printf("%s\n",regare[1]);
              const char* regar = action["regar"];
              const char* abrir_portao = action["abrir portao"];
              const char* opinha;
              if(action["opalele zinho meu"]) opinha = action["opalele zinho meu"]; else opinha = "";
              USE_SERIAL.println("opa:");
              USE_SERIAL.println(opinha);
              USE_SERIAL.println(opinha != "");
              USE_SERIAL.println("Checando sizeof const chars **:");
              USE_SERIAL.println(sizeof(*regar));
              USE_SERIAL.println(sizeof(*abrir_portao));
              USE_SERIAL.println(sizeof(*opinha));
              USE_SERIAL.println("Tentando com strings...");
              String regar2 = action["regar"];
              String abrir_portao2 = action["abrir portao"];
              USE_SERIAL.println("Checando tamanhpo das strings");
              USE_SERIAL.println(regar2);
              USE_SERIAL.println(sizeof(regar2));
//              USE_SERIAL.println(*regar2);
              USE_SERIAL.println(abrir_portao2);
              USE_SERIAL.println(sizeof(abrir_portao2));
  //            USE_SERIAL.println(&abrir_portao2);
              
              USE_SERIAL.println("Checando se é array de elementos *:");
              USE_SERIAL.println(isArray(*regar));
              USE_SERIAL.println(isArray(*abrir_portao));
              USE_SERIAL.println(isArray(*opinha));
              const int x = 2;
              USE_SERIAL.println(isArray(x));
              USE_SERIAL.println("Checando se é array de elementos object[name]:");
              USE_SERIAL.println(isArray(action["regar"]));
              USE_SERIAL.println(isArray(action["abrir_portao"]));
              USE_SERIAL.println(isArray(action["opinha"]));
              if(regar) USE_SERIAL.printf("%s\n",regar);
              if(regar) USE_SERIAL.println(regar);
              //USE_SERIAL.println(sizeof(regar));
              if(sizeof(abrir_portao)) USE_SERIAL.println("Bora openar o gueit");
              //USE_SERIAL.println(sizeof(abrir_portao));
              //USE_SERIAL.println(*abrir_portao);
              //USE_SERIAL.printf(abrir_portao);
              if(opinha != "") USE_SERIAL.println("DEU OPINHA!");
              else USE_SERIAL.println("NAO DEU OPA =(");
              //if(data["gate"])
              */
              //postMessage("IIIIIIIIIIIIAAAAAAAAEEEEEEE PERSONAAAAL");
            }
        }
            break;
        case sIOtype_ACK:
            USE_SERIAL.printf("[IOc] get ack: %u\n", length);
            hexdump(payload, length);
            break;
        case sIOtype_ERROR:
            USE_SERIAL.printf("[IOc] get error: %u\n", length);
            hexdump(payload, length);
            break;
        case sIOtype_BINARY_EVENT:
            USE_SERIAL.printf("[IOc] get binary: %u\n", length);
            hexdump(payload, length);
            break;
        case sIOtype_BINARY_ACK:
            USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
            hexdump(payload, length);
            break;
    }
}











void portalRead() {
  USE_SERIAL.print("Portal Aberto PIN: ");
  USE_SERIAL.println(digitalRead(PORTAL_ABERTO_PIN));
  USE_SERIAL.print("Portal Aberto: ");
  USE_SERIAL.println(is_portal_open);
  USE_SERIAL.print("Portal Fechado PIN: ");
  USE_SERIAL.println(digitalRead(PORTAL_FECHADO_PIN));
  USE_SERIAL.print("Portal Fechado: ");
  USE_SERIAL.println(is_portal_closed);
  USE_SERIAL.print("Is portal Closing? ");
  USE_SERIAL.println(is_portal_closing);
  USE_SERIAL.print("Is portal Opening? ");
  USE_SERIAL.println(is_portal_opening);
}

void setup() {
  pinMode(HORTA_PIN, OUTPUT);
  digitalWrite(HORTA_PIN, LOW);
  pinMode(PORTAL_PIN, OUTPUT);
  digitalWrite(PORTAL_PIN, LOW);
  pinMode(PORTAL_ABERTO_PIN, INPUT);
  pinMode(PORTAL_FECHADO_PIN, INPUT);

  portalRead();
    // USE_SERIAL.begin(921600);
    USE_SERIAL.begin(115200);

    //Serial.setDebugOutput(true);
    USE_SERIAL.setDebugOutput(true);

    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();

      for(uint8_t t = 4; t > 0; t--) {
          USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
          USE_SERIAL.flush();
          delay(1000);
      }

    // always use this to "mount" the filesystem
    bool result = SPIFFS.begin();

    // this opens the file "f.txt" in read-mode
    File f = SPIFFS.open("/config.txt", "r");
char json[f.size()];
  if (!f) {
    Serial.println("File doesn't exist yet. Creating it");

   // open the file in write mode
    File f = SPIFFS.open("/config.txt", "w");
    if (!f)
      Serial.println("file creation failed");
  } else {
    String line;
    // we could open the file
    while(f.available()) {
      //Lets read line by line from the file
      line += f.readStringUntil('\n');
      //Serial.println(line);
    }
    //char json[f.size()];
    line.toCharArray(json,f.size());
  }
  f.close();
  Serial.println("OPALELE:");
  Serial.println(json);
  DynamicJsonDocument doc(sizeof(json)*2);
   // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, json);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  // Fetch values.
  //
  // Most of the time, you can rely on the implicit casts.
  // In other case, you can do doc["time"].as<long>();
  const char* ssid = doc["ssid"];
  const char* password = doc["pass"];
  const char* host_0 = doc["host"][0]; // "192.168.0.100"
  const char* host_1 = doc["host"][1]; // "192.168.0.101"
  const char* host[] = {host_0, host_1};
  const char* port = doc["port"];
  const char* device = doc["device"];
  const char* device_pwd = doc["device_pwd"];

  Serial.println(ssid);
  Serial.println(password);
  Serial.println(host[0]);
  Serial.println(host[1]);
  Serial.println(port);
  Serial.println(device);
  Serial.println(device_pwd);
  
    // disable AP
    if(WiFi.getMode() & WIFI_AP) {
        WiFi.softAPdisconnect(true);
    }

    WiFiMulti.addAP(ssid, password);

    //WiFi.disconnect();
    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
    }

    String ip = WiFi.localIP().toString();
    USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(device);

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

  // NOTE: if updating FS this would be the place to unmount FS using FS.end()
  if (type == "filesystem")
    SPIFFS.end();
  Serial.println("Start updating " + type);
  });
  ArduinoOTA.onStart([]() {
    Serial.println("\nStarting OTA update");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  USE_SERIAL.println("Ready");
  USE_SERIAL.print("IP address: ");
  USE_SERIAL.println(WiFi.localIP());
  // server address, port and URL
  const char* current_host;
  int n_hosts = sizeof(doc["host"]);
  for (int i = 0;i <= n_hosts; i++) {
    if(checkHost(host[i],port)) {
      current_host = doc["host"][i];
      break;
    }
    if(i == n_hosts) {
      USE_SERIAL.println("Couldn't connect to any of the hosts listed.");
    }
  } y
    socketIO.begin(current_host, atoi(port));
    //socketIO.begin("192.168.0.100", 3030);
    // event handler
    socketIO.onEvent(socketIOEvent);
    //socketIO.setAuthorization("2@2.2", "123"); //n funciona usando socketIO, só funciona usando WebSocket
    //socketIO.setExtraHeaders();
}

unsigned long messageTimestamp = 0;
void loop() {
    ArduinoOTA.handle();
    socketIO.loop();
    if(io_connected) {
      regaLoop();
      portalLoop();
      if (millis() - messageTimestamp > 2000) { portalRead(); messageTimestamp = millis(); }
    }
}
