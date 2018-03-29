#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

#include <ESP8266WebServer.h>

// Alternativa a WifiManager
// https://github.com/chriscook8/esp-arduino-apboot/blob/master/ESP-wifiboot.ino
#include <WiFiManager.h>          

#include <WiFiClient.h>
#include <ESP8266HTTPUpdateServer.h>

#include <PubSubClient.h>

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <ESP8266mDNS.h>

#define PARAM_LENGTH 15

const char* CONFIG_FILE   = "/config.json";

/* Possible switch states */
const char STATE_OFF      = '0';
const char STATE_ON       = '1';
const String MODULE_TYPE  = "ligthSwitchStation";
const String CHANNEL_TYPE = "lightSwitch";

struct Channel {
  String name;
  uint8_t switchPin;
  int switchState;
  uint8_t relayPin;
  char relayState;
};

#ifdef NODEMCUV2
Channel channels[] = {
  {"", D7, 0, D1, STATE_OFF},
  {"", D6, 0, D2, STATE_OFF},
  {"", D0, 0, D4, STATE_OFF}
};
const uint8_t MAX_CHANNELS = 3;
#else
Channel channels[] = {
  {"", 13, 0, 5, STATE_OFF},
  {"", 12, 0, 4, STATE_OFF},
  {"", 16, 0, 2, STATE_OFF}
};
const uint8_t MAX_CHANNELS = 3;
#endif

WiFiClient espClient;
PubSubClient mqttClient(espClient);
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

WiFiManagerParameter mqttServer("mqttServer", "MQTT Server", "192.168.0.105", 16);
WiFiManagerParameter mqttPort("mqttPort", "MQTT Port", "1883", 6);
WiFiManagerParameter moduleLocation("moduleLocation", "Module location", "room", PARAM_LENGTH);
WiFiManagerParameter moduleName("moduleName", "Module name", "ceiling", PARAM_LENGTH);
WiFiManagerParameter ch_A_name("ch_A_name", "Channel A name", "ch_A", PARAM_LENGTH);
WiFiManagerParameter ch_B_name("ch_B_name", "Channel B name", "ch_B", PARAM_LENGTH);
WiFiManagerParameter ch_C_name("ch_C_name", "Channel C name", "ch_C", PARAM_LENGTH);

long nextBrokerConnAtte = 0;

template <class T> void log (T text) {
  if (LOGGING) {
    Serial.print("*SW: ");
    Serial.println(text);
  }
}

template <class T, class U> void log (T key, U value) {
  if (LOGGING) {
    Serial.print("*SW: ");
    Serial.print(key);
    Serial.print(": ");
    Serial.println(value);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  log("Starting module");
  bool existConfig = loadConfig();
    
  // pins settings
  for (size_t i = 0; i < MAX_CHANNELS; ++i) {
    pinMode(channels[i].relayPin, OUTPUT);
    pinMode(channels[i].switchPin, INPUT);
    digitalWrite(channels[i].switchPin, HIGH);
  }
  
  // WiFi Manager Config  
  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setStationNameCallback(getStationName);
  wifiManager.setMinimumSignalQuality(WIFI_MIN_SIGNAL);
  if (existConfig) {
    wifiManager.setConnectTimeout(WIFI_CONN_TIMEOUT);
  } else {
    // If no previous config, no reason to try to connect to saved network. Wifi.diconnect() erases saved credentials
    WiFi.disconnect();
  }
  wifiManager.addParameter(&mqttServer);
  wifiManager.addParameter(&mqttPort);
  wifiManager.addParameter(&moduleLocation);
  wifiManager.addParameter(&moduleName);
  wifiManager.addParameter(&ch_A_name);
  wifiManager.addParameter(&ch_B_name);
  wifiManager.addParameter(&ch_C_name);

  if (!wifiManager.autoConnect(("ESP_" + String(ESP.getChipId())).c_str(), "12345678")) {
    log(F("Failed to connect and hit timeout"));
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
  log(F("Connected to wifi network. Local IP"), WiFi.localIP());
  log(F("Configuring MQTT broker"));
  String port = String(mqttPort.getValue());
  log(F("Port"), port);
  log(F("Server"), mqttServer.getValue());
  mqttClient.setServer(mqttServer.getValue(), (uint16_t) port.toInt());
  mqttClient.setCallback(receiveMqttMessage);

  // Set channels
  if (ch_A_name.getValueLength() > 0) {
    channels[0].name = ch_A_name.getValue();
  }
  if (ch_B_name.getValueLength() > 0) {
    channels[1].name = ch_B_name.getValue();
  }
  if (ch_C_name.getValueLength() > 0) {
    channels[2].name = ch_C_name.getValue();
  }

  // OTA Update Stuff
  WiFi.mode(WIFI_STA);
  MDNS.begin(getStationName());
  MDNS.addService("http", "tcp", 80);
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  Serial.print(F("HTTPUpdateServer ready! Open http://"));
  Serial.print(WiFi.localIP().toString());
  Serial.println(F("/update in your browser"));
}

bool loadConfig() { 
  //read configuration from FS json
  if (SPIFFS.begin()) {
    if (SPIFFS.exists(CONFIG_FILE)) {
      //file exists, reading and loading
      File configFile = SPIFFS.open(CONFIG_FILE, "r");
      if (configFile) {
        size_t size = configFile.size();
        if (size > 0) {
          // Allocate a buffer to store contents of the file.
          char buf[size];
          configFile.readBytes(buf, size);
          DynamicJsonBuffer jsonBuffer;
          JsonObject& json = jsonBuffer.parseObject(buf);
          json.printTo(Serial);
          if (json.success()) {
            mqttServer.update(json[mqttServer.getID()]);
            mqttPort.update(json[mqttPort.getID()]);
            moduleName.update(json[moduleName.getID()]);
            moduleLocation.update(json[moduleLocation.getID()]);
            ch_A_name.update(json[ch_A_name.getID()]);
            ch_B_name.update(json[ch_B_name.getID()]);
            ch_C_name.update(json[ch_C_name.getID()]);
            return true;
          } else {
            log(F("Failed to load json config"));
          }
        } else {
          log(F("Config file empty"));
        }
      } else {
        log(F("No config file found"));
      }
      configFile.close();
    } else {
      log(F("No config file found"));
    }
  } else {
    log(F("Failed to mount FS"));
  }
  return false;
}

/** callback notifying the need to save config */
void saveConfigCallback () {
  File configFile = SPIFFS.open(CONFIG_FILE, "w");
  if (configFile) {
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    //TODO Trim param values
    json[mqttServer.getID()] = mqttServer.getValue();
    json[mqttPort.getID()] = mqttPort.getValue();
    json[moduleName.getID()] = moduleName.getValue();
    json[moduleLocation.getID()] = moduleLocation.getValue();
    json[ch_A_name.getID()] = ch_A_name.getValue();
    json[ch_B_name.getID()] = ch_B_name.getValue();
    json[ch_C_name.getID()] = ch_C_name.getValue();
    json.printTo(configFile);
  } else {
    log(F("Failed to open config file for writing"));
  }
  configFile.close();
}

String* trimParamValue(String *p) {
  p->trim();
  return p;
}

void loop() {
  httpServer.handleClient();
  processPhysicalInput();
  if (!mqttClient.connected()) {
    connectBroker();
  }
  mqttClient.loop();
}

void receiveMqttMessage(char* topic, unsigned char* payload, unsigned int length) {
  log(F("mqtt message"), topic);
  for (size_t i = 0; i < MAX_CHANNELS; ++i) {
    if (isChannelEnabled(&channels[i])) {
      if (getChannelTopic(&channels[i], "cmd").equals(topic)) {
        processCommand(&channels[i], payload, length);
        return;
      }
    }
  }
  if (String(topic).equals(getStationTopic("rst"))) {
     reset();
  } else if (String(topic).equals(getStationTopic("hrst"))) {
    hardReset();
  } else if (String(topic).equals(getStationTopic("rtt"))) {
    restart();
  } else {
    log(F("Unknown topic"));
  }
}

void hardReset () {
  log(F("Doing a module hard reset"));
  SPIFFS.format();
  delay(200);
  reset();
}

void reset () {
  log(F("Reseting module configuration"));
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  delay(200);
  restart();
}

void restart () {
  log(F("Restarting module"));
  ESP.restart();
  delay(2000);
}

void publishState (Channel *c) {
  mqttClient.publish(getChannelTopic(c, "state").c_str(), new char[2]{c->relayState, '\0'});
}

void processCommand (Channel *c, unsigned char* payload, unsigned int length) {
  if (length != 1 || !payload) {
    log(F("Invalid payload. Ignoring."));
    return;
  }
  if (!isDigit(payload[0])) {
      log(F("Invalid payload format. Ignoring."));
      return;
  }
  switch (payload[0]) {
    case '0':
    case '1':
      setRelayState(c, payload[0]);
    break;
    default:
      log(F("Invalid state"), payload[0]);
    return;
  } 
  publishState(c);
}

void processPhysicalInput() {
  for (size_t i = 0; i < MAX_CHANNELS; ++i) {
    if (isChannelEnabled(&channels[i])) {
      processChannelInput(&channels[i]);
    }
  }
}

void processChannelInput(Channel *c) {
  int read = digitalRead(c->switchPin);
  if (read != c->switchState) {
    log(F("Phisical switch state has changed. Updating module"), c->name);
    c->switchState = read;
    flipRelayState(c);
    publishState(c);
  }
}

bool isChannelEnabled (Channel *c) {
  return c->name.length() > 0;
}

void setRelayState (Channel *c, char newState) {
  if (c->relayState != newState) {
    flipRelayState(c);
  } else {
    log("Same state to channel, no update done", c->name);
  }
}

void flipRelayState (Channel *c) {
  c->relayState = c->relayState == STATE_OFF ? STATE_ON : STATE_OFF;
  switch (c->relayState) {
    case STATE_OFF:
      digitalWrite(c->relayPin, LOW);
      break;
    case STATE_ON:
      digitalWrite(c->relayPin, HIGH);
      break;
    default:
      break;
  }
  log(F("Channel updated"), c->name);
  log(F("State changed to"), c->relayState);
}

char* getStationName () {
  int size = MODULE_TYPE.length() + moduleLocation.getValueLength() + moduleName.getValueLength() + 4;
  String type(MODULE_TYPE);
  String location(moduleLocation.getValue()); 
  String name(moduleName.getValue());
  char sn[size+1];
  (type + "_" + location + "_" + name).toCharArray(sn, size+1);
  return sn;
}

void connectBroker() {
  if (nextBrokerConnAtte <= millis()) {
    nextBrokerConnAtte = millis() + MQTT_BROKER_CONNECTION_RETRY;
    log(F("Connecting MQTT broker as"), getStationName());
    if (mqttClient.connect(getStationName())) {
      log(F("Connected"));
      subscribeTopic(getStationTopic("#").c_str());
      for (size_t i = 0; i < MAX_CHANNELS; ++i) {
        if (isChannelEnabled(&channels[i])) {
          subscribeTopic(getChannelTopic(&channels[i], "cmd").c_str());
        }
      }
    }
  } else {
    log(F("Failed. RC:"), mqttClient.state());
  }
}

void subscribeTopic(const char *t) {
  log("Subscribing mqtt topic", t);
  mqttClient.subscribe(t);
}

String getChannelTopic (Channel *c, String cmd) {
  return CHANNEL_TYPE + F("/") + String(moduleLocation.getValue()) + F("/") + c->name + F("/") + cmd;
}

String getStationTopic (String cmd) {
  return MODULE_TYPE + F("/") + moduleLocation.getValue() + F("/") + moduleName.getValue() + F("/") + cmd;
}
