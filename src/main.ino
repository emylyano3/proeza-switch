#include <FS.h>              
#include <WiFiClient.h>
#include <ESP8266HTTPUpdateServer.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <ESPConfig.h>

#ifndef ESP01
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <ESP8266mDNS.h>
#endif

const char* CONFIG_FILE   = "/config.json";

/* Possible switch states */
const char    STATE_OFF     = '0';
const char    STATE_ON      = '1';
const String  MODULE_TYPE   = "ligthStation";
const String  CHANNEL_TYPE  = "light";

struct Channel {
  ESPConfigParam *param;
  uint8_t switchPin;
  int switchState;
  uint8_t relayPin;
  char relayState;
};

WiFiClient espClient;
PubSubClient mqttClient(espClient);
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

ESPConfigParam mqttServer     (Text, "mqttServer", "MQTT Server", "192.168.0.105", PARAM_LENGTH, "required");
ESPConfigParam mqttPort       (Text, "mqttPort", "MQTT Port", "1883", 6, "required");
ESPConfigParam moduleLocation (Text, "moduleLocation", "Module location", "room", PARAM_LENGTH, "required");
ESPConfigParam moduleName     (Text, "moduleName", "Module name", "ceiling", PARAM_LENGTH, "required");
#ifndef ESP01
ESPConfigParam ch_A_name(Text, "ch_A_name", "Channel A name", "ch_A", PARAM_LENGTH);
ESPConfigParam ch_B_name(Text, "ch_B_name", "Channel B name", "ch_B", PARAM_LENGTH);
ESPConfigParam ch_C_name(Text, "ch_C_name", "Channel C name", "ch_C", PARAM_LENGTH);
#endif 

#ifdef ESP01
Channel channels[] = {
  {&moduleName, 3, LOW, 2, STATE_OFF}
};
const uint8_t CHANNELS_COUNT  = 1;
const uint8_t TX_PIN          = 1;
#elif NODEMCUV2 
Channel channels[] = {
  {&ch_A_name, D1, LOW, D1, STATE_OFF},
  {&ch_B_name, D2, LOW, D2, STATE_OFF},
  {&ch_C_name, D0, LOW, D4, STATE_OFF}
};
const uint8_t CHANNELS_COUNT  = 3;
const uint8_t LED_PIN         = D7;
#else
Channel channels[] = {
  {&ch_A_name, 13, LOW, 5, STATE_OFF},
  {&ch_B_name, 12, LOW, 4, STATE_OFF},
  {&ch_C_name, 16, LOW, 2, STATE_OFF}
};
const uint8_t CHANNELS_COUNT  = 3;
const uint8_t LED_PIN         = 2;
#endif

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
#ifdef ESP01
  //to avoid using pin 0 as input
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY, TX_PIN);
#else
  Serial.begin(115200);
#endif
  delay(500);
  Serial.println();
  log("Starting module");
    
  // pins settings
  for (size_t i = 0; i < CHANNELS_COUNT; ++i) {
    pinMode(channels[i].relayPin, OUTPUT);
    pinMode(channels[i].switchPin, INPUT);
    digitalWrite(channels[i].switchPin, HIGH);
  }
  
  // WiFi Manager Config  
  ESPConfig moduleConfig;
  moduleConfig.setSaveConfigCallback(saveConfigCallback);
  moduleConfig.setStationNameCallback(getStationName);
  moduleConfig.setMinimumSignalQuality(WIFI_MIN_SIGNAL);
  moduleConfig.setConnectionTimeout(WIFI_CONN_TIMEOUT);
  moduleConfig.addParameter(&mqttServer);
  moduleConfig.addParameter(&mqttPort);
  moduleConfig.addParameter(&moduleLocation);
  moduleConfig.addParameter(&moduleName);
#ifndef ESP01
  for (uint8_t i = 0; i < CHANNELS_COUNT; ++i) {
    moduleConfig.addParameter(channels[i].param);
  }
  moduleConfig.setFeedbackPin(LED_PIN);
#endif
  moduleConfig.setAPStaticIP(IPAddress(10,10,10,10),IPAddress(IPAddress(10,10,10,10)),IPAddress(IPAddress(255,255,255,0)));
  moduleConfig.setPortalSSID("ESP-light-switch");
  moduleConfig.connectWifiNetwork(loadConfig());
  
  log(F("Connected to wifi network. Local IP"), WiFi.localIP());
  log(F("Configuring MQTT broker"));
  String port = String(mqttPort.getValue());
  log(F("Port"), port);
  log(F("Server"), mqttServer.getValue());
  mqttClient.setServer(mqttServer.getValue(), (uint16_t) port.toInt());
  mqttClient.setCallback(receiveMqttMessage);

  // OTA Update Stuff
  WiFi.mode(WIFI_STA);
#ifndef ESP01
  MDNS.begin(getStationName());
  MDNS.addService("http", "tcp", 80);
#endif
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
        #ifndef ESP01
          // Allocate a buffer to store contents of the file.
          char buf[size];
          configFile.readBytes(buf, size);
          DynamicJsonBuffer jsonBuffer;
          JsonObject& json = jsonBuffer.parseObject(buf);
          json.printTo(Serial);
          if (json.success()) {
            mqttServer.update(json[mqttServer.getName()]);
            mqttPort.update(json[mqttPort.getName()]);
            moduleName.update(json[moduleName.getName()]);
            moduleLocation.update(json[moduleLocation.getName()]);
            for (uint8_t i = 0; i < CHANNELS_COUNT; ++i) {
              channels[i].param->update(json[channels[i].param->getName()]);
            }
            return true;
          } else {
            log(F("Failed to load json config"));
          }
        #else
          // Avoid using json to reduce build size
          while (configFile.position() < size) {
            String line = configFile.readStringUntil('\n');
            line.trim();
            uint16_t ioc = line.indexOf('=');
            if (ioc >= 0 && ioc + 1 < line.length()) {
              String key = line.substring(0, ioc++);
              log("Read key", key);
              String val = line.substring(ioc, line.length());
              log("Key value", val);
              if (key.equals(mqttPort.getName())) {
                mqttPort.updateValue(val.c_str());
              } else if (key.equals(mqttServer.getName())) {
                mqttServer.updateValue(val.c_str());
              } else if (key.equals(moduleLocation.getName())) {
                moduleLocation.updateValue(val.c_str());
              } else if (key.equals(moduleName.getName())) {
                moduleName.updateValue(val.c_str());
              } else {
                log("ERROR. Unknown key");
              }
            } else {
              log("Config bad format", line);
            }
          }
          return true;
        #endif
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
  #ifndef ESP01
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    //TODO Trim param values
    json[mqttServer.getName()] = mqttServer.getValue();
    json[mqttPort.getName()] = mqttPort.getValue();
    json[moduleName.getName()] = moduleName.getValue();
    json[moduleLocation.getName()] = moduleLocation.getValue();
    for (uint8_t i = 0; i < CHANNELS_COUNT; ++i) {
      json[channels[i].param->getName()] = channels[i].param->getValue();
    }
    json.printTo(configFile);
  #else
    String line = String(mqttServer.getName()) + "=" + String(mqttServer.getValue());
    configFile.println(line);
    line = String(mqttPort.getName()) + "=" + String(mqttPort.getValue());
    configFile.println(line);
    line = String(moduleName.getName()) + "=" + String(moduleName.getValue());
    configFile.println(line);
  #endif
  } else {
    log(F("Failed to open config file for writing"));
  }
  configFile.close();
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
  log(F("Received mqtt message topic"), topic);
  for (size_t i = 0; i < CHANNELS_COUNT; ++i) {
    if (isChannelEnabled(&channels[i])) {
      if (getChannelTopic(&channels[i], "cmd").equals(topic)) {
        processCommand(&channels[i], payload, length);
        return;
      }
    }
  }
  if (String(topic).equals(getStationTopic("hrst"))) {
    hardReset();
  } else {
    log(F("Unknown topic"));
  }
}

void hardReset () {
  log(F("Doing a module hard reset"));
  SPIFFS.format();
  WiFi.disconnect();
  delay(200);
  ESP.restart();
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
  for (size_t i = 0; i < CHANNELS_COUNT; ++i) {
    if (isChannelEnabled(&channels[i])) {
      processChannelInput(&channels[i]);
    }
  }
}

void processChannelInput(Channel *c) {
  int read = digitalRead(c->switchPin);
  if (read != c->switchState) {
    log(F("Phisical switch state has changed. Updating module"), c->param->getValue());
    c->switchState = read;
    flipRelayState(c);
    publishState(c);
  }
}

bool isChannelEnabled (Channel *c) {
  return c->param->getValueLength() > 0;
}

void setRelayState (Channel *c, char newState) {
  if (c->relayState != newState) {
    flipRelayState(c);
  } else {
    log("Same state to channel, no update done", c->param->getValue());
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
  log(F("Channel updated"), c->param->getValue());
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
      for (size_t i = 0; i < CHANNELS_COUNT; ++i) {
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
  return CHANNEL_TYPE + F("/") + moduleLocation.getValue() + F("/") + c->param->getValue() + F("/") + cmd;
}

String getStationTopic (String cmd) {
  return MODULE_TYPE + F("/") + moduleLocation.getValue() + F("/") + moduleName.getValue() + F("/") + cmd;
}
