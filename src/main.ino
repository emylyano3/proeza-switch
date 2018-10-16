#include <FS.h>
#include <ESPDomotic.h>

#ifdef ESP01
Channel _switch ("A", "Switch", 3, INPUT, HIGH, -1);
Channel _relay ("B", "Relay", 2, OUTPUT, HIGH, -1);
const uint8_t TX_PIN      = 1;
#elif NODEMCUV2
Channel _switch ("A", "Switch", D1, INPUT, HIGH, -1);
Channel _relay ("B", "Relay", D2, OUTPUT, HIGH, -1);
const uint8_t LED_PIN     = D3;
#else
Channel _switch ("A", "Switch", 1, INPUT, HIGH, -1);
Channel _relay ("B", "Relay", 2, OUTPUT, HIGH, -1);
const uint8_t LED_PIN     = 3;
#endif

template <class T> void log (T text) {
  #ifdef LOGGING
  Serial.print("*SW: ");
  Serial.println(text);
  #endif
}

template <class T, class U> void log (T key, U value) {
  #ifdef LOGGING
  Serial.print("*SW: ");
  Serial.print(key);
  Serial.print(": ");
  Serial.println(value);
  #endif
}

ESPDomotic _domoticModule;

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
  String ssid = "Light switch " + String(ESP.getChipId());
  _domoticModule.setPortalSSID(ssid.c_str());
  #ifndef ESP01
  _domoticModule.setFeedbackPin(LED_PIN);
  #endif
  _domoticModule.setMqttConnectionCallback(mqttConnectionCallback);
  _domoticModule.setMqttMessageCallback(receiveMqttMessage);
  _domoticModule.setModuleType("light");
  _domoticModule.addChannel(&_switch);
  _domoticModule.addChannel(&_relay);
  _domoticModule.init();
}

void loop() {
  _domoticModule.loop();
  processPhysicalInput();
}

void processPhysicalInput() {
  for (size_t i = 0; i < _domoticModule.getChannelsCount(); ++i) {
    Channel *c = _domoticModule.getChannel(i);
    if (c->pinMode == INPUT && c->isEnabled()) {
      // Just process those channels that are intented to act as input. e.g. a light switch
      processChannelInput(c);
    }
  }
}

void processChannelInput(Channel *c) {
  int read = digitalRead(c->pin);
  if (read != c->state) {
    log(F("Physical switch state has changed. Updating module"), c->name);
    digitalWrite(c->pin, c->state = read);
    log(F("Channel updated"), c->name);
    log(F("State changed to"), "" + c->state);
    _domoticModule.getMqttClient()->publish(_domoticModule.getChannelTopic(c, "feedback/state").c_str(), c->state == LOW ? "1" : "0");
  }
}

void mqttConnectionCallback() {
  // no additional subsription is needed
}

void receiveMqttMessage(char* topic, uint8_t* payload, unsigned int length) {
  // no additional message to process
}