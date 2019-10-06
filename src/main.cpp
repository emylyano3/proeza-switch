#include <ESPDomotic.hpp>

void processInput();
bool channelHandler(const String& range, const String& value);

#ifdef ESP01
// usable pins GPIO2 (GPIO3 if using SERIAL_TX_ONLY)
const uint8_t SWITCH_PIN  = 2;
const uint8_t RELAY_PIN   = 0;
const uint8_t TX_PIN      = 1;
#elif NODEMCUV2
// usable pins D0,D1,D2,D5,D6,D7 (D10 is TX (GPIO1), D9 is RX (GPIO3), D3 is GPIO0, D4 is GPIO2, D8 is GPIO15)
const uint8_t SWITCH_PIN  = D5;
const uint8_t RELAY_PIN   = D1;
const uint8_t FEEDBACK_PIN     = D7;
#else
// usable pins GPIO 4,5,12,13,14,16
const uint8_t SWITCH_PIN  = 4;
const uint8_t RELAY_PIN   = 5;
const uint8_t FEEDBACK_PIN     = 12;
#endif

Channel _light ("light", "Light", RELAY_PIN, OUTPUT, HIGH);
Channel _switch ("switch", "Switch", SWITCH_PIN, INPUT, LOW);

ESPDomotic  _domoticModule;

void setup() {
#ifdef ESP01
  //to avoid using pin 0 as input
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY, TX_PIN);
#else
  Serial.begin(115200);
#endif
  delay(500);
  Serial.println();
  _switch.state = digitalRead(_switch.pin);
  debug("Starting module");
  _light.property()
    ->setId("on")
    ->setName("On")
    ->setDataType("boolean")
    ->setRetained(true)
    // ->settable(channelHandler)
    ;
  String ssid = "Light switch " + String(ESP.getChipId());
  _domoticModule.setPortalSSID(ssid.c_str());
  #ifndef ESP01
  _domoticModule.setFeedbackPin(FEEDBACK_PIN);
  #endif
  _domoticModule.setConfigPortalTimeout(90);
  _domoticModule.setWifiConnectTimeout(45);
  _domoticModule.setConfigFileSize(256);
  _domoticModule.setModuleType("switch");
  _domoticModule.setFirmware("homiev3", "v0.1");
  _domoticModule.addChannel(&_light);
  _domoticModule.addChannel(&_switch);
  _domoticModule.init();
}

bool channelHandler(const String& range, const String& value) {
  Serial.printf("channelHandler range %s value %s", range, value);
  return false;
}

void loop() {
  _domoticModule.loop();
  processInput();
}

void processInput() {
  int read = digitalRead(_switch.pin);
  if (read != _switch.state) {
    debug(F("Switch state has changed"));
    _switch.state = read;
    _light.state = _light.state == LOW ? HIGH : LOW;
    digitalWrite(_light.pin, _light.state);
    //TODO Aca hacer solo la notificacion al nodo y que el envio se se resuelva dentro de la logica de homie
    // _domoticModule.getMqttClient()->publish(_domoticModule.getChannelTopic(&_light, "feedback/state").c_str(), _light.state == LOW ? "1" : "0");
    debug(F("Output channel state changed to"), _light.state == LOW ? "ON" : "OFF");
  }
}