#include <MQTT.h>
#include <IotWebConf.h>
#include <Adafruit_MAX31865.h>
#include <pt100rtd.h>

// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
const char thingName[] = "pt100ThermoThing";

// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "password";

#define STRING_LEN 128
#define NUMBER_LEN 128

// -- Configuration specific key. The value should be modified if config structure was changed.
#define CONFIG_VERSION "mqt1"

// -- When CONFIG_PIN is pulled to ground on startup, the Thing will use the initial
//      password to build an AP. (E.g. in case of lost password)
#define CONFIG_PIN 15

// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
#define STATUS_PIN 2

// -- Cable select pins for up to 4 thermometers.
int THERMO_CS_PINS[] = {4, 25, 26, 27};

// -- Callback method declarations.
void wifiConnected();
void configSaved();
boolean formValidator();

DNSServer dnsServer;
WebServer server(80);
HTTPUpdateServer httpUpdater;
WiFiClient net;
MQTTClient mqttClient;

char mqttServerValue[STRING_LEN];
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];
char mqttTopicValue[STRING_LEN];
char thermoNumberValue[NUMBER_LEN];
char thermoWiresValue[NUMBER_LEN];
char updateFreqValue[NUMBER_LEN];

IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);
IotWebConfParameter thermoNumberParam = IotWebConfParameter("Number of sensors", "thermoNumber", thermoNumberValue, NUMBER_LEN, "1", "1..4", NULL, "min='1' max='4' step='1'");
IotWebConfParameter thermoWiresParam = IotWebConfParameter("Number of sensor wires connected", "thermoWires", thermoWiresValue, NUMBER_LEN, "3", "2..4", NULL, "min='2' max='4' step='1'");
IotWebConfParameter updateFreqParam = IotWebConfParameter("MQTT update frequency (sec)", "updateFreq", updateFreqValue, NUMBER_LEN, "60", "15..86400", NULL, "min='15' max='86400' step='1'");
IotWebConfParameter mqttServerParam = IotWebConfParameter("MQTT server", "mqttServer", mqttServerValue, STRING_LEN);
IotWebConfParameter mqttUserNameParam = IotWebConfParameter("MQTT user", "mqttUser", mqttUserNameValue, STRING_LEN);
IotWebConfParameter mqttUserPasswordParam = IotWebConfParameter("MQTT password", "mqttPass", mqttUserPasswordValue, STRING_LEN, "password");
IotWebConfParameter mqttTopicParam = IotWebConfParameter("MQTT topic", "mqttTopic", mqttTopicValue, STRING_LEN, "homeauto/thermoThing");

boolean needMqttConnect = false;
boolean needReset = false;
int pinState = HIGH;
unsigned long lastReport = 0;
unsigned long lastMqttConnectionAttempt = 0;
Adafruit_MAX31865 thermo[] = {Adafruit_MAX31865(THERMO_CS_PINS[0]), 
                              Adafruit_MAX31865(THERMO_CS_PINS[1]),
                              Adafruit_MAX31865(THERMO_CS_PINS[2]),
                              Adafruit_MAX31865(THERMO_CS_PINS[3])};

// The value of the Rref resistor. Use 430.0 for PT100
#define RREF      430.0

// init the Pt100 table lookup module
pt100rtd PT100 = pt100rtd() ;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting up...");

  iotWebConf.setStatusPin(STATUS_PIN);
  iotWebConf.setConfigPin(CONFIG_PIN);
  iotWebConf.addParameter(&updateFreqParam);
  iotWebConf.addParameter(&mqttServerParam);
  iotWebConf.addParameter(&mqttUserNameParam);
  iotWebConf.addParameter(&mqttUserPasswordParam);
  iotWebConf.addParameter(&mqttTopicParam);
  iotWebConf.addParameter(&thermoWiresParam);
  iotWebConf.addParameter(&thermoNumberParam);
  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.setWifiConnectionCallback(&wifiConnected);
  iotWebConf.setupUpdateServer(&httpUpdater);

  // -- Initializing the configuration.
  boolean validConfig = iotWebConf.init();
  if (!validConfig)
  {
    updateFreqValue[0] = '\0';
    mqttServerValue[0] = '\0';
    mqttUserNameValue[0] = '\0';
    mqttUserPasswordValue[0] = '\0';
    mqttTopicValue[0] = '\0';
    thermoNumberValue[0] = '\0';
    thermoWiresValue[0] = '\0';
  }

  // -- Set up required URL handlers on the web server.
  server.on("/", handleRoot);
  server.on("/config", [] { iotWebConf.handleConfig(); });
  server.on("/json", [] { handleJson(); });
  server.onNotFound([]() {
    iotWebConf.handleNotFound();
  });

  mqttClient.begin(mqttServerValue, net);

  int thermoNumber = atoi(thermoNumberValue);
  int thermoWires = atoi(thermoWiresValue);
  for (int i=0; i < thermoNumber; i++) {
    // Use hardware SPI, just pass the CS pin
    switch(thermoWires) {
      case 2:
        thermo[i].begin(MAX31865_2WIRE);
        break;
      case 3:
        thermo[i].begin(MAX31865_3WIRE);
        break;
      case 4:
        thermo[i].begin(MAX31865_4WIRE);
        break;
      default:
        Serial.print("Cannot initialize, invalid thermoWires value: "); Serial.println(thermoWires);
        break;
    }
  }

  Serial.println("Ready.");
}

void loop()
{
  // -- doLoop should be called as frequently as possible.
  iotWebConf.doLoop();
  mqttClient.loop();

  if (needMqttConnect)
  {
    if (connectMqtt())
    {
      needMqttConnect = false;
    }
  }
  else if ((iotWebConf.getState() == IOTWEBCONF_STATE_ONLINE) && (!mqttClient.connected()))
  {
    Serial.println("MQTT reconnect");
    connectMqtt();
  }

  if (needReset)
  {
    Serial.println("Rebooting after 1 second.");
    iotWebConf.delay(1000);
    ESP.restart();
  }

  unsigned long now = millis();
  char fullTopic[STRING_LEN];
  int thermoNumber = atoi(thermoNumberValue);
  unsigned long updateFreq = atoi(updateFreqValue) * 1000;
  if (updateFreq < now - lastReport)
  {
    for (int i=0; i < thermoNumber; i++) {
      uint16_t rtd = thermo[i].readRTD();
      float temperature = rtd_to_celsius(rtd);
      Serial.print("Temperature = "); Serial.println(temperature);
      uint8_t fault = thermo[i].readFault();
      if (fault) {
        Serial.print("Fault 0x"); Serial.println(fault, HEX);
        thermo[i].clearFault();
      } else {
        sprintf(fullTopic,"%s/%i", mqttTopicValue, i+1);
        Serial.print("Sending on MQTT channel '"); Serial.print(fullTopic); Serial.print("': "); Serial.println(temperature);
        mqttClient.publish(fullTopic, String(temperature).c_str());
      }
    }

    lastReport = now;
  }
}

/**
   Handle web requests to "/" path.
*/
void handleRoot()
{
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  String s = "<!DOCTYPE html><html lang=\"en\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "<title>thermoThing Web Conf</title></head><body>thermoThing - PT100 to MQTT";
  s += "<ul>";
  s += "<li>MQTT server: ";
  s += mqttServerValue;
  s += "</ul>";
  s += "Go to <a href='config'>configure page</a> to change values.";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}


void handleJson()
{
  Serial.println("Query JSON data via HTTP.");

  String s = "{\n";
  s += "  \"thingName\": \""; s += thingName; s += "\",\n";
  s += "  \"wiresPerSensor\": \""; s += thermoWiresValue; s += "\",\n";
  s += "  \"address\": \""; s += WiFi.localIP().toString(); s += "\",\n";
  s += "  \"sensor\":{\n";

  int thermoNumber = atoi(thermoNumberValue);
  for (int i=0; i < thermoNumber; i++) {
    s += "    \""; s += (i+1); s += "\":{\n";
    uint16_t rtd = thermo[i].readRTD();
    float temperature = rtd_to_celsius(rtd);
    uint8_t fault = thermo[i].readFault();
    if (fault) {
      s += "      \"state\": \"fault\",\n";
      s += "      \"fault\": \"0x"; s += String(fault, HEX); s += "\"\n";
      thermo[i].clearFault();
    } else {
      s += "      \"state\": \"ok\",\n";
      s += "      \"temperature\": \""; s += temperature; s += "\"\n";
    }
    if (i==(thermoNumber - 1)) {
      s += "    }\n";
    } else {
      s += "    },\n";
    }
  }
  s += "  }\n";
  s += "}\n";
  server.send(200, "application/json", s);
}

void wifiConnected()
{
  needMqttConnect = true;
}

void configSaved()
{
  Serial.println("Configuration was updated.");
  needReset = true;
}

boolean formValidator()
{
  Serial.println("Validating form.");
  boolean valid = true;

  int l = server.arg(mqttServerParam.getId()).length();
  if (l < 3)
  {
    mqttServerParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }

  return valid;
}

boolean connectMqtt() {
  unsigned long now = millis();
  if (1000 > now - lastMqttConnectionAttempt)
  {
    // Do not repeat within 1 sec.
    return false;
  }
  Serial.println("Connecting to MQTT server...");
  if (!connectMqttOptions()) {
    lastMqttConnectionAttempt = now;
    return false;
  }
  Serial.println("Connected!");
  return true;
}

boolean connectMqttOptions()
{
  boolean result;
  if (mqttUserPasswordValue[0] != '\0')
  {
    result = mqttClient.connect(iotWebConf.getThingName(), mqttUserNameValue, mqttUserPasswordValue);
  }
  else if (mqttUserNameValue[0] != '\0')
  {
    result = mqttClient.connect(iotWebConf.getThingName(), mqttUserNameValue);
  }
  else
  {
    result = mqttClient.connect(iotWebConf.getThingName());
  }
  return result;
}

float rtd_to_celsius(uint16_t rtd) {
  uint16_t ohmsx100;
  uint32_t dummy;
  float Tlut;

  // Use uint16_t (ohms * 100) since it matches data type in lookup table.
  dummy = ((uint32_t)(rtd << 1)) * 100 * ((uint32_t) floor(RREF)) ;
  dummy >>= 16 ;
  ohmsx100 = (uint16_t) (dummy & 0xFFFF) ;
  Tlut  = PT100.celsius(ohmsx100);
  return Tlut;
}
