// Термодатчики
#include <OneWire.h>
#include <DallasTemperature.h>

// настройки порта для работы с термодатчиками
#define ONE_WIRE_BUS 8
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// До 8-ми датчиков температуры.
// Т.к. используется бойлер косвенного нагрева, планируется контролировать:
// - подача-обратка котла
// - подача-обратка отопительного контура
// - подача-обратка ГВС
// - температура в бойлерной
// - температура на улице
#define THERMO_MAX_CNT 2
char thermCnt = 0;
DeviceAddress insideThermometer[THERMO_MAX_CNT];

// Периодическое обновление
long lastUpdate = 0;

/// -------------------- MQTT ---------------------------
#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial

#ifndef __AVR_ATmega328P__
#define SerialAT Serial1

// or Software Serial on Uno, Nano
#else
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(11, 10);  // RX, TX
#endif

#define TINY_GSM_DEBUG SerialMon

// Your GPRS credentials, if any
const char apn[]      = "internet.mts.ru";
const char gprsUser[] = "mts";
const char gprsPass[] = "mts";

// MQTT details
const char * broker = "dev.rightech.io";
const char * connect_to = "mqtt-a1506-kl6xda";
const char * topic = "base/state/temperature";

#include <TinyGsmClient.h>
#include <PubSubClient.h>

#define PUB_DELAY (5 * 1000) /* 5 seconds */

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

uint32_t lastReconnectAttempt = 0;

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.println(topic);
  SerialMon.write(payload, len);
  SerialMon.println();
}

boolean mqttConnect() {
  SerialMon.print(F("Connecting to "));
  SerialMon.print(broker);

  // Connect to MQTT Broker
  boolean status = mqtt.connect(connect_to);

  if (status == false) {
    SerialMon.println(F(" fail"));
    return false;
  }
  SerialMon.println(F(" success"));

  // mqtt.subscribe("base/relay/led1");
  return mqtt.connected();
}

void setup(void)
{
  // контроль наличия внешнего питания
  pinMode(6, INPUT);
  // start serial port
  SerialMon.begin(9600);
  delay(10);

  SerialMon.println("Wait...");

  // Set GSM module baud rate
  SerialAT.begin(38400);
  delay(6000);

    // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println(F("Initializing modem..."));
  // modem.restart();
  modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print(F("Modem Info: "));
  SerialMon.println(modemInfo);

  SerialMon.print(F("Waiting for network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(F(" fail"));
    delay(10000);
    return;
  }
  SerialMon.println(F(" success"));

  if (modem.isNetworkConnected()) {
    SerialMon.println(F("Network connected"));
  }

  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(F(" fail"));
    delay(10000);
    return;
  }
  SerialMon.println(F(" success"));

  if (modem.isGprsConnected()) {
    SerialMon.println(F("GPRS connected"));
  }

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback([] (char* topic, byte * payload, unsigned int len)  {
    SerialMon.write(payload, len);
    SerialMon.println();
  });

  // locate devices on the bus
  sensors.begin();
  thermCnt = sensors.getDeviceCount();
  Serial.print(F("Found "));
  Serial.print(thermCnt, DEC);
  Serial.println(F(" devs."));

  for(char i = 0; i < thermCnt; ++i)
  {
    if (!sensors.getAddress(insideThermometer[i], i)) 
    {
      Serial.print(F("Dev"));
      Serial.print(i, DEC);
      Serial.print(F(" addr error")); 
    }
    else
    {
      sensors.setResolution(insideThermometer[i], 9);
      printAddress(i, insideThermometer[i]);
      Serial.println();
    }
  }
}

/*
 * Main function. It will request the tempC from the sensors and display on Serial.
 */

// заглушка
long last = 0;
void publishTemperature() {
  long now = millis();
  if (now - last > PUB_DELAY) {
    if(thermCnt > 0)
    {
      sensors.requestTemperatures(); // Send the command to get temperatures
      float tempC = sensors.getTempC(insideThermometer[0]);
      mqtt.publish("base/state/temperature", String(tempC).c_str());
      int a7 = analogRead(A7);
      float v = (a7 * 5.0) / 1024.0;
      mqtt.publish("base/state/battery", String(v).c_str());
      int d6 = digitalRead(6);
      mqtt.publish("base/state/externalpower", String(d6).c_str());
      
    }
    // mqtt.publish("base/state/humidity", String(random(40, 90)).c_str());
    last = now;
  }
}


void loop(void)
{ 
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) {
    SerialMon.println(F("Network disconnected"));
    if (!modem.waitForNetwork(180000L, true)) {
      SerialMon.println(F(" fail"));
      delay(10000);
      return;
    }
    if (modem.isNetworkConnected()) {
      SerialMon.println(F("Network re-connected"));
    }

    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      SerialMon.println(F("GPRS disconnected!"));
      SerialMon.print(F("Connecting to "));
      SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(" fail");
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) {
        SerialMon.println(F("GPRS reconnected"));
      }
    }
  }

  if (!mqtt.connected()) {
    SerialMon.println(F("=== MQTT NOT CONNECTED ==="));
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }

  mqtt.loop();
  publishTemperature();
  ////////////////////////////////////////////////////
  // long t = millis();
  // if((t - lastUpdate > 20000) || (t < lastUpdate)) // для отработки переполнения
  // {
  //   lastUpdate = t;
  //   RequestAndPrintTemp();
  // }
}

void RequestAndPrintTemp()
{
  {
    sensors.requestTemperatures(); // Send the command to get temperatures
    // It responds almost immediately. Let's print out the data
    for(int i = 0; i < thermCnt; ++i)
    {
        printTemperature(insideThermometer[i], 0, 0, i+1); // Use a simple function to print out the data
    }

    // измерение напряжения на батарее/питании
    int a7 = analogRead(A7);
    float v = (a7 * 5.0) / 1024.0;
    // наличие внешнего питания 
    int d6 = digitalRead(6);
    Serial.println(" External: " + String(d6) + " Battery: " + String(v, 1));
  }
}

// function to print a device address
void printAddress(uint8_t id, DeviceAddress deviceAddress)
{
  Serial.print(String(id) + " ");
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress, uint8_t x, uint8_t y, uint8_t id)
{
  float tempC = sensors.getTempC(deviceAddress);
  printAddress(id, deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println( " :err");
    return;
  }
  Serial.print(String(tempC < 0 ? " -" : " +") + String(abs(tempC) < 10 ? " " : "") + String(tempC, 0));
}
