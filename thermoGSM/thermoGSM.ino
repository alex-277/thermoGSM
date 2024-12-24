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
SoftwareSerial SerialAT(11, 10);  // RX, TX - TODO поменять местами (сделать RX 10, TX 11)
#endif

// #define TINY_GSM_DEBUG SerialMon // раскомментировать для вывода отладочных сообщений библиотеки

// Your GPRS credentials, if any
const char apn[]      = "internet.mts.ru";
const char gprsUser[] = "mts";
const char gprsPass[] = "mts";

// MQTT details
// const char * broker = "dev.rightech.io";
// const long port = 1883;
// const char * connect_to = "mqtt-a1506-kl6xda";
const char * broker = "m9.wqtt.ru";
const long port = 18758;
const char * user = "u_3K5SRD";
const char * psw = "zis9Zquk";

const char * topic_temp = "bolier1/temperature";
const char * topic_V = "bolier1/battery";
const char * topic_external = "bolier1/externalpower";

#include <TinyGsmClient.h>
#include <PubSubClient.h>

#define PUB_DELAY (5 * 1000) /* данные шлем каждые 5 минут, для проверки */

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

boolean mqttConnect() {

  // Connect to MQTT Broker
  boolean status = mqtt.connect("SIM800l_REMOTE", user, psw);

  if (status == false) {
    return false;
  }
  // TODO подписаться на топики, которые хотим принимать
  return mqtt.connected();
}

void watch_blink()
{
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
}

void setup(void)
{
  // встроенный светодиод для контроля
  pinMode(LED_BUILTIN, OUTPUT);
  // контроль наличия внешнего питания
  pinMode(6, INPUT);
  // start serial port
  SerialMon.begin(9600);
  delay(10);

  SerialMon.println(F("Start..."));

  // Термодатчики
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

  // MQTT Broker setup
  mqtt.setServer(broker, port).setKeepAlive(10 * 60);

  // GSM
  // Set GSM module baud rate
  SerialAT.begin(38400);
  delay(6000);

    // Restart takes quite some time
  // To skip it, call init() instead of restart()
  // SerialMon.println(F("Initializing modem..."));
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  // SerialMon.print(F("Modem Info: "));
  SerialMon.println(modemInfo);

  // SerialMon.print(F("Waiting for network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(F("waitForNetwork fail! Wait..."));
    delay(1000);
    return;
  }
  // SerialMon.println(F(" success"));

  if (modem.isNetworkConnected()) {
    // SerialMon.println(F("Network connected"));
  }

  // GPRS connection parameters are usually set after network registration
  // SerialMon.print(F("Connecting to "));
  // SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(F("GPRS connect fail! Wait..."));
    delay(1000);
    return;
  }
  // SerialMon.println(F(" success"));

  if (modem.isGprsConnected()) {
    // SerialMon.println(F("GPRS connected"));
  }


}

/*
 * Main function. It will request the tempC from the sensors and display on Serial.
 */

// Опрос сенсоров и отправка данных
uint32_t last = 0;
void publishTemperature() {
  uint32_t now = millis();
  if (now - last > PUB_DELAY) {
    if(thermCnt > 0)
    {
      sensors.requestTemperatures(); // Send the command to get temperatures
      float tempC = sensors.getTempC(insideThermometer[0]);
      mqtt.publish(topic_temp, String(tempC).c_str());
    }
    int a7 = analogRead(A7);
    float v = (a7 * 5.0) / 1024.0;
    mqtt.publish(topic_V, String(v).c_str());
    int d6 = digitalRead(6);
    mqtt.publish(topic_external, String(d6).c_str());
    last = now;
  }
}

long gsmConnectingsAttempts = 0;
uint32_t loop_counter = 0;  // счетчик времени для индикации
void loop(void)
{ 
  // Make sure we're still registered on the network
  if (!modem.isNetworkConnected()) 
  {
    SerialMon.print(F("Signal Quality:"));
    SerialMon.println(modem.getSignalQuality(), DEC);
    if(gsmConnectingsAttempts > 3)
    {
      SerialAT.end();
      delay(100);
      SerialAT.begin(38400);
      delay(6000);
      SerialMon.println(F("loop:restart modem"));
      modem.restart();
      gsmConnectingsAttempts = 0;
    }
    SerialMon.println("loop:network disconnected");

    if (!modem.waitForNetwork(/*180000L, true*/)) {
      SerialMon.println(F("Network disconnected! Wait..."));
      // SerialMon.println(F(" fail"));
      delay(10);
      ++gsmConnectingsAttempts;
      return;
    }
    if (modem.isNetworkConnected()) {
      // SerialMon.println(F("Network re-connected"));
      watch_blink();
      delay(100);
      watch_blink();
    }

    // and make sure GPRS/EPS is still connected
    if (!modem.isGprsConnected()) {
      // SerialMon.println(F("GPRS disconnected!"));
      // SerialMon.print(F("Connecting to "));
      // SerialMon.print(apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        SerialMon.println(F("!modem.gprsConnect. Wait..."));
        delay(10000);
        return;
      }
      if (modem.isGprsConnected()) {
        // SerialMon.println(F("GPRS reconnected"));
        watch_blink();
        delay(100);
        watch_blink();
      }
    }
  }
  gsmConnectingsAttempts = 0;
  if (!mqtt.connected()) {
    // SerialMon.println(F("=== MQTT NOT CONNECTED ==="));
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      SerialMon.println(F("== Try connect to broker..."));
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    watch_blink();
    delay(20);
    watch_blink();
    delay(20);
    watch_blink();
    delay(20);
    watch_blink();
    delay(20);
    return;
  }

  // добрались сюда - можем передавать данные 
  mqtt.loop();
  publishTemperature();
  if(millis() - loop_counter > 2000)
  {
    loop_counter = millis();
    watch_blink();
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
