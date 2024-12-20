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
#define THERMO_MAX_CNT 8
char thermCnt = 0;
DeviceAddress insideThermometer[THERMO_MAX_CNT];

// Периодическое обновление
long lastUpdate = 0;


void setup(void)
{
  // контроль наличия внешнего питания
  pinMode(6, INPUT);
  // start serial port
  Serial.begin(9600);

  // locate devices on the bus
  sensors.begin();
  thermCnt = sensors.getDeviceCount();
  Serial.print("Found ");
  Serial.print(thermCnt, DEC);
  Serial.println(" devs.");

  for(char i = 0; i < thermCnt; ++i)
  {
    if (!sensors.getAddress(insideThermometer[i], i)) 
      Serial.println("Dev" + String(i) + " addr error"); 
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

void loop(void)
{ 
  long t = millis();
  if((t - lastUpdate > 20000) || (t < lastUpdate)) // для отработки переполнения
  {
    lastUpdate = t;
    RequestAndPrintTemp();
  }
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
