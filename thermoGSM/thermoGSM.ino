// Термодатчики
#include <OneWire.h>
#include <DallasTemperature.h>

// ЖК-дисплей
#ifdef THERMO_USE_LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EncButton.h>
#endif

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

// ЖК-дисплей
#ifdef THERMO_USE_LCD
// Работа с LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);
// ЖК-индикатор выключен, включается по кнопке
bool isLcdOn = false;
Button btn(3);
#endif

void setup(void)
{
  pinMode(6, INPUT);
  // start serial port
  Serial.begin(9600);
  
  // locate devices on the bus
  sensors.begin();
  thermCnt = sensors.getDeviceCount();
  Serial.print("Found ");
  Serial.print(thermCnt, DEC);
  Serial.println(" devs.");
// ЖК-дисплей
#ifdef THERMO_USE_LCD
  lcd.init(); // Инициализация LCD, для просмотра на нем информации по датчикам
  lcd.backlight();
#endif

  for(char i = 0; i < thermCnt; ++i)
  {
    if (!sensors.getAddress(insideThermometer[i], i)) 
      Serial.println("Dev" + String(i) + " addr error"); 
    else
    {
      sensors.setResolution(insideThermometer[i], 9);
      printAddress(i, insideThermometer[i]);
      Serial.println();
#ifdef THERMO_USE_LCD
      printAddressLCD(i, insideThermometer[i]);
      delay(2000);
#endif
    }
  }
#ifdef THERMO_USE_LCD
  lcd.noBacklight();
  lcd.clear();
#endif  
}

/*
 * Main function. It will request the tempC from the sensors and display on Serial.
 */

void loop(void)
{ 
#ifdef THERMO_USE_LCD
  btn.tick();
  if(btn.click())
  {
    lcd.backlight();
    RequestAndPrintTemp();
    delay(1000);
    lcd.noBacklight();
  }  
  else
#endif
  {
    if((millis() - lastUpdate > 5000) || (millis() < lastUpdate)) // для отработки переполнения
    {
      lastUpdate = millis();
      RequestAndPrintTemp();
    }
  }
}

char prefix[] = {'-', '|'}; // "вращение" для проверки, что измерения идут
char prefix_id = 0;
void RequestAndPrintTemp()
{
  {
    sensors.requestTemperatures(); // Send the command to get temperatures
#ifdef THERMO_USE_LCD
    lcd.setCursor(0, 0);
    prefix_id = prefix_id % 2;
    lcd.print(prefix[prefix_id]);
    prefix_id++;
#endif
    // It responds almost immediately. Let's print out the data
    for(int i = 0; i < thermCnt; ++i)
    {
        printTemperature(insideThermometer[i], 1 + 5 * (i % 3), i / 3, i+1); // Use a simple function to print out the data
    }

    // измерение напряжения на батарее/питании
    int a7 = analogRead(A7);
    float v = (a7 * 5.0) / 1024.0;
    // наличие внешнего питания 
    int d6 = digitalRead(6);
#ifdef THERMO_USE_LCD
    lcd.setCursor(11, 1);
    lcd.print("V" + String(abs(v) < 9.99 ? " " : "" )+ String(v, 1));
#else
    Serial.println(" External: " + String(d6) + " Battery: " + String(v, 1));
#endif
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

#ifdef THERMO_USE_LCD
void printAddressLCD(uint8_t id, DeviceAddress deviceAddress)
{
  lcd.setCursor(0, 0);
  lcd.print(String(id));
  lcd.setCursor(0, 1);
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) lcd.print("0");
    lcd.print(deviceAddress[i], HEX);
  }
}
#endif

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress, uint8_t x, uint8_t y, uint8_t id)
{
  float tempC = sensors.getTempC(deviceAddress);
#ifdef THERMO_USE_LCD
  lcd.setCursor(x, y);
#endif
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
#ifdef THERMO_USE_LCD
    lcd.print(String(id) + ":err");
#else
    Serial.println(String(id) + ":err");
#endif
    return;
  }
#ifdef THERMO_USE_LCD
  lcd.print(String(id));
  lcd.print(String(tempC < 0 ? "-" : "+") + String(abs(tempC) < 10 ? " " : "") + String(tempC, 0));
#else
  Serial.print(String(id));
  Serial.print(String(tempC < 0 ? "-" : "+") + String(abs(tempC) < 10 ? " " : "") + String(tempC, 0));
#endif
}
