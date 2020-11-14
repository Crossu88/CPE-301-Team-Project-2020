#include "dht.h"
#include <LiquidCrystal.h>

#define DHT11_PIN 54
#define WS_ADC_ID 1
#define WATER_MIN 64
#define ANALOGUE_PIN( n ) ( 54 + n )

void PollWaterSensor();
void PollDHT();


char printBuffer[128];

dht DHT;
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup()
{
  Serial.begin(9600);

  lcd.begin(16, 2);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  PollDHT();

  PollWaterSensor();

  delay(2000);
}

void PollWaterSensor()
{
  int value = analogRead(WS_ADC_ID); // get adc value

  sprintf(printBuffer, "ADC%d level is %d\n", WS_ADC_ID, value);
  Serial.print(printBuffer);

  if (value > WATER_MIN)
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on
  else
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off
}

void PollDHT()
{
  DHT.read11(DHT11_PIN);

  lcd.setCursor(0,0); 
  lcd.print("Temp: ");
  lcd.print(DHT.temperature);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print(DHT.humidity);
  lcd.print("%");
}
