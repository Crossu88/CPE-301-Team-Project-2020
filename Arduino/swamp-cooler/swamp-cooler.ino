/* -- Includes -- */
#include "dht.h"
#include <LiquidCrystal.h>
/* -- Preprocessor Definitions -- */
#define DHT11_PIN 54
#define WS_ADC_ID 1
#define WATER_MIN 64
#define ANALOGUE_PIN( n ) ( 54 + n )
/* -- Prototypes -- */
void PollWaterSensor();
void PollDHT();
void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);
/* -- Globals -- */
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
/* -- Globals -- */
// Print buffer for the sprintf function
char printBuffer[128];
// DHT object for reading the value of the DHT sensor
dht DHT;
// Define the pins being used by the LCD
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

void setup()
{
  // Begin serial monitoring
  Serial.begin(9600);
  // Begin transmitting to the LCD screen
  lcd.begin(16, 2);
  // Setup the built in LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // Poll the DHT and print to the LCD
  PollDHT();
  // Poll the water sensor and print to the Serial Monitor
  PollWaterSensor();
  // Delay for 2 seconds
  delay(2000);
}

void PollWaterSensor()
{
  // Read the value of the ADC
  // legacy read -- int value = analogRead(WS_ADC_ID); // get adc value
  unsigned int value = adc_read( WS_ADC_ID );
  // Print the ADC ID and the value read from the port.
  sprintf(printBuffer, "ADC%d level is %d\n", WS_ADC_ID, value);
  Serial.print(printBuffer);
  // Change the value of the LED depending on the water level
  if (value > WATER_MIN)
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on
  else
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED off
}

void PollDHT()
{
  // Read the DHT sensor
  DHT.read11(DHT11_PIN);
  // Print to the LCD
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

void adc_init()
{
  // setup the A register
  // set bit   7 to 1 to enable the ADC
  *my_ADCSRA |= 0b10000000;
  
  // clear bit 5 to 0 to disable the ADC trigger mode
  // clear bit 3 to 0 to disable the ADC interrupt
  // clear bit 2:0 to 0 to set prescaler selection to slow reading
  *my_ADCSRA &= 0b11010000;

  // setup the B register
  // clear bit 3 to 0 to reset the channel and gain bits
  // clear bits 2-0 to 0 to set free running mode
  *my_ADCSRB &= 0b11110000;

  // setup the MUX register
  // clear bit 7 to 0 for AVCC analog reference
  // set bit   6 to 1 for AVCC analog reference
  // clear bit 5 to 0 for right adjust result
  // clear bit 4:0 to 0 to reset the channel and gain bits
  *my_ADMUX = 0b01000000;
}

unsigned int adc_read( unsigned char adc_channel_num )
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;

  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;

  // set the channel number
  if (adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;

    //set MUX5
    *my_ADCSRB |= 0b00001000;
  }
  
  // set the channel selection bits
  *my_ADMUX += adc_channel_num;

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;

  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0 );

  // return the result in the ADC data register
  return *my_ADC_DATA;
}
