/* -- Includes -- */
#include <Arduino.h>
#include <dht.h>
#include <LiquidCrystal.h>
#include <Servo.h>

/* -- Preprocessor Definitions -- */
#define DHT11_PIN 54
#define WS_ADC_ID 1
#define POT_ADC_ID 2
#define BUTTON_PIN 9
#define LED_BLUE 10
#define LED_GREEN 11
#define LED_YELLOW 12
#define LED_RED 13
#define WATER_THRESHOLD 64
#define TEMP_THRESHOLD 30
#define ANALOGUE_PIN( n ) ( 54 + n )

/* -- Register Variables -- */
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

/* -- Globals -- */
// Enum for states of the system.
enum SystemState {RUNNING, IDLE, DISABLED, ERROR};
// Bool for enabling/disabling the system
bool SysEnabled = true;
// Print buffer for the sprintf function
char PrintBuffer[128];
// DHT object for reading the value of the DHT sensor
dht DHT;
// Servo object for the fan
Servo FanServo;
// Define the pins being used by the LCD
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

/* -- Prototypes -- */
void HandleState();
void PollDHT();
void PrintDHTReading();
void PrintErrorState();
void PrintDisabledState();
void SetLEDState(SystemState state);
void ClearLEDState();
void adc_init();
bool CheckWaterState();
bool CheckTempState();
void SetFanAngle(int angle);
int PollPotentiometer();
unsigned int PollWS();
unsigned int adc_read(unsigned char adc_channel_num);

/* -- Program Body -- */
//
// Main
//
void setup()
{
  // Begin serial monitoring
  Serial.begin(9600);
  // Begin transmitting to the LCD screen
  lcd.begin(16, 2);
  // Attach the servo to PWM 9
  FanServo.attach(8);
  // Setup the LEDs
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
}
void loop()
{
  // 
  HandleState();
  // Delay for 2 seconds
  delay(2000);
}
//
// Determines the current state of the system and takes
// the appropriate actions depending on that state
//
void HandleState()
{
  // Read the button's state to see if it's being pressed
  // If it is, then flip the system state
  if (digitalRead(BUTTON_PIN) == LOW)
    SysEnabled = !SysEnabled;

  // If the system is enabled, determine if the system is running, 
  // idle, or in an error state
  if (SysEnabled)
  {
    // System is enabled, check the water state for an error
    Serial.println("Enabled State");
    if ( CheckWaterState() )
    {
      // If we have no water level error, then poll the DHT
      PollDHT();
      // Check the temperature to see if it's above the threshold
      if ( CheckTempState() )
      {
        // Temperature is above threshold, trigger the running state
        Serial.println("Running State");
        SetLEDState(RUNNING);
        SetFanAngle( PollPotentiometer() );
      }
      else
      {
        // Temperature is below threshold, trigger idle state
        Serial.println("Idle State");
        SetLEDState(IDLE);
        SetFanAngle(0);
      }
    }
    else
    {
      // Water level error, trigger the error state
      Serial.println("Error State");
      SetLEDState(ERROR);
    }
  }
  else
  {
    Serial.println("Disabled State");
    SetLEDState(DISABLED);
    SetFanAngle(0);
  }
}
//
// Sets the state of the LEDs based on the provided SystemState enum
//
void SetLEDState(SystemState state)
{
  // Clear the current state of the LEDs
  ClearLEDState();

  // Depending on the provided state, set a specific LED to HIGH
  switch (state)
  {
  case RUNNING:
    digitalWrite(LED_BLUE, HIGH);
    break;
  case IDLE:
    digitalWrite(LED_GREEN, HIGH);
    break;
  case DISABLED:
    digitalWrite(LED_YELLOW, HIGH);
    break;
  case ERROR:
    digitalWrite(LED_RED, HIGH);
    break;
  default:
    break;
  }
}
//
// Clears the state of the LEDs, setting them all to LOW
//
void ClearLEDState()
{
  // Set the blue LED to LOW
  digitalWrite(LED_BLUE, LOW);
  // Set the green LED to LOW
  digitalWrite(LED_GREEN, LOW);
  // Set the yellow LED to LOW
  digitalWrite(LED_YELLOW, LOW);
  // Set the red LED to LOW
  digitalWrite(LED_RED, LOW);
}
//
// Checks the water level as read by the water sensor
//
unsigned int PollWS()
{
  // Read the value of the ADC
  unsigned int value = adc_read( WS_ADC_ID );
  return value;
}
//
// Polls the Water Sensor connected to the Arduino, prints to the serial monitor
//
bool CheckWaterState()
{
  // Initialize variables
  bool water_state = true;
  unsigned int water_level = PollWS();
  // Print the ADC ID and the value read from the port.
  sprintf(PrintBuffer, "ADC%d level is %d\n", WS_ADC_ID, water_level);
  Serial.print(PrintBuffer);
  // If the water level is low, change the state to false
  if (water_level < WATER_THRESHOLD)
    water_state = false;

  return water_state;
}
//
// Polls the DHT connected to the Arduino for Temperature and Humidity
//
void PollDHT()
{
  // Read the DHT sensor for Temperature and Humidity
  // No need to return anything, the DHT is a global
  DHT.read11(DHT11_PIN);
  // Print the DHT reading
  PrintDHTReading();
}
//
// Checks the temperature to see if it is above the heat threshold
//
bool CheckTempState()
{
  bool temp_state = false;

  if (DHT.temperature > TEMP_THRESHOLD)
    temp_state = true;
  
  return temp_state;
}
//
// Sets the angle of the servo
//
void SetFanAngle(int angle)
{
  FanServo.write(angle);                  // sets the servo position according to the scaled value
}
//
// Poll the potentiometer and adjust the angle of the servo
//
int PollPotentiometer()
{
  int pot_val = analogRead(POT_ADC_ID);            // reads the value of the potentiometer (value between 0 and 1023)
  pot_val = map(pot_val, 0, 1023, 0, 90);     // scale it to use it with the servo (value between 0 and 180)
  return pot_val; 
}
void PrintDHTReading()
{
  // Prints the Temperature and the Humidity of the DHT to the LCD
  lcd.clear();
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
//
// Prints the error state to the LCD
//
void PrintErrorState()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR:");
  lcd.setCursor(0, 1);
  lcd.print("Water Level Low!");
}
//
// Prints the error state to the LCD
//
void PrintDisabledState()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WARNING:");
  lcd.setCursor(0, 1);
  lcd.print("System Disabled!");
}
//
// Sets up the ADC for input
//
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
//
// Reads and returns the result from and ADC channel
//
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