#include <Wire.h>   //for I2C LCD
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <GSM.h>
#include <SoftwareSerial.h>   //AT commands leaving through software 

#define TEN_SECONDS 10000

const int MEMORY_ADDRESS_THRESHOLD = 0;   //eeprom address to keep threshold value
const int GPSBAUD = 9600;

char DATA_stream[256]; //incoming buffer from software UART
char DATA_out[100];   //use for anything
String longitude;
String lattitude;

const char* telephone = "+2348034344308";

int GAS_SENSOR_PIN = A7;
int BUZZER = 15;    //D12
int LED_RED_ALARM = 8;   //D5
int LED_GREEN_OKAY = 11;   //D8
int BUTTON_THRESH_UP = 5;   //D2
int BUTTON_THRESH_DOWN = 6;   //D3
int GAS_TRUTH_VALUE = 7;    //D4

unsigned int MQ6_gas_level = 0;   //read from analog input pin
byte threshold_upper;   //stored threshold in memory
byte threshold_lower;

unsigned int threshold;   //threshold value in percentage
unsigned long time_now = millis();

void sendSMS(void);
void beep(unsigned int duration);
void buttonPressCheck(void);
void getGasConc(void);
bool getGPSdata(void);
void alert(void);
int checkResponse(const char *response, unsigned long timeout);
int sendCmdAndWaitForResponse(const char *cmd, const char *a_part_of_expected_response, unsigned long timeout, unsigned char trials);

SoftwareSerial softUART(9,10);   //for a7 module
TinyGPSPlus gps;
GSM_SMS sms;

void setup()
{
  pinMode(GAS_SENSOR_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED_ALARM, OUTPUT);
  pinMode(LED_GREEN_OKAY, OUTPUT);
  pinMode(BUTTON_THRESH_UP, INPUT);   //going low activates it
  pinMode(BUTTON_THRESH_DOWN, INPUT);
  pinMode(GAS_TRUTH_VALUE, INPUT);

  digitalWrite(LED_GREEN_OKAY, LOW);
  digitalWrite(LED_RED_ALARM, HIGH);    //turn on red led
  beep(700);

  //SETUP LCD HERE
  
  EEPROM.get(MEMORY_ADDRESS_THRESHOLD, threshold);   //retrieve saved eeprom address

  softUART.begin(GPSBAUD);
  while(!softUART);

  getGasConc();

  digitalWrite(LED_RED_ALARM, LOW);
  digitalWrite(LED_GREEN_OKAY, HIGH);
}

void loop()
{
  time_now = millis();
  if(millis() - time_now > 30000)
  {
    getGPSdata();
  }
  getGasConc();
  buttonPressCheck();
  if (MQ6_gas_level > threshold)
  {
    alert();
  }
}

void buttonPressCheck(void)
{
  if(digitalRead(BUTTON_THRESH_UP) == LOW && (threshold < 100))
  {
    beep(500);
    EEPROM.update(MEMORY_ADDRESS_THRESHOLD, threshold++);
    //UPDATE LCD HERE
  }
  if(digitalRead(BUTTON_THRESH_DOWN) == LOW && (threshold > 0))
  {
    beep(500);
    EEPROM.update(MEMORY_ADDRESS_THRESHOLD, threshold--);
    //UPDATE LCD HERE
  }
}

void getGasConc(void)
{
   MQ6_gas_level = analogRead(GAS_SENSOR_PIN);
  //MQ6_gas_level = map(MQ6_gas_level, 0, 100, 0, 255)
  //MQ6_gas_level = constrain(MQ6_gas_level, 0
}

bool getGPSdata(void)
{
  while(softUART.available() > 0)
  {
    if(gps.encode(softUART.read()))
    {
      if(gps.location.isValid())
      {
        lattitude = gps.location.lat();
        longitude = gps.location.lng();
      }
    }
    else
    {
      return false;
    }
  }
}

void alert(void)
{
  digitalWrite(BUZZER,HIGH);
  digitalWrite(LED_RED_ALARM,HIGH);
  digitalWrite(LED_GREEN_OKAY,LOW);
  sendSMS();
}

void beep(unsigned int duration_in_milliseconds)
{
  digitalWrite(BUZZER, HIGH);
  delay(duration_in_milliseconds);
  digitalWrite(BUZZER, LOW);
}

void sendSMS()
{
  sprintf(DATA_out, "WARNING: lat=%s,lng=%s",lattitude.c_str(),longitude.c_str());
}
