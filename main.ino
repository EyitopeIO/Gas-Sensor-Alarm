#include <Wire.h>   //for I2C LCD
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>   //AT commands leaving through software 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define TEN_SECONDS 10000
#define THREE 3
#define I2C_ADDRESS 0x27    //Run a I2C scanner code to find this
#define BACKLIGHT_PIN 3     //Library examples declares this as 13. Beware.





/*******************Demo for MQ-6 Gas Sensor Module V1.3*****************************
Contact:  support[at]sandboxelectronics.com

Lisence: Attribution-NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)

Note:    This piece of source code is supposed to be used as a demostration ONLY. More
         sophisticated calibration is required for industrial field application. 

                                                    Sandbox Electronics    2014-02-03
************************************************************************************/

/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (7)     //define which analog input channel you are going to use
#define         RL_VALUE                     (1) //prev=20   //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (10)    //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
                                                     //normal operation

/**********************Application Related Macros**********************************/
#define         GAS_LPG                      (0)
#define         GAS_CH4                      (1)

/*****************************Globals***********************************************/
float           LPGCurve[3]  =  {3,   0,  -0.4};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg1000, lg1), point2: (lg10000, lg0.4) 
float           CH4Curve[3]  =  {3.3, 0,  -0.38};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg2000, lg1), point2: (lg5000,  lg0.7) 
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms

/******************************************************************************************************************************* */





const int MEMORY_ADDRESS_THRESHOLD = 0;   //eeprom address to keep threshold value
const int GPSBAUD = 9600;

char DATA_in[100];   //use for anything
String longitude;
String lattitude;

String telephone = "\"+2348034344308\"";

int BUZZER = 4;   
int LED_RED_ALARM = 5; 
int LED_GREEN_OKAY = 6; 
int BUTTON_THRESH_UP = 2; 
int BUTTON_THRESH_DOWN = 3;
int threshold_upper;   //stored threshold in memory
int threshold_lower;

unsigned int THRESHOLD = 2100;  //ppm

int sendSMS(void);
void beep(unsigned int duration);
void buttonPressCheck(void);
bool getGPSdata(void);
void alert(void);
void calibrateSensor(void);
void showReadings(void);
//int sendCmdAndWait(String cmd, String resp);
byte sendCmdAndWait(String cmd, String resp, SoftwareSerial object, unsigned int response_time, unsigned int trial);

SoftwareSerial gsm_uart(9, 10); //Tx, Rx
SoftwareSerial gps_uart(7,8);    //Tx, Rx
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(I2C_ADDRESS, 2, 1, 0, 4, 5, 6, 7, BACKLIGHT_PIN, POSITIVE);

void setup()
{
  lcd.begin(16, 2);
  lcd.home();
  lcd.setCursor(0,0);
  lcd.setBacklight(HIGH);
  lcd.print(F("Setting up..."));

  pinMode(MQ_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED_ALARM, OUTPUT);
  pinMode(LED_GREEN_OKAY, OUTPUT);
  pinMode(BUTTON_THRESH_UP, INPUT);   //going low activates it
  pinMode(BUTTON_THRESH_DOWN, INPUT);

  digitalWrite(LED_GREEN_OKAY, LOW);
  digitalWrite(LED_RED_ALARM, HIGH);    //turn on red led
  beep(500);
  
  EEPROM.get(MEMORY_ADDRESS_THRESHOLD, THRESHOLD);   //retrieve saved eeprom address

  Serial.begin(9600);
  gps_uart.begin(9600);
  gsm_uart.begin(9600);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("Warming"));
  lcd.setCursor(0,1);
  lcd.print(F("Sensor..."));
  Ro = MQCalibration(MQ_PIN);
  
  digitalWrite(LED_RED_ALARM, LOW);
  digitalWrite(LED_GREEN_OKAY, HIGH);
  beep(500);
  lcd.clear();
}


void loop()
{
  showReadings();
  unsigned long time_now = millis();
  buttonPressCheck();
  if(millis() - time_now > 10000)   //data every 30s
  {
    lcd.clear();
    lcd.setCursor(0, 0); //column, row
    lcd.print(F("Getting "));
    lcd.setCursor(0, 1);
    lcd.print(F("GPS data..."));
    getGPSdata();
  }
  if ( (MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) > THRESHOLD) || (MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CH4) > THRESHOLD) )
  {
    alert();
  }
}

void showReadings(void)
{
  lcd.home();
  lcd.print(F("LPG: "));
  lcd.setCursor(5, 0); 
  lcd.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG));
  lcd.setCursor(10, 0); 
  lcd.print(F("ppm"));

  lcd.setCursor(0, 1);
  lcd.print(F("CH4: "));
  lcd.setCursor(5, 1); 
  lcd.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_CH4));
  lcd.setCursor(10, 1);
  lcd.print(F("ppm"));
}


void buttonPressCheck(void)
{
  if(digitalRead(BUTTON_THRESH_UP) == LOW)
  {
    beep(500);
    EEPROM.update(MEMORY_ADDRESS_THRESHOLD, THRESHOLD++);
    lcd.clear();
    lcd.setCursor(0, 0);    //column, row
    lcd.print(F("Threshold: "));
    lcd.setCursor(0, 1);
    lcd.print(THRESHOLD);
    lcd.setCursor(5, 1);
    lcd.print(F("ppm"));
    delay(500);
    lcd.clear();
  }
  if(digitalRead(BUTTON_THRESH_DOWN) == LOW)
  {
    beep(500);
    EEPROM.update(MEMORY_ADDRESS_THRESHOLD, THRESHOLD--);
    lcd.clear();
    lcd.setCursor(0, 0); //column, row
    lcd.print(F("Threshold: "));
    lcd.setCursor(0, 1);
    lcd.print(THRESHOLD);
    lcd.setCursor(5, 1);
    lcd.print(F("ppm"));
    delay(500);
    lcd.clear();
  }
  if( (digitalRead(BUTTON_THRESH_DOWN) == LOW) && (digitalRead(BUTTON_THRESH_UP) == LOW) )
  {
    lcd.clear();
    lcd.home();
    lcd.print(F("Threshold: "));
    lcd.setCursor(0, 1);
    lcd.print(THRESHOLD);
    lcd.setCursor(5, 1);
    lcd.print(F("ppm"));
    delay(500);
    lcd.clear();
  }
}


bool getGPSdata(void)
{
  while(gps_uart.available() > 0)
  {
    if(gps.encode(gps_uart.read()))
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


int sendSMS()
{
  if(sendCmdAndWait(F("AT\n"),F("OK"), gsm_uart, TEN_SECONDS, THREE))
  {
    if(sendCmdAndWait(F("AT+CMGF=1\n"),F("OK"), gsm_uart, TEN_SECONDS, THREE))
    {
      String Outgoing = "AT+CMGS=" + telephone + '\n';
      if (sendCmdAndWait(Outgoing, ">", gsm_uart, TEN_SECONDS, THREE))
      {
        Outgoing = "WARNING! lat,lng: " + lattitude + ',' + longitude;
        gsm_uart.print(Outgoing);
        sendCmdAndWait(char(26), "CMGS", gsm_uart, TEN_SECONDS, THREE); 
      }
      else
      { 
        return 0;
      }
    }
    else
    {
      return 0;
    }
  }
  else
  {
    return 0;
  }
}

/*  
int sendCmdAndWait(String cmd, String resp)
{
   gps.write(cmd);
   unsigned long t_now = millis();
   int len = resp.length();
   int i = 0;
   while ( (millis() - t_now) > 10 )
   {
    while(gps.available())
    {
      char c = gps.read();
      i = (c == resp[i]) ? i++ : 0;
      if(i == len)
      {
        return 1;
      }
    }
   }
   return 0;
}
*/

byte sendCmdAndWait(String cmd, String resp, SoftwareSerial object, unsigned int response_time, unsigned int trial)
{
  unsigned long time_now = millis();
  int len = resp.length();
  object.print(cmd);
  while(trial--)
  { 
    while(millis() - time_now < response_time)
    {
      while (object.available())
      {
        int i, index_ = 0;
        char c = object.read();
        DATA_in[index_++] = c; //keeping our response just in case
        i = (c == resp[i]) ? i++ : 0;
        if (i == len)
        {
          return 1; //success
        }
      }
    }
  }
  return 0; //fail
}


void emptyBuffer(char* arg)
{
  unsigned int i;
  for (i=0;i<strlen(arg);i++)
  {
    arg[i] = '\0';
  }
}




/****************** MQResistanceCalculation ****************************************
Input:   raw_adc - raw value read from adc, which represents the voltage
Output:  the calculated sensor resistance
Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
  return (((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

/***************************** MQCalibration ****************************************
Input:   mq_pin - analog channel
Output:  Ro of the sensor
Remarks: This function assumes that the sensor is in clean air. It use  
         MQResistanceCalculation to calculates the sensor resistance in clean air 
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin)
{
  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++)
  { //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES; //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR; //divided by RO_CLEAN_AIR_FACTOR yields the Ro
                                   //according to the chart in the datasheet

  return val;
}
/*****************************  MQRead *********************************************
Input:   mq_pin - analog channel
Output:  Rs of the sensor
Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin)
{
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++)
  {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

/*****************************  MQGetGasPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
Output:  ppm of the target gas
Remarks: This function passes different curves to the MQGetPercentage function which 
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if (gas_id == GAS_LPG)
  {
    return MQGetPercentage(rs_ro_ratio, LPGCurve);
  }
  else if (gas_id == GAS_CH4)
  {
    return MQGetPercentage(rs_ro_ratio, CH4Curve);
  }

  return 0;
}

/*****************************  MQGetPercentage **********************************
Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/
int MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, (((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}
