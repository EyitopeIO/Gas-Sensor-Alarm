#include <Wire.h>   
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>  
#include <LiquidCrystal_I2C.h>
#include <Sim800L.h>

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
#define         RL_VALUE                     (10) //define the load resistance on the board, in kilo ohms
//#define         RL_VALUE                     (20) //define the load resistance on the board, in kilo ohms
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


static const int MEMORY_ADDRESS_THRESHOLD = 0;   //eeprom address to keep threshold value
static const int VERIFY_SAVED_PARA = 256;

String Incoming = "";   //hold incoming serial data

//Location of communication lab, central engineering workshop
volatile double longitude = 7.302806;  
volatile double lattitude = 5.133139;


char *telephone_precious = "+2348034344308";
char *telephone_lecturer = "+2348060981990";
static unsigned char text[160];

int BUZZER = 4;   
int LED_RED_ALARM = 5; 
int LED_GREEN_OKAY = 6; 
int BUTTON_THRESH_UP = 11; 
int BUTTON_THRESH_DOWN = 12;
int threshold_upper;   //stored threshold in memory
int threshold_lower;

volatile static uint32_t LPG_level;
volatile static uint32_t CH4_level;

static unsigned int THRESHOLD;

int sendSMS(void);
void beep(unsigned int duration);
void buttonPressCheck(void);
bool getGPSdata(void);
void alert(void);
void one_time_burn(void);
void showReadings(void);

static const int gps_RXPin = 8, gps_TXPin = 7;
static const int gsm_RXPin = 9, gsm_TXPin = 10;
static const uint32_t BAUD_DEFAULT = 9600;

Sim800L gsm_uart(gsm_RXPin, gsm_TXPin);   //Rx, Tx
SoftwareSerial gps_uart(gps_RXPin, gps_TXPin);    
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(I2C_ADDRESS, 2, 1, 0, 4, 5, 6, 7, BACKLIGHT_PIN, POSITIVE);

void setup()
{
  lcd.begin(16, 2);
  lcd.home();
  lcd.setCursor(0,0);
  lcd.setBacklight(HIGH);

  pinMode(MQ_PIN, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_RED_ALARM, OUTPUT);
  pinMode(LED_GREEN_OKAY, OUTPUT);
  pinMode(BUTTON_THRESH_UP, INPUT);   //going low activates it
  pinMode(BUTTON_THRESH_DOWN, INPUT);

  digitalWrite(LED_GREEN_OKAY, LOW);
  digitalWrite(LED_RED_ALARM, HIGH);    //turn on red led
  beep(100);

  one_time_burn();

  Serial.begin(BAUD_DEFAULT);
  gps_uart.begin(BAUD_DEFAULT);
  gsm_uart.begin(BAUD_DEFAULT);
  lcd.clear();
  lcd.setCursor(0,0);   //same as lcd.home( )
  lcd.print(F("Warming"));
  lcd.setCursor(0,1);
  lcd.print(F("Sensor..."));
  Ro = MQCalibration(MQ_PIN);
  
  digitalWrite(LED_RED_ALARM, LOW);
  digitalWrite(LED_GREEN_OKAY, HIGH);
  beep(500);
  lcd.clear();

  getGPSdata();
}


void loop()
{
  unsigned long time_now = millis();
  while(millis() - time_now <= 60000)
  {
    showReadings();
    buttonPressCheck();
    LPG_level = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
    CH4_level = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CH4);
    if ( (LPG_level > THRESHOLD) || (CH4_level > THRESHOLD) )
    {
      alert();
    }
  }
  getGPSdata(); //Skips right away if no GPS data available
}


void one_time_burn(void)
{
  /*
   * This is to ensure that the default value is loaded into the system when system starts for the first time
   * Subsequent threshold values are then saved by the user.
   */
  unsigned int once;
  EEPROM.get(VERIFY_SAVED_PARA, once);   //retrieve saved eeprom address
  if (once != (unsigned int)222)
  {
    EEPROM.put(VERIFY_SAVED_PARA, (unsigned int)222);
    EEPROM.put(MEMORY_ADDRESS_THRESHOLD, 1000);
  }
  else
  { 
    EEPROM.get(MEMORY_ADDRESS_THRESHOLD, THRESHOLD);   //retrieve saved eeprom address
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
  if((digitalRead(BUTTON_THRESH_DOWN)==LOW) && (digitalRead(BUTTON_THRESH_UP)==LOW)){
    lcd.clear();
    lcd.home();
    lcd.print(F("Threshold: "));
    lcd.setCursor(0, 1);
    lcd.print(THRESHOLD);
    lcd.setCursor(5, 1);
    lcd.print(F("ppm"));
    delay(1000);
    lcd.clear();
  }
  if(digitalRead(BUTTON_THRESH_UP) == LOW) {
    beep(100);
    THRESHOLD = THRESHOLD + 100;
    EEPROM.put(MEMORY_ADDRESS_THRESHOLD, THRESHOLD);
    lcd.clear();
    lcd.setCursor(0, 0);    //column, row
    lcd.print(F("Threshold ++ "));
    lcd.setCursor(0, 1);
    lcd.print(THRESHOLD);
    lcd.setCursor(5, 1);
    lcd.print(F("ppm"));
    delay(1000);
    lcd.clear();
  }
  if(digitalRead(BUTTON_THRESH_DOWN) == LOW) {
    beep(100);
    THRESHOLD = THRESHOLD - 100;
    EEPROM.put(MEMORY_ADDRESS_THRESHOLD, THRESHOLD);
    lcd.clear();
    lcd.setCursor(0, 0); //column, row
    lcd.print(F("Threshold -- "));
    lcd.setCursor(0, 1);
    lcd.print(THRESHOLD);
    lcd.setCursor(5, 1);
    lcd.print(F("ppm"));
    delay(1000);
    lcd.clear();
  }
}

bool getGPSdata(void)
{
  gps_uart.listen();
  lcd.clear();
  //unsigned long time_now = millis();
  lcd.home();
  lcd.print(F("Waiting for"));
  lcd.setCursor(0, 1);
  lcd.print(F("GPS signal... "));
  while(gps_uart.available() > 0) {
    
    if(gps.encode(gps_uart.read())) {
      if(gps.location.isValid()) {
        lattitude = gps.location.lat();
        longitude = gps.location.lng();
      }
    }
  }
  lcd.clear();
  lcd.home();
  lcd.print(F("Lat:"));
  lcd.print(lattitude, 4);
  lcd.setCursor(0,1);
  lcd.print(F("Lng:"));
  lcd.print(longitude, 4);
  delay(2000);
  lcd.clear();
}

void alert(void)
{
  lcd.clear();
  lcd.home();
  lcd.print(F("Alert process"));
  lcd.setCursor(0, 1);
  lcd.print(F("running..."));
  delay(500);
  /*
  digitalWrite(BUZZER,HIGH);
  digitalWrite(LED_RED_ALARM,HIGH);
  digitalWrite(LED_GREEN_OKAY,LOW);
  */
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
  gsm_uart.listen(); 
  sprintf(text, "Gas alert! Conc. CH4,LPG: %u,%u ppm; Lat,Lng: %.2lf, %.2lf",CH4_level,LPG_level,lattitude,longitude);
  //char *text = Outgoing.c_str();  //Convert C++ string to C char array
  
  if(gsm_uart.sendSms(telephone_precious, text)){
    lcd.clear();
    lcd.home();
    lcd.print(F("SMS failed to"));
    lcd.setCursor(0, 1);
    lcd.print(telephone_precious);
    delay(500);
  }
  else {
    lcd.clear();
    lcd.home();
    lcd.print(F("SMS sent to"));
    lcd.setCursor(0, 1);
    lcd.print(telephone_precious);
    delay(1000);
  }
  /*
  if(gsm_uart.sendSms(telephone_lecturer, text)){
    lcd.clear();
    lcd.home();
    lcd.print(F("SMS failed to"));
    lcd.setCursor(0, 1);
    lcd.print(telephone_lecturer);
    delay(500);
  }
  else {
    lcd.clear();
    lcd.home();
    lcd.print(F("SMS sent to"));
    lcd.setCursor(0, 1);
    lcd.print(telephone_lecturer);
    delay(500);
  }
  */
  
  if(gsm_uart.sendSms("+2348142357637", text)){
    lcd.clear();
    lcd.home();
    lcd.print(F("SMS failed to"));
    lcd.setCursor(0, 1);
    lcd.print("+2348142357637");
    delay(1000);
  }
  else {
    lcd.clear();
    lcd.home();
    lcd.print(F("SMS sent to"));
    lcd.setCursor(0, 1);
    lcd.print("+2348142357637");
    delay(1000); 
  }
  
  lcd.clear();
  lcd.home();
  lcd.print(F(" All SMS sent."));
  delay(2000);
  lcd.clear();
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
