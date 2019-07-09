#include <Wire.h>   //for I2C LCD
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>   //AT commands leaving through software 
#define TEN_SECONDS 10000

                                                   
/************************************************************************************/
//  Sandbox Electronics    2014-02-03
/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (20)    //define the load resistance on the board, in kilo ohms
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


/*******************************************************************************/


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
int threshold_upper;   //stored threshold in memory
int threshold_lower;

unsigned int THRESHOLD = 2100;  //ppm

int sendSMS(void);
void beep(unsigned int duration);
void buttonPressCheck(void);
bool getGPSdata(void);
void alert(void);
void calibrateSensor();
int sendCmdAndWait(const char* cmd, const char* resp);

SoftwareSerial softUART(9,10);   //Rx, Tx
TinyGPSPlus gps;




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
  
  EEPROM.get(MEMORY_ADDRESS_THRESHOLD, THRESHOLD);   //retrieve saved eeprom address

  softUART.begin(GPSBAUD);
  while(!softUART);
  getGPSdata();
  
  Serial.begin(9600);
  Serial.print("Calibrating...\n");
  Ro = MQCalibration(MQ_PIN);
  Serial.print("Calibrating is done...\n");
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
  
  digitalWrite(LED_RED_ALARM, LOW);
  digitalWrite(LED_GREEN_OKAY, HIGH);
}





void loop()
{
  unsigned long time_now = millis();
  buttonPressCheck();
  if(millis() - time_now > 30000)   //data every 30s
  {
    getGPSdata();
  }
  if(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) >  THRESHOLD)
  {
    alert();
  }
}





void buttonPressCheck(void)
{
  if(digitalRead(BUTTON_THRESH_UP) == LOW)
  {
    beep(500);
    EEPROM.update(MEMORY_ADDRESS_THRESHOLD, THRESHOLD++);
    //UPDATE LCD HERE
  }
  else if(digitalRead(BUTTON_THRESH_DOWN) == LOW)
  {
    beep(500);
    EEPROM.update(MEMORY_ADDRESS_THRESHOLD, THRESHOLD--);
    //UPDATE LCD HERE
  }
  else
  {
    beep(500);
  }
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


int sendSMS()
{
  if(sendCmdAndWait("AT\n","OK"))
  {
    if(sendCmdAndWait("AT+CMGF=1\n","OK"))
    {
      emptyBuffer(DATA_out);
      sprintf(DATA_out, "AT+CMGS=%s", telephone); 
      if(sendCmdAndWait(DATA_out,">"))
      {
       sprintf(DATA_out, "WARNING: lat=%s,lng=%s",lattitude.c_str(),longitude.c_str());
       softUART.write(DATA_out);
       sendCmdAndWait(char(26), "CMGS"); 
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

  
int sendCmdAndWait(const char* cmd, const char* resp)
{
   softUART.write(cmd);
   while(softUART.available())
   {
    int len = strlen(resp);
    int i = 0;
    char c = softUART.read();
    i = (c == resp[i]) ? i++ : 0;
    if(i == len)
    {
      return 1;
    }
    else
    {
      return 0;
    }
   }
   return 0;
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
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
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
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
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
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
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
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CH4 ) {
      return MQGetPercentage(rs_ro_ratio,CH4Curve);
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
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, (((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
