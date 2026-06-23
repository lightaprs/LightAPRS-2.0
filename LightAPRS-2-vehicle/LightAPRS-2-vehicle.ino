#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdio.h>
#include <avr/dtostrf.h>
#include <MemoryFree.h>
#include <ZeroAPRS.h>                       //https://github.com/hakkican/ZeroAPRS
#include <SparkFun_Ublox_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library
#include <Adafruit_BMP085.h>                //https://github.com/adafruit/Adafruit-BMP085-Library

#define BattPin       A5
#define GpsPwr        7
#define PwDwPin       A3
#define PowerHL       A4
#define PttPin        3

//macros
#define GpsON       digitalWrite(GpsPwr, LOW)
#define GpsOFF      digitalWrite(GpsPwr, HIGH)
#define PttON       digitalWrite(PttPin, HIGH)
#define PttOFF      digitalWrite(PttPin, LOW)
#define RadioON     digitalWrite(PwDwPin, HIGH)
#define RadioOFF    digitalWrite(PwDwPin, LOW)
#define RfHiPwr     digitalWrite(PowerHL, HIGH)
#define RfLowPwr    digitalWrite(PowerHL, LOW)

//#define DEVMODE // Development mode. Uncomment to enable for debugging.

//****************************** APRS CONFIG **********************************
char    CallSign[7]="NOCALL"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN
int8_t  CallNumber=9;//SSID http://www.aprs.org/aprs11/SSIDs.txt
char    Symbol='>'; // 'O' for balloon, '>' for car, for more info : http://www.aprs.org/symbols/symbols-new.txt
bool    alternateSymbolTable = false ; //false = '/' , true = '\'

char Frequency[9]="144.3900"; //default frequency. 144.3900 for US, 144.8000 for Europe

char    comment[50] = "LightAPRS 2.0"; // Max 50 char but shorter is better.
char    StatusMessage[50] = "LightAPRS 2.0 by TA2NHP & TA2MUN";
//*****************************************************************************

//****************************** TELEMETRY CONFIG ******************************
#define ENABLE_TELEMETRY_DEFS // Transmits PARM, UNIT, EQNS definition messages
uint16_t TelemetryDefInitialDelay = 2; // Minutes to wait before first transmission
uint16_t TelemetryDefInterval = 60;    // Minutes between subsequent transmissions
//******************************************************************************

uint16_t  BeaconWait=50;  //seconds sleep for next beacon (HF or VHF). This is optimized value, do not change this if possible.
uint16_t  BattWait=60; //seconds sleep if super capacitors/batteries are below BattMin (important if power source is solar panel) 
float     BattMin=3.3; // min Volts to wake up.
float     DraHighVolt=5.0; // min Volts for radio module (DRA818V) to transmit (TX) 1 Watt, below this transmit 0.5 Watt.

//****************************** APRS SETTINGS *********************************
//do not change WIDE path settings below if you don't know what you are doing :) 
uint8_t   Wide1=1; // 1 for WIDE1-1 path
uint8_t   Wide2=1; // 1 for WIDE2-1 path

/**
Airborne stations above a few thousand feet should ideally use NO path at all, or at the maximum just WIDE2-1 alone.
Due to their extended transmit range due to elevation, multiple digipeater hops are not required by airborne stations.
Multi-hop paths just add needless congestion on the shared APRS channel in areas hundreds of miles away from the aircraft's own location.
NEVER use WIDE1-1 in an airborne path, since this can potentially trigger hundreds of home stations simultaneously over a radius of 150-200 miles.
*/
uint8_t pathSize=2; // 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N
boolean autoPathSizeHighAlt = false; //force path to WIDE2-N only for high altitude (airborne) beaconing (over 1.000 meters (3.280 feet)) 
boolean  aliveStatus = true; //for tx status message on first wake-up just once.
boolean radioSetup = false; //do not change this, temp value
static char telemetry_buff[100];// telemetry buffer
uint16_t TxCount = 1; //increased +1 after every APRS transmission

#ifdef ENABLE_TELEMETRY_DEFS
unsigned long lastDefTxMillis = 0;
bool initialDefSent = false;
#endif

//****************************** GPS SETTINGS   *********************************
int16_t   GpsResetTime=1800; // timeout for reset if GPS is not fixed
boolean ublox_high_alt_mode_enabled = false; //do not change this
int16_t GpsInvalidTime=0; //do not change this
boolean gpsSetup=false; //do not change this.

//********************************************************************************

SFE_UBLOX_GPS myGPS;
Adafruit_BMP085 bmp;

void setup() {
  // While the energy rises slowly with the solar panel, 
  // using the analog reference low solves the analog measurement errors.
  analogReference(AR_INTERNAL1V65);
  pinMode(PttPin, OUTPUT);
  pinMode(GpsPwr, OUTPUT);
  pinMode(BattPin, INPUT);
  pinMode(PwDwPin, OUTPUT);
  pinMode(PowerHL, OUTPUT);
  
  GpsOFF;
  PttOFF;
  RadioOFF; 
  RfLowPwr;

  SerialUSB.begin(115200);
  // Wait up to 5 seconds for serial to be opened, to allow catching
  // startup messages on native USB boards (that do not reset when
  // serial is opened).
  unsigned long start = millis();
  while (millis() - start < 5000 && !SerialUSB){;}

  SerialUSB.println(F("Starting"));
  Serial1.begin(9600);// for DorjiDRA818V

  APRS_init();
  APRS_setCallsign(CallSign, CallNumber);
  APRS_setDestination("APLIGA", 0);
  APRS_setPath1("WIDE1", Wide1);
  APRS_setPath2("WIDE2", Wide2);
  APRS_setPathSize(2);
  APRS_useAlternateSymbolTable(alternateSymbolTable);
  APRS_setSymbol(Symbol);
  APRS_setPathSize(pathSize);
  APRS_setGain(2);

  configDra818(Frequency);

  Wire.begin();
  bmp.begin();

  SerialUSB.println(F(""));
  SerialUSB.print(F("APRS (VHF) CallSign: "));
  SerialUSB.print(CallSign);
  SerialUSB.print(F("-"));
  SerialUSB.println(CallNumber);
}

void loop() {

if (readBatt() > BattMin) {
    
    if (aliveStatus) {	
      sendStatus();
      aliveStatus = false;

      while (readBatt() < BattMin) {
        sleepSeconds(BattWait);
      }
    }
      
      if(!gpsSetup) {gpsStart();}
      
      //Models for GPS: DYN_MODEL_PORTABLE, DYN_MODEL_STATIONARY, DYN_MODEL_PEDESTRIAN, DYN_MODEL_AUTOMOTIVE, DYN_MODEL_SEA, 
      //DYN_MODEL_AIRBORNE1g, DYN_MODEL_AIRBORNE2g, DYN_MODEL_AIRBORNE4g, DYN_MODEL_WRIST, DYN_MODEL_BIKE
      //DYN_MODEL_PORTABLE is suitable for most situations except airborne vehicles.
      if(!ublox_high_alt_mode_enabled){setupUBloxDynamicModel(DYN_MODEL_PORTABLE);}
      
      if (myGPS.getPVT()) {
        gpsDebug();
        if ( (myGPS.getFixType() != 0) && (myGPS.getSIV() > 3) ) {
          GpsInvalidTime=0;
          updatePosition();
          updateTelemetry();

          if(autoPathSizeHighAlt && ((myGPS.getAltitude() * 3.2808399)  / 1000.f) > 3000){
            //force to use high altitude settings (WIDE2-n)
            APRS_setPathSize(1);
          } else {
            //use default settings  
            APRS_setPathSize(pathSize);
          }

          sendLocationAndTelemetry();

          #ifdef ENABLE_TELEMETRY_DEFS
            unsigned long currentMillis = millis();
            unsigned long initialDelayMs = TelemetryDefInitialDelay * 60000UL;
            unsigned long intervalMs = TelemetryDefInterval * 60000UL;

            if (!initialDefSent && currentMillis > initialDelayMs) {
              sendTelemetryDefinitions();
              initialDefSent = true;
              lastDefTxMillis = currentMillis;
            }
            else if (initialDefSent && (currentMillis - lastDefTxMillis >= intervalMs)) {
              sendTelemetryDefinitions();
              lastDefTxMillis = currentMillis;
            }
          #endif

          freeMem();
          SerialUSB.flush();
          sleepSeconds(BeaconWait);
        }else{
          GpsInvalidTime++;
          if(GpsInvalidTime > GpsResetTime){
            GpsOFF;
            ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
            delay(1000);
            GpsON;
            GpsInvalidTime=0;
          }
        }
      } else {
        #if defined(DEVMODE)
        SerialUSB.println(F("Not enough sattelites"));
        #endif
      }
    
  } else {
    sleepSeconds(BattWait);
  }
}

void gpsStart(){  
  bool gpsBegin=false;  
  while(!gpsBegin){
    GpsON;
    delay(1000);
    Wire.begin();
    gpsBegin=myGPS.begin();
    if(gpsBegin)break;
    #if defined(DEVMODE)  
    SerialUSB.println(F("Ublox GPS not detected at default I2C address. Will try again"));
    #endif 
    delay(2000);
  }
   // do not overload the buffer system from the GPS, disable UART output
  myGPS.setUART1Output(0); //Disable the UART1 port output 
  myGPS.setUART2Output(0); //Disable Set the UART2 port output
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR  
  gpsSetup=true;
}

void sleepSeconds(int sec) {
  PttOFF;
  RadioOFF;

  SerialUSB.flush();
  for (int i = 0; i < sec; i++) {
    delay(1000);   
  }
}

byte configDra818(char *freq)
{
  RadioON;
  char ack[3];
  int n;
  delay(2000);
  char cmd[50];
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  Serial1.println(cmd);
  SerialUSB.println("RF Config");
  ack[2] = 0;
  while (ack[2] != 0xa)
  {
    if (Serial1.available() > 0) {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = Serial1.read();
    }
  }
  delay(2000);
  RadioOFF;
  if (ack[0] == 0x30) {
      SerialUSB.print(F("Frequency updated: "));
      SerialUSB.print(freq);
      SerialUSB.println(F("MHz"));
  } else {
      SerialUSB.println(F("Frequency update error!!!"));    
  }
  return (ack[0] == 0x30) ? 1 : 0;
}

void updatePosition() {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  int temp = 0;
  double d_lat = myGPS.getLatitude() / 10000000.f;
  double dm_lat = 0.0;
  if (d_lat < 0.0) {
    temp = -(int)d_lat;
    dm_lat = temp * 100.0 - (d_lat + temp) * 60.0;
  } else {
    temp = (int)d_lat;
    dm_lat = temp * 100 + (d_lat - temp) * 60.0;
  }

  dtostrf(dm_lat, 7, 2, latStr);
  if (dm_lat < 1000) {
    latStr[0] = '0';
  }

  if (d_lat >= 0.0) {
    latStr[7] = 'N';
  } else {
    latStr[7] = 'S';
  }

  APRS_setLat(latStr);
  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
  double d_lon = myGPS.getLongitude() / 10000000.f;
  double dm_lon = 0.0;
  if (d_lon < 0.0) {
    temp = -(int)d_lon;
    dm_lon = temp * 100.0 - (d_lon + temp) * 60.0;
  } else {
    temp = (int)d_lon;
    dm_lon = temp * 100 + (d_lon - temp) * 60.0;
  }

  dtostrf(dm_lon, 8, 2, lonStr);
  if (dm_lon < 10000) {
    lonStr[0] = '0';
  }
  if (dm_lon < 1000) {
    lonStr[1] = '0';
  }

  if (d_lon >= 0.0) {
    lonStr[8] = 'E';
  } else {
    lonStr[8] = 'W';
  }

  APRS_setLon(lonStr);
  APRS_setTimeStamp(myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond());
}

void updateTelemetry() {
  // Base Course/Speed/Altitude formatting ONLY
  sprintf(telemetry_buff, "%03d", (int)(myGPS.getHeading() / 100000));
  telemetry_buff[3] = '/';
  sprintf(telemetry_buff + 4, "%03d", (int)(myGPS.getGroundSpeed() * 0.00194384f));
  telemetry_buff[7] = '/';
  telemetry_buff[8] = 'A';
  telemetry_buff[9] = '=';

  float tempAltitude = (myGPS.getAltitude() * 3.2808399)  / 1000.f;
  if (tempAltitude > 0) {
    sprintf(telemetry_buff + 10, "%06lu", (long)tempAltitude);
  } else {
    sprintf(telemetry_buff + 10, "%06ld", (long)tempAltitude);
  }
  
  telemetry_buff[16] = ' '; 
  telemetry_buff[17] = '\0'; // Strictly terminate the string here
  
  // Append user comment
  strcat(telemetry_buff, comment);

  #if defined(DEVMODE)
  SerialUSB.println(telemetry_buff);
  #endif
}

void sendLocationAndTelemetry() {
  SerialUSB.println(F("Location & Telemetry sending..."));
  if (readBatt() > DraHighVolt) RfHiPwr; 
  else RfLowPwr; 
  RadioON;
  delay(2000);

  // 1. Send Location Packet
  PttON;
  delay(1000);  
  APRS_sendLoc(telemetry_buff);
  delay(10);
  PttOFF;

  delay(1000); // 1-second gap to let digipeaters reset

  // 2. Generate and Send Standalone Telemetry Packet
  char t_buff[60];
  int t_seq = TxCount % 1000;
  int t_batt = (int)(readBatt() * 100);             
  int t_temp = (int)(bmp.readTemperature() + 50);   
  
  float pressure_hPa = bmp.readPressure() / 100.0;
  int t_press = (int)sqrt(pressure_hPa / 0.016);
  if (t_press > 255) { t_press = 255; }             
  
  int t_sats = (int)myGPS.getSIV();                 

  // Notice no T# here. Our updated library prepends it automatically.
  sprintf(t_buff, "%03d,%03d,%03d,%03d,%03d,%03d,00000000", 
          t_seq, t_batt, t_temp, t_press, t_seq, t_sats);

  PttON;
  delay(500);  
  APRS_sendTelemetry(t_buff);
  delay(10);
  PttOFF;

  // Power Down
  RadioOFF;
  delay(1000);
  
  SerialUSB.print(F("Location & Telemetry sent - TxCount: "));
  SerialUSB.println(TxCount);  
  TxCount++;
}

#ifdef ENABLE_TELEMETRY_DEFS
void sendTelemetryDefinitions() {
  SerialUSB.println(F("Sending Telemetry Definitions (PARM/UNIT/EQNS)..."));
  
  char myCall[10];
  sprintf(myCall, "%s-%d", CallSign, CallNumber);
  char paddedCall[10];
  sprintf(paddedCall, "%-9s", myCall); 

  char msgBuff[60];

  if (readBatt() > DraHighVolt) RfHiPwr; 
  else RfLowPwr; 
  RadioON;
  delay(2000); 

  // 1. Send PARM
  PttON; 
  delay(500); 
  sprintf(msgBuff, "%s:PARM.Batt,Temp,Press,TxC,Sats", paddedCall); 
  APRS_sendMsg(msgBuff);
  delay(10); 
  PttOFF;

  delay(1000); 

  // 2. Send UNIT
  PttON; 
  delay(500);
  sprintf(msgBuff, "%s:UNIT.V,degC,hPa,cnt,cnt", paddedCall); 
  APRS_sendMsg(msgBuff);
  delay(10); 
  PttOFF;

  delay(1000); 

  // 3. Send EQNS
  PttON; 
  delay(500);
  sprintf(msgBuff, "%s:EQNS.0,0.01,0,0,1,-50,0.016,0,0,0,1,0,0,1,0", paddedCall); 
  APRS_sendMsg(msgBuff);
  delay(10); 
  PttOFF;

  RadioOFF;
  SerialUSB.println(F("Telemetry Definitions Sent."));
}
#endif

void sendStatus() {
  SerialUSB.println(F("Status sending..."));
  if (readBatt() > DraHighVolt) RfHiPwr; //DRA Power 1 Watt
  else RfLowPwr; //DRA Power 0.5 Watt
  RadioON;
  delay(2000);
  PttON;
  delay(1000);  
  APRS_sendStatus(StatusMessage);
  delay(10);
  PttOFF;
  RadioOFF;
  delay(1000);
  SerialUSB.print(F("Status sent - "));
  SerialUSB.println(TxCount);
  TxCount++;
}

void gpsDebug() { 
#if defined(DEVMODE)
    byte fixType = myGPS.getFixType();
    SerialUSB.print(F("FixType: "));
    SerialUSB.print(fixType);    

    int SIV = myGPS.getSIV();
    SerialUSB.print(F(" Sats: "));
    SerialUSB.print(SIV);

    float flat = myGPS.getLatitude() / 10000000.f;    
    SerialUSB.print(F(" Lat: "));
    SerialUSB.print(flat);    

    float flong = myGPS.getLongitude() / 10000000.f;    
    SerialUSB.print(F(" Long: "));
    SerialUSB.print(flong);        

    float altitude = myGPS.getAltitude() / 1000;
    SerialUSB.print(F(" Alt: "));
    SerialUSB.print(altitude);
    SerialUSB.print(F(" (m)"));

    float speed = myGPS.getGroundSpeed();
    SerialUSB.print(F(" Speed: "));
    SerialUSB.print(speed * 0.00194384f);
    SerialUSB.print(F(" (kn/h)"));    
        
    SerialUSB.print(" Time: ");    
    SerialUSB.print(myGPS.getYear());
    SerialUSB.print("-");
    SerialUSB.print(myGPS.getMonth());
    SerialUSB.print("-");
    SerialUSB.print(myGPS.getDay());
    SerialUSB.print(" ");
    SerialUSB.print(myGPS.getHour());
    SerialUSB.print(":");
    SerialUSB.print(myGPS.getMinute());
    SerialUSB.print(":");
    SerialUSB.print(myGPS.getSecond());
    
    SerialUSB.print(" Temp: ");
    SerialUSB.print(bmp.readTemperature());
    SerialUSB.print(" C");
    
    SerialUSB.print(" Press: ");
    SerialUSB.print(bmp.readPressure() / 100.0);
    SerialUSB.print(" hPa");
    SerialUSB.println();  
#endif
}

void setupUBloxDynamicModel(dynModel newDynamicModel) {
  // If we are going to change the dynamic platform model, let's do it here.
  // Possible values are:
  //DYN_MODEL_PORTABLE //Applications with low acceleration, e.g. portable devices. Suitable for most situations.
  //DYN_MODEL_STATIONARY //Used in timing applications (antenna must be stationary) or other stationary applications. Velocity restricted to 0 m/s. Zero dynamics assumed.
  //DYN_MODEL_PEDESTRIAN   //Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed.
  //DYN_MODEL_AUTOMOTIVE   //Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed
  //DYN_MODEL_SEA        //Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed.
  //DYN_MODEL_AIRBORNE1g   //Airborne <1g acceleration. Used for applications with a higher dynamic range and greater vertical acceleration than a passenger car. No 2D position fixes supported.
  //DYN_MODEL_AIRBORNE2g   //Airborne <2g acceleration. Recommended for typical airborne environments. No 2D position fixes supported.
  //DYN_MODEL_AIRBORNE4g   //Airborne <4g acceleration. Only recommended for extremely dynamic environments. No 2D position fixes supported.
  //DYN_MODEL_WRIST      // Not supported in protocol versions less than 18. Only recommended for wrist worn applications. Receiver will filter out arm motion.
  //DYN_MODEL_BIKE       // Supported in protocol versions 19.2

    if (myGPS.setDynamicModel(newDynamicModel) == false) // Set the dynamic model to DYN_MODEL_AIRBORNE4g
    {
      #if defined(DEVMODE)
        SerialUSB.println(F("***!!! Warning: setDynamicModel failed !!!***"));
      #endif 
    }
    else
    {
      ublox_high_alt_mode_enabled = true;
      #if defined(DEVMODE)
        SerialUSB.print(F("Dynamic platform model changed successfully! : "));
        SerialUSB.println(myGPS.getDynamicModel());
      #endif  
    }
} 

float readBatt() {
  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0f;
  do {    
    value =analogRead(BattPin);
    value +=analogRead(BattPin);
    value +=analogRead(BattPin);
    value = value / 3.0f;
    value = (value * 1.65) / 1024.0f;
    value = value / (R2/(R1+R2));
  } while (value > 20.0);
  return value ;
}

void freeMem() {
#if defined(DEVMODE)
  SerialUSB.print(F("Free RAM: ")); SerialUSB.print(freeMemory(), DEC); SerialUSB.println(F(" byte"));
#endif
}