#include <Adafruit_INA260.h>
#include <HTTPS_VIPER.h>
#include <HydraProbe.h>
#include <Timezone.h>
#include <Time.h>
#include <TimeLib.h>
#include <SoftwareSerial.h>
// Tim's Version
/* * Teensy 3.5 Custom PCB developed to automate ISCO 6700 Sampling with a digital output pin
   Unit is controlled via SMS commands
   Bottle number and time sampled is parsed from ISCO Serial Output
   Data including bottle number, water level and fixed GPS coordinates are streamed to VIPER database using HTTP Post from a FONA 808 cellular module
   Time, bottle number and water level are saved to SD card locally
   Water level (presence) is detected by an optical sensor (Optomax Digital Liquid Level Sensor)
   Auto Sampling Mode - Pulses pin every (grabSampleInterval) minutes when water is present in the collection basin
   Grab Sampling Mode - Pulses pin whenever a grab sample is initiated by SMS command


  Hawes Collier
*/
//094 Pod code Version 0.1 - Cleaned up working code and added watchdog to testing unit. 2/21/18 - Tim McArthur
//Version 1.0  - Cleaned up variable/function names and logic. 4/11/2018 - Tim McArthur
//Version 1.01 - Added static set of numbers to accept commands from
//             - Default starts in grab mode
//             - Grab sample ignores water level. Still depends on ISCO being enabled
//Version 1.02 - Change auto sampling interval via text message
//Version 1.03 - Saving sampling mode and interval to EEPROM to hold on restart - 4/25/2018 - Tim McArthur
//             - Writing more information to SD card\
//Version 1.04 - Daylight Savings time, confirm Google number functions properly.
//Version 1.08 - Fixes for VIPER and Texting
//Version 1.09 - Reverting serial functionality and bottle number tracking
//Version 1.10 - Minor changes to Davis calculations. Added REPL reboot to fix major bug which was causing significant downtime.
//Version 1.11 - Added ISCO rain bucket code. Modified Davis rainbucket code to move off non-functional I2C bus - THM
//Version 1.12 - Minor fixes for rainbucket calculations, immediate VIPER post command added (3), other general bug fixes - THM
//Version 1.13 - Added flowmeter code - THM
//Version 1.20 - Added additional text functionality. Deployed to shed row 6/11/2021 - THM
//Version 1.21 - Added handshake protocol for Teensy to XBee Serial communication. Results are very promising - 6/14/2021 THM
//Version 1.22 - Changed VIPER descriptor for Moisture Sensors - 06222021 - THM Field deployed in Elizabeth City
//Version 1.23 - Fixed bug in ISCO tipping bucket logic. Wasnt able to reset to 0. 7/6/2021 - THM
//Version 1.24 - Includes bug fixes for parsing ISCO 6712 rainbucket values. Averaging for flowrate and rain intensity implemented.THM
//Version 1.25 - Minor fix for Daily Rain values (allows 0 as valid) THM
//Version 1.26 - Sampler will now restart in whatever sampling mode it was before it restarted instead of defaulting to GRAB THM
//GPS and unit values used in VIPER Post

//char GPS[] = "40.5087450, -74.3580650 0"; //needs a (space 0) at the end, example ="35.068471,-89.944730 0"
int unit = 7;
char versionNum[] = "_094_Pod_ISCOv1.26";

//////////////////////////////////


// DAYLIGHT SAVINGS TIME
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    // Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     // Standard time = UTC - 5 hours
Timezone myTZ(myDST, mySTD);
/////////

//Watchdog
#include <Adafruit_SleepyDog.h>
int errorCount;
//EEPROM
#include <EEPROM.h>
int eepromModeAddr = 0;
int eepromIntervalAddr = 1;
int eepromfirst_upload = 2;
int is_first = 0;
#define FIRST 1
//////////////////////////////////////////////

//Char arrays to hold time information

char monthChar[5];
char dateChar[5];
char hourChar[5];
char minuteChar[5];
char secondChar[5];

///////////////////////////////////////

//Char arrays to hold measurement values for posting to VIPER
char Temp1Char[10];
char Temp2Char[10];
char Temp3Char[10];
char levelChar[10];
char bVChar[10];

///////////////////////////////////////
//Static numbers to text when bottles are full and accept SMS comands from:

bool successTimText = false; //logic to control when to text bottle full message
int readSMSSuccess = 0;
int deleteSMSSuccess = 0;
int sender_count = 0;
/////////////////////////

/////////////////////////Parsivel
int lastParsivel;
int Parsiveltime;
const int SSERIAL_RX_PIN = 10;  //Soft Serial Receive pin
const int SSERIAL_TX_PIN = 11;  //Soft Serial Transmit pin
const int SSERIAL_CTRL_PIN = 3;  //RS485 Direction control
const int RS485_TRANSMIT = HIGH;
const int RS485_RECEIVE = LOW;
String parsivel_data;
char* parsivel_intensity;

int byteReceived;
int byteSent;
/////////////////////////


//////Flowmeter
int totalMilliLitres = 0;
float flowrate = 0;
float flowrateArray[50];
int flowrateIndex = 0;
/////

//////////////////////////Rain Bucket   **ISCO rain bucket data comes over the ISCO serial comms. Davis data is communicated over Serial (Serial3 on Master Teensy, Serial4 on slave Teensy)

///Rain bucket  each tip is 0.01inch
#define ISCORain 0
#define DAVIS 1
#define FLOWMETER 0

unsigned long lastDavisCall = millis();
unsigned long lastFlowCall = millis() - 30000; //30 sec offset from rain period
int rain_period = 60000;
int flow_period = 60000;
///// ISCO only, gets previous and current tips to calculate delta
int cr = 0; //current number of tips (rolls over at 256)
int pr = 0; //previous number of tips
///
int tips = 0; //Davis only. Tips are cleared each poll of davis sensor
int dailyTips = 0; //number of tips today    (rolls over at 256)
int lastDaily = 0; //ISCO only. Used to calculate the number of daily tips since last time data was saved
double intensity = 0; //  (mm/hr)
double intensityArray[50];
int intensityIndex = 0;
unsigned long lastChange = millis();
bool firstIntensity = true;
////

//DAVIS RAINBUCKET


//uses same variables as ISCO rainbucket
////////
///////////////I2C Comms //No more I2C comms, but some of this looks like droplet stuff. Leaving for now
int rain_level = 1; //1 is no rain, 2 is little, 3 is lots of rain
int droplet_pin = A14; // pin 33
//INA260 comms Placeholder
//////////////END I2C Inits


//Hydraprobes
unsigned long lastProbe = millis();
int Probetime = 60000;
float temp = 0, moisture = 0, conductivity = 0, permittivity = 0;
float temp2 = 0, moisture2 = 0, conductivity2 = 0, permittivity2 = 0;
float temp3 = 0, moisture3 = 0, conductivity3 = 0, permittivity3 = 0;
HydraProbe moistureSensor;  //define data Pin in Header (library .h file)
HydraProbe moistureSensor2;
HydraProbe moistureSensor3;
int HPConnected = 0;
//
//Xbee Socket connection
HTTPS_VIPER http = HTTPS_VIPER();
char senderNum[30];    //holds number of last number to text SIM
char replybuffer[255]; //holds Fonas text reply
char * replybuffer_command; //holds Fonas command part
char * replybuffer_interval; //holds Fonas interval part
#define xbeeSerial Serial1
//#define parsivelSerial Serial2
int commandNum = -1; //integer value for which text command has been sent
unsigned long lastPost = millis();
float minsToPost = 5;
int numPulse = 1; //number of signal pulses to send ISCO per sample
//Timing for Samples
boolean sample_occurred = false; // goes true when a sample occurs
long sample_start = 0;  // keeps time for sample start
long sample_stop = 0; // once this
long sample_stop_time = 5;  // 5 minutes
long sample_period = 8;// 8 hours
int  samples_allowed = 2;
unsigned long text_time = 0;
unsigned long text_period = 5000;//15000
boolean part2 = false; //to send 2nd half of text message for large messages
////////////////////////////

//Voltage Readings
#define BattRail_VIN A1 //Pin 15
float BattVoltage;
#define FiveRail_VIN A9
float FiveVoltage;
//#define ThreeRail_VIN A5
#define levelPin A20
//float ThreeVoltage;
///////////////////////

//SD
String inputData;
char bottleNumChar[10];
char bottleInfo[100]; // array holds last sampled bottle info
char msgTxt[255]; // array holds last sampled bottle info
char lastBottleInfo[100];
int writeNum; // integer to control whether or not the header data is written to SD file
char fileName[20];  //fileName built from date
char fileNameLog[25];
char dateTime[30];  // dateTime to be written to SD card header
char dateTimeVIPER[40]; //dateTime for VIPER string
#include <SD.h>
const int chipSelect = BUILTIN_SDCARD; //For Teensy 3.5, SPI for SD card is separate
File logFile;
File myFile;
int minsToSave = 1;
int date;  //holds current date so new file can be written at midnight
unsigned long lastSave = millis();  // holds time since execution of last save to ensure data is saved every 60 seconds

///////////////SD

//EEPROM timing
unsigned long ident_save = millis();
int timer_ident = 60; //mins
bool first_eeprom = true;
int ident_ADR = 4;
int ident = 1;
//

//ISCO
#include <Time.h>
#define IscoSamplePin 16
#define version2 1
#define version1 0
bool accepted = false;  // Flag gets set to true when we see > character from ISCO.
//  bottle_limit = 23; Do not exceed this!
int bottle_number = 1;
char iscoData[500];
int cdIndex = 0;
bool autoSampleMode = false;
int grabSampleInterval = 2 ; //Minute interval to auto grab sample as long as water level is high enough
char iscoTime[15]; //holds last bottle sample time from ISCO memory
#define iscoSerial Serial4
bool timerOn = false;//logic for texting after sample is complete
unsigned long textTimer = millis();
unsigned long sampleTimer = millis();
bool ISCORail = true;
int lastSDAvg = 0;
int levelReading;
bool iscoConnected = false;
unsigned long iscoTimeout = millis();
/////////////////ISCO

bool SDcard = true;
///////////////////////////RELAYS
bool RQ30 = false;
//Soil moisture is on pin 30
#define HPdata 30
#define HPRelay 27 //Outpin PIN to control HydraProbe 12V Rail
#define RQ30Relay 26 //Output PIN to control RQ30 12V Rail
//unsigned long fonaTimer = millis(); //timer to reset Fona
unsigned long lastRQ = millis();
unsigned long RQTime = 60000 * 5;
//////////////////////////RELAYS
Adafruit_INA260 ina260 = Adafruit_INA260();
boolean ina260Available = false;

// SMS flags
bool rec = false;

unsigned long timeout = 0;
boolean sendUpdate = false; //true if waiting to receive new bottle info after sampling event
boolean newBottle = false; //true if a new bottle was parsed from serial

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  Wire.begin();
  errorCount = 0;
  int countdownMS = Watchdog.enable(120000); //120 seconds or 2 minute watchdog timer
  Serial.print("Enabled the watchdog with max countdown of ");
  Serial.print(countdownMS, DEC);
  Serial.println(" milliseconds!");
  Serial.println();
  // SETUP DST
  setTime(myTZ.toUTC(compileTime()));

  delay(2000);
  setSyncProvider(getTeensy3Time);
  Serial.print("Initializing SD card...");  //startup SD card

  if (!SD.begin(chipSelect)) {
    SDcard = false;
    Serial.println("initialization failed!");
  }
  Serial.println("SD Card initialization done.");
  Serial.print("Code version "); Serial.println(versionNum);

  pinMode(HPRelay, OUTPUT); //Set up HydraProbe relay

  //pinMode(levelPin, INPUT);
  delay(50);

  pinMode(BattRail_VIN, INPUT);
  digitalWrite(HPRelay, HIGH); //turn on HydraProbe 12V Rail
  Watchdog.reset();
  //setup_parsivel();// Setup Parsivel output string

  /* #if FIRST  //change FIRST to 1 when uploading and running first time
    EEPROM.write(eepromfirst_upload, 0);
    is_first = 1;
    #endif
    if (!is_first)
    EEPROM.get(eepromfirst_upload, is_first);

    if(!is_first)
    { // if not fresh upload, take old values
    EEPROM.get(eepromModeAddr, grabSampleMode); //Get Sampling mode from EEPROM
    EEPROM.get(eepromIntervalAddr, grabSampleInterval); //Get sampleInterval from EEPROM
    }
    else {
     grabSampleMode = false;
     grabSampleInterval = 1;
    }
  */




  ////////////////////////// FIRST TIME UPLOAD, GET THE IDENTITY FOR THIS WETBOARD SET AT 1 OR 0
  int first_up = 0;

  if (first_up) {
    EEPROM.write(ident_ADR, ident); // This sets the identity number we will be sending over to the Xbee to prevent the VIPER database from overlapping old data.  This updates every 2 hours onto EEPROM atm
  }
  else {
    ident = EEPROM.read(ident_ADR); // This reads the identity number we will be sending over to the Xbee to prevent the VIPER database from overlapping old data.  This updates every 2 hours onto EEPROM atm
  }

  // if grabSample is in true this doesn't do anything


  EEPROM.get(eepromModeAddr, autoSampleMode); //Get Sampling mode from EEPROM 

  if (autoSampleMode == true)
  {
    Serial.println("AUTO Sample Mode from Memory");
  }else
  {
    Serial.println("GRAB Sample Mode");
  }
  EEPROM.get(eepromIntervalAddr, grabSampleInterval); //Get sampleInterval from EEPROM

  if (grabSampleInterval < 480 && grabSampleInterval > 0)
  {
    Serial.print("Interval is "); Serial.print(grabSampleInterval); Serial.println("minutes");
  } else
  {
    grabSampleInterval = 20;
    Serial.print("Interval defaulted to "); Serial.print(grabSampleInterval); Serial.println("minutes");
  }

  xbeeSerial.begin(115200);
  //Startup SMS and Cell client DO NOT CHANGE THIS BAUD RATE UNLESS YOU CHANGE XBEE BAUD RATE WITH XCTU
  //  Future design notes for Wiley and team:
  //  If you can setup serial1 for the alternative UART that Xbee uses with its UART class, we may have more fine control functionality
  //  At the moment, we can only use the Xbee as so:
  // 1. Change baud rate to speed up slow down serial comms
  // 2. read in data
  // 3. write out data
  // We want to be able to do more functions on data later if this ever gets a 2.0



  pinMode(IscoSamplePin, OUTPUT);  // Setup sample pin output to make ISCO sample when function is called to flip this pin
  digitalWrite(IscoSamplePin, LOW);
  //Serial.begin(115200);
  iscoSerial.begin(9600); //Setup comms with ISCO

  Watchdog.reset();
  //delay(5000);
  //massSMS("Testing WET Board mass Text (from Teensy)");
  moistureSensor.debugOn(); //Can turn debug on or off to see verbose output (OR NOT)
  if (moistureSensor.begin(1))
  {
    ++HPConnected;  //Red tape
  }
  moistureSensor2.debugOn(); //Can turn debug on or off to see verbose output (OR NOT)
  if (moistureSensor2.begin(2))
  {
    ++HPConnected;  //Yellow tape
  }
  moistureSensor3.debugOn(); //Can turn debug on or off to see verbose output (OR NOT)
  if (moistureSensor3.begin(3))
  {
    ++HPConnected;  //Green tape
  }


  /*
     01234567
     02134567
     03142567
  */


  if (!ina260.begin()) {  // Check to see if  I2c on XBEE is on if there is no comms.  This messed up the INA.
    Serial.println("Couldn't find INA260 chip");
    ina260Available = false;
  }
  else {
    Serial.println("Found INA260 chip!");
    ina260Available = true;
  }
  Watchdog.reset();

  connectToISCO();

}
void loop() {
  timeout = 0;
  Watchdog.reset();
  do
  {
    delay(20);
    Watchdog.reset();
    ++timeout;
  } while (iscoSerial.available() == 0 && timeout < 1000);

  int returned = getBottleNumber();
#if ISCORain
  
  if (getDaysRain() > -1)
  {
    dailyTips = getDaysRain();
    Serial.print("Today's Rain is " ); Serial.println(dailyTips);
  }
#endif
  if  (returned > 0 && returned < 25)
  {
    Serial.println("Valid bottle Number");
    bottle_number = returned;
    if (sendUpdate)
    {
      Serial.println("send update is true");
      sendUpdate = false;
      if (newBottle)
      {
        //  delay(30000);
        Serial.println("Sending update SMS");
        sendSMS(bottleInfo);
      }
    }
  }
  Serial.print("Current bottle Number = "); Serial.println(bottle_number);

  //checkUserInput();
  rec = false;

  unsigned long currentTime = millis();


  Watchdog.reset();
  //checkTexts(); //check for SMS commands
  //delay(1000);
  Serial.print("Droplet reader level (1-3) is ");
  Serial.println(droplet_read());
#if version1
  /*if (ISCORail) // this is true at start
    {
    Serial.print("Started waiting for ISCO...Millis = "); Serial.println(millis()); //If ISCO is enabled, reset the watchdog while waiting for data on ISCO Serial
    do {
       //request_ISCO();
    } while (iscoSerial.available() == 0 && millis() - iscoTimeout < 40000); //If there is data on the serial line, OR its been 40 seconds, continue
    iscoTimeout = millis(); //reset timer
    Serial.print("Finished waiting for ISCO...Millis = "); Serial.println(millis());
    do {  //read in data from ISCO Serial
      //readIscoSerial(0);
    } while (iscoSerial.available() > 0);
    } else
    {
    // checkTexts();
    delay(5000); //If ISCORail is disabled , wait 5 seconds every loop
    }
  */
#endif
  //Serial.println("Checking texts...");
  //checkTexts(); //check for SMS commands
  Watchdog.reset(); //ensure the system doesn't prematurely reset


  checkUserInput(); //check Serial for input commands
  Serial.print("Batt V ="); Serial.println(getBV()); //get voltage rail readings
  BattVoltage = getBV() / 1000.0;


  if (currentTime - lastPost > minsToPost * 60000) //Post data every (minsToPost) minutes
  { // at the moment we post every 1 minute
    checkTexts(); //The post process consumes any queued texts, so run them first

    delay(2000);
    Serial.println("----------");
    Serial.println("AVERAGING");
    Serial.println("----------");
    averageData();
    postData();
    http.clearData();
    lastPost = millis(); //reset timer
    //clearIscoSerial(); //clear isco and fona serials
  } else
  {
    Serial.print(((minsToPost * 60000) - (currentTime - lastPost)) / 1000); Serial.println(" Seconds left b4 Post");
  }


#if FLOWMETER

  unsigned long currentMillis = millis();
  if (currentMillis - lastFlowCall > flow_period) { // if it is time to get flowmeter data
    lastFlowCall = currentMillis;
    Serial.println("Checking Flowmeter");
    getFlowMeter();

  }
#endif


#if DAVIS

  unsigned long currentMillis = millis();
  if (currentMillis - lastDavisCall > rain_period) { // if it is time to get Davis data
    lastDavisCall = currentMillis;
    Serial.println("Checking Davis data");
    I2C_rain();

  }
#endif

  currentTime = millis();
  if (currentTime - text_time >  text_period) {

    checkTexts();

    text_time = millis();
    delay(1000);
    Serial.println("Made it past Texts");
  }


  //Watchdog.reset();
  if (currentTime - lastProbe >  Probetime) {
    checkHydraProbes();
    lastProbe = millis();
  }


  if (millis() - lastSave > minsToSave * 60000) //Save data every (minsToSave) minutes (1 mins)
  {

    Serial.println("----------");
    Serial.println("SAVING");
    Serial.println("----------");
    
    if (SDcard) saveData();
    lastSave = millis();
  }



  if (!autoSampleMode) //Display sample mode on Serial Monitor
  {
    Serial.println("------------------Currently in GRAB Sample Mode--------------------");
  } else
  {
    Serial.println("------------------Currently in AUTO Sample Mode--------------------");


    if (millis() - sampleTimer > 60000 * grabSampleInterval) //.....AND....it's time to sample...
    {
      Serial.println("Timer finished. Ready to sample");
      if (getWaterLevel()) //If there is water in the pan......
      {
        Serial.println("Water detected.");
        sampleTimer = millis();
        if (ISCORail) //if true collect a sample
        {
          toggleSample(); // collect if available
        }
      } else
      {
        Serial.println("No water in the pan. Waiting for water to sample");
      }
    } else
    {
      Serial.print("Sample timer has "); Serial.print(((60000 * grabSampleInterval) - (millis() - sampleTimer)) / 1000); Serial.println("Seconds remaining");
    }
  }

  if (getWaterLevel()) //If there is water in the pan......
  {
    Serial.println("Water detected.");
  }





  iscoData[cdIndex] = '\0'; //null out data
  cdIndex = 0;

  //checkTexts();

  if (millis() - ident_save > timer_ident * 2 *  60000) //Save ident once per 2 hours
    //EEPROM
  {
    Serial.println("----------");
    Serial.println("Saving Ident for Viper push");
    Serial.println("----------");
    ident_save = millis();
    EEPROM.write(ident_ADR, ident);
  }
  //checkTexts();
  Serial.println("Here_end_loop");

  Watchdog.reset();




}

#if FLOWMETER
char flowIn[50];
char flowRateIn[10];
char milsIn[12];
void getFlowMeter() {

  //Serial3.println("R");
  //Serial.println("Sending data request");

  unsigned long timeout = 0;
  do {
    ++timeout;
    delay(1);
  } while (!Serial3.available() && timeout < 2000);
  delay(400);

  if (Serial3.available())
  {
    Serial.println("Reading in Flow Data");
    int index = 0;
    do {
      flowIn[index] = Serial3.read();
      Serial.print(flowIn[index]);
      ++index;

    } while (Serial3.available());
    flowIn[index] = '\0';

  } else
  {
    Serial.println("Serial not available");
  }

  char *flowRateChar = subStr(flowIn, ",", 1);
  char *milsChar = subStr(flowIn, ",", 2);


  Serial.println(flowRateChar);
  Serial.println(milsChar);


  flowrateArray[flowrateIndex] = atof(flowRateChar);
  totalMilliLitres = atoi(milsChar);

  Serial.print("flowRate = "); Serial.println(flowrateArray[flowrateIndex]);
  ++flowrateIndex;
  Serial.print("totalMils = "); Serial.println(totalMilliLitres);


  Serial3.flush(); // clearSerialData

}


#endif


void averageData()
{
  double intensitySum = 0;
  for (int x = 0 ; x < intensityIndex+1 ; ++x)
  {
    intensitySum = intensitySum + intensityArray[x];
  }
  intensity = intensitySum / intensityIndex + 1;
  if (intensity > 300)//Greatest rain Intensity ever recorded
  {
    Serial.println("Intensity too high, must be first record");
    intensity = 0;
  }
  intensityIndex = 0;
  lastSDAvg = 0;

  float flowrateSum = 0;
  for (int x = 0 ; x < flowrateIndex ; ++x)
  {
    flowrateSum = flowrateSum + flowrateArray[x];
  }
  flowrate = flowrateSum / flowrateIndex + 1;
  flowrateIndex = 0;
}

char* parse_Intensity(String message) {

  int len = message.length();
  int indx_semi_1 = 0;
  int indx_semi_2 = 0;
  int i = 0;
  int count = 0;
  String mess;
  char* intensity;
  for ( i = 0; i < len; i++)
    if (message.charAt(i) == ';')
      count++;
  if (count == 1)
    indx_semi_1 = i;
  if (count == 2) {
    indx_semi_2 = i;
    i = len;
  }
  mess = message.substring(indx_semi_1, indx_semi_2);
  int leng = mess.length();
  mess.toCharArray(intensity, leng + 1);
  return intensity;
}


void checkTexts() //reads SMS(s) off the Fona buffer to check for commands
{
  /*
     Variables to hold message data before sending
  */
  int msg_int = 0;
  int ind = 0;

  String msg = "";
  char return_msg[200];
  sprintf(return_msg, "%s", "");

  /* removed polling - Xbee will automatically send data
    if (!rec)
    {
    xbeeSerial.flush();
    xbeeSerial.print("?!"); //Sends a ? to the XBEE, prompting the Xbee to send over the Text data.
    delay(20);
    xbeeSerial.flush();
    }
  */


  if (xbeeSerial.available() > 0) {
    msg = xbeeSerial.readString();
    msg = msg.trim();
    msg.replace(" ", "");
    msg.replace("\r", "");
    msg.replace("\n", "");
    ind = msg.length();
    rec = true;
  }
  //Serial.print("msg is: " + msg + " length is ");
  //Serial.println(msg.length());
  if (ind == 0) {
    //Serial.println("No valid messages");
    return;
  }
  else {
    Serial.print("message is ");
    Serial.println(msg);
    msg.toCharArray(msgTxt, msg.length());
    writeMsgTxt();
    if (msg == "?!")
    {
      ++errorCount;
    }
    if (errorCount > 5) //force watchdog timeout
    {
      Serial.print("Forcing restart");
      xbeeSerial.print('$'); //send handshake character

      int serialTimeout = 0; //wait for handshake character
      do {
        delay(1);
        ++serialTimeout;
      } while (!xbeeSerial.available() && serialTimeout < 3000);

      if (serialTimeout > 2998)
      {
        Serial.println("No handshake received, timeout");
      }


      if (xbeeSerial.read() == '$') //if handshake received, sent first portion of data
      {
        Serial.println("Received handshake");
        xbeeSerial.print("r!");
      }
      //  do {
      //    delay(2000);
      //    Serial.print(".");
      //  } while (true);
    }
    Serial.print("Length is : ");
    Serial.println(msg.length());
    msg_int = msg.toInt();
    Serial.println(msg_int);
  }
  // may need to add \0 later
  if (msg == "?!")
    return;
  if (msg == "0") {
    // create reset text?
  }

  if (msg.length() > 30)
  {
    xbeeSerial.println("xbee.atcmd('FR')");//We are in REPL mode after an error caused a break in execution. Reset the module
  } else {

    if (msg_int == 1 && msg.length() < 5 ) {

      float batt = getBV(); // call for battery
      //String battString = String(batt);
      sprintf(return_msg, "%s%f", batt, " mV");
      sendSMS(return_msg);
      rec = true;
      Serial.println("Found something!");
    }

    else if (msg_int == 2 && msg.length() < 5) {
      Sample();
      // sprintf(return_msg, "%s", "Sampling");
      // delay(1000);
      // sendSMS(return_msg);
      rec = true;
    }

    else if (msg_int == 3 && msg.length() < 5) { //Change command 3 to attempt VIPER post

      lastPost = -1000000;
      sprintf(return_msg, "%s", "Attempt post");
      sendSMS(return_msg);

      rec = true;
    }

    else if (msg_int == 4 && msg.length() < 5) {
      if (!autoSampleMode)
      {
        autoSampleMode = true;
        EEPROM.write(eepromModeAddr, true);
        sprintf(return_msg, "%s%i%s", "Automatic Sampling Mode. Interval - ", grabSampleInterval , " minutes");
        sendSMS(return_msg);

      } else
      {
        autoSampleMode = false;
        EEPROM.write(eepromModeAddr, false);
        sprintf(return_msg, "%s", "Grab Sample Mode");
        sendSMS(return_msg);
      }
      rec = true;
    }

    else if (msg.substring(0, 2) == "5_") { //This command is the one to change the Sample interval of the ISCO //was 0,3 - (4).toInt
      int new_time = 0;
      new_time = msg.substring(2).toInt();
      grabSampleInterval = new_time;
      EEPROM.put(eepromIntervalAddr, grabSampleInterval);
      sprintf(return_msg, "%s%i%s", "Auto Sample interval set to ", grabSampleInterval, " minutes");
      sendSMS(return_msg);
      rec = true;
    }

    else if (msg_int == 6)
    { // change mins to 2
      minsToPost = 2;
      sprintf(return_msg, "%s", "Post interval set to 2 minutes");
      sendSMS(return_msg);
      rec = true;
    }

    else if (msg_int == 7) { //change mins to 5
      minsToPost = 5;
      sprintf(return_msg, "%s", "Post interval set to 5 minutes");
      sendSMS(return_msg);
      rec = true;

    }

    else if (msg_int == 8 ) {
      if (part2)
      {
        Serial.println("***part2 is true****");
        char rainbucket[10];
        sprintf(rainbucket, "%s", "None");
#if DAVIS
        sprintf(rainbucket, "%s", "Davis");
#endif
#if ISCORain
        sprintf(rainbucket, "%s", "ISCO");
#endif
        char flowmeter[10];
        sprintf(flowmeter, "%s", "No");
#if FLOWMETER
        sprintf(flowmeter, "%s", "Yes");
#endif
        sprintf(return_msg, "%s%s%s%.2f%s%s%s%s%.2f%s", "Rainbucket: ", rainbucket, "\nRain Intensity: ", intensity, " mm/hr", "\nFlowmeter: ", flowmeter, "\nFlowrate: ", flowrate, " Lpm");
        sendSMS(return_msg);
        // delay(30000);
        part2 = false;
        rec = true;
      } else
      {

        rec = true;
        char mode[10];
        if (!autoSampleMode)
        {
          sprintf(mode, "%s", "GRAB");
        } else
        {
          sprintf(mode, "%s", "AUTO");
        }
        char iscoStatus[20];
        if (iscoConnected)
        {
          sprintf(iscoStatus, "%s", "Connected");
        } else
        {
          sprintf(iscoStatus, "%s", "Not Connected");
        }

        sprintf(return_msg, "%s%s%s%i%s%s%.2f%s%s%s%s%i%s", "Mode: ", mode, "\nInterval: ", grabSampleInterval, " minutes", "\nBatt V: ", BattVoltage, " volts", "\nISCO: ", iscoStatus, "\nHydraprobes: ", HPConnected, " Connected");
        sendSMS(return_msg);
        part2 = true;

      }
    }
    else if (msg_int == 9 ) {
      rec = true;
      sprintf(return_msg, "%s", "Will reboot master Teensy");
      sendSMS(return_msg);
      do {} while (true);

    }
    /*
      else if (msg.substring(0,2)=="9_"){
      rec = true;
      sendSMS("No RQ30 Connected");
      }
      else if (msg.substring(0,3)=="10_"){
      rec = true;
      sendSMS("No Sensor Connected");
      }
    */

    else {
      Serial.println("No valid messages");
      return;
    }
  }

}





///////////////////////////////***********************Xbee FUNCTIONS
/*** SMS ***/

void sendSMS(String message) {
  // send an SMS!
  delay(500);
  Serial.print("Sending: ");
  Serial.println("C" + message);
  xbeeSerial.print('$'); //send handshake character

  int serialTimeout = 0; //wait for handshake character
  do {
    delay(1);
    ++serialTimeout;
  } while (!xbeeSerial.available() && serialTimeout < 3000);

  int timeNow = millis();

  if (serialTimeout > 2998)
  {
    Serial.println("No handshake received, timeout");
  }


  if (xbeeSerial.read() == '$') //if handshake received, sent first portion of data
  {
    delay(50);
    xbeeSerial.print("C" + message + "!");
    Serial.print("Responded to handshake after "); Serial.print(millis() - timeNow); Serial.println(" ms");
  }


  delay(500);
  xbeeSerial.flush();// added this to clean up serial port after sending text message.

}

void massSMS(String message) {
  // send an SMS!

  xbeeSerial.print('$'); //send handshake character

  int serialTimeout = 0; //wait for handshake character
  do {
    delay(1);
    ++serialTimeout;
  } while (!xbeeSerial.available() && serialTimeout < 3000);

  if (serialTimeout > 2998)
  {
    Serial.println("No handshake received, timeout");
  }


  if (xbeeSerial.read() == '$') //if handshake received, sent first portion of data
  {
    delay(50);
    Serial.println("Received handshake");
    xbeeSerial.print("A" + message + "!");
  }

}

void flushxbeeSerial() { // flush xbee Serial port
  //xbeeSerial.flush();
}

///////////////////////////////////////**********************FONA FUNCTIONS

void checkUserInput()
{
  Serial.println("Checking user input");
  if (Serial.available())
  {
    char a = Serial.read();
    Serial.print("char is: ");
    Serial.println(a);
    /*
      if (a == 'S')
      {
      #if version1
      toggleSample();
      #endif
      #if version2
      Sample();
      #endif
      }
      if (a == 'T')
      {
      }
      if (a == 'Q')
      {
      #if version1
      sendQuery();
      #endif
      }
      if (a == 'G')
      {
      delay(200);
      if (getBottleNumber() == 0)
      {
        Serial.println("No communication with ISCO. Could not find data");
      } else
      {
        Serial.print("Sampled -- Bottle ");
        Serial.print(getBottleNumber());
        if (getBottleNumber() == 24)
        {
          massSMS("All bottles are full. ISCO Sampler must be reset and bottles replaced");
        }
        unsigned long sampled = getSampledTime();
        //Serial.print("time_t=");Serial.println(sampled);
        Serial.print(" @ ");
        Serial.print(month(sampled)); Serial.print("/"); Serial.print(day(sampled)); Serial.print("/"); Serial.print(year(sampled)); Serial.print("  "); Serial.print(hour(sampled)); Serial.print(":"); Serial.print(minute(sampled)); Serial.print(":"); Serial.println(second(sampled));
      }
      }
    */
    if (a == 'X')
    {
      xbeeSerial.print("r!");
    }


  }

}



void toggleSample()
{

  char Message[50];


  Serial.println("Start sampler");
  textTimer = millis();
  timerOn = true;



  if (bottle_number > 24)
  {
    bottle_number = 1;
  }
  if (bottle_number == 1)
  {
    sprintf(Message, "%s%i", "Received Sample Command. Attempting to Sample Bottle # ", bottle_number);
  } else
  {
    sprintf(Message, "%s%i", "Received Sample Command. Now Sampling Bottle # ", bottle_number + 1);
  }
  // sendSMS(Message);

  Sample();


}


void Sample()
{
  sendUpdate = true;
  /*
    request_ISCO();
    if (accepted){
    iscoSerial.print("7\r");// This should use the menu command: TAKE_SAMPLE or 7
    //iscoSerial.println("TAKE_SAMPLE\r");
    }
  */
  if (bottle_number <= 23) {
    bottle_number++;
  } else {
    sendSMS("Bottles may need to be reset. If already replaced, you should not receive this message again once a new bottle is sampled");
  }
  for (int i = 0; i < numPulse; i++) {
    digitalWrite(IscoSamplePin, HIGH);
    delay(300);
    digitalWrite(IscoSamplePin, LOW);
    delay(100);
  }


}
#if version1
void sendQuery() //Function to establish comms with ISCO if needed
{
  Serial.println("Sending query to ISCO");
  iscoSerial.write('?');
  iscoSerial.write(13);
  delay(2000);
  if (!iscoSerial.available())
  {
    iscoSerial.write('?');
    iscoSerial.write(13);
    delay(2000);
    Serial.println("Trying another baud...");
  } else
  {
    readIscoSerial();
  }
  if (!iscoSerial.available())
  {
    iscoSerial.write('?');
    iscoSerial.write(13);
    delay(10);
    Serial.println("3rd baud attempt...");
  } else
  {
    readIscoSerial();
  }
  delay(2000);
  if (!iscoSerial.available())
  {
    Serial.println("No data available...giving up");
  }

}
#endif

void cleararray(char* a) {
  for (int x = 0; x < sizeof(a) / sizeof(a[0]); x++)
  {
    a[x] = 0;
  }
}

void clearIscoSerial()
{
  Serial.println("Start Clearing ISCO");
  for (int x = 0; x < 64 ; ++x)
  {
    if (iscoSerial.available())
      char a = iscoSerial.read();

  }
  Serial.println("Done clearing ISCO");
}
/*
  #if version1
  void readIscoSerial() //CHECK FOR OVERFLOW ON THIS ARRAY
  {
  unsigned long timer = millis();
  do
  {
    if (iscoSerial.available())
    {
      char a = iscoSerial.read();
      if (cdIndex < sizeof(iscoData))
      {
        iscoData[cdIndex] = a; //here might need to ring buffer
      }
      ++cdIndex;
    }


  } while (millis() - timer < 1000);
  }
  #endif

  //Global to see if the menu is open on the ISCO.
  #if version2
  void readIscoSerial(int comm) //CHECK FOR OVERFLOW ON THIS ARRAY
  {

  String data = "";
  unsigned long timer = millis();

  do
  {
    if (iscoSerial.available())
    {
      char a = iscoSerial.read();
      if(a == '>')
      accepted = true;
      if (cdIndex < sizeof(iscoData))
      {
        iscoData[cdIndex] = a; //here might need to ring buffer
      }
      ++cdIndex;
    if (accepted and comm == 1)
        Sample();


  }} while (millis() - timer < 2000);
  }
  #endif

  #if version2
  void request_ISCO() //CHECK FOR OVERFLOW ON THIS ARRAY
  {
  Serial.println("Sending req to ISCO");
  iscoSerial.println("??????????????????????");

  delay(2000);
  if (!iscoSerial.available())
  {
    iscoSerial.print("??????????????????????");

    delay(2000);
    //Serial.println("Trying another baud...");
  } else
  {
    //readIscoSerial(comm);
  }
  if (!iscoSerial.available())
  {
    iscoSerial.println("????");

    delay(10);
    Serial.println("3rd baud attempt...");
  } else
  {
   // readIscoSerial(comm);
  }
  delay(2000);
  if (!iscoSerial.available())
  {
    Serial.println("No data available...giving up");
  }
  else{
   // readIscoSerial(comm);
  }

  }
  #endif
*/


unsigned long getSampledTime() //get time time the last bottle was sampled
{
  int commaNumber2 = 0;
  float sampledTime;
  for (int x = 0; x < 210 ; ++x)
  {
    if (iscoData[x] == ',')
    {
      ++commaNumber2;
      //Serial.print(commaNumber);
    }
    if (commaNumber2 == 11)
    {
      int startTimeIndex = x + 1;

      for (int y = 0; y < 11; ++y)
      {
        iscoTime[y] = iscoData[startTimeIndex];
        ++startTimeIndex;
      }
      Serial.println(iscoTime);
      double sampledTime = atof(iscoTime);
      time_t unixTime;
      Serial.print("Sampled time "); Serial.println(sampledTime, 6);
      unixTime = (sampledTime - 25569) * 86400;
      Serial.print("unix time "); Serial.println(unixTime);
      return unixTime;

      //Serial.print("Next two characters are");Serial.print(iscoData[x+1]);Serial.println(iscoData[x+2]);
      //Serial.print("Bottle Number is ");Serial.println(bottleNumber);

    }

  }
  return 0;
}

float getBV()
{
  if (ina260Available)
  {
    float c = 0.0;
    c = ina260.readBusVoltage();
    return c;
  } else
  {
    return -99.99;
  }

}

void toggleRQ30()
{
  if (RQ30)
  {
    RQ30 = false;
    digitalWrite(RQ30Relay, LOW);
  } else
  {
    RQ30 = true;
    digitalWrite(RQ30Relay, HIGH);
  }
}


bool getWaterLevel()
{
  int reading = 0;
  int over = 0;
  int under = 0;
  for (int x = 0 ; x < 50 ; ++x)
  {
    reading = analogRead(levelPin);
    //Serial.print("Analog reading is : ");Serial.println(reading);
    if (reading > 100)
    {
      ++over;
    } else
    {
      ++under;
    }
  }
  if (over + 20 > under) //bias towards failing dry
  {
    return false;
  } else
  {
    return true;
  }

}

void postData() //post Data to VIPER
{
  int intWL;
  if (getWaterLevel()) //Transpose the water level to 50 (wet) or 0 (dry) for viewing on VIPER
  {
    intWL = 1;
    levelReading = 1;
  } else
  {
    intWL = 0;
    levelReading = 0;
  }

  // format addX
  http.addInt("BottleNumber", bottle_number, "/24");
  //http.addInt("MoistureReading", droplet_read(), "rM");
  http.addFloat("BatteryVoltage", getBV(), "mV");
  http.addInt("LevelReading", intWL, "0/50");
  http.addFloat("RainIntensity", intensity, "mm/hr");
  http.addInt(".01in daily tips", dailyTips, "");
  http.addFloat("HProbeTTemp", temp, "degC");
  http.addFloat("HProbeTMoisture", moisture * 100 , "%");
  http.addFloat("HProbeTConductivity", conductivity, "S/m");
  http.addFloat("HProbeTPermittivity", permittivity, "Dielectric Units");
  http.addFloat("HProbeMTemp", temp2, "degC");
  http.addFloat("HProbeMMoisture", moisture2 * 100, "%");
  http.addFloat("HProbeMConductivity", conductivity2, "S/m");
  http.addFloat("HProbeMPermittivity", permittivity2, "Dielectric Units");
  http.addFloat("HProbeBTemp", temp3, "degC");
  http.addFloat("HProbeBMoisture", moisture3 * 100, "%");
  http.addFloat("HProbeBConductivity", conductivity3, "S/m");
  http.addFloat("HProbeBPermittivity", permittivity3, "Dielectric Units");

#if FLOWMETER
  http.addFloat("flowMeter", flowrate, "Lpm");
  http.addInt("totalMilliLitres", totalMilliLitres, "mLs");
#endif
  Serial.print("data: " ); Serial.print(getBottleNumber()); Serial.print(",");
  Serial.print(getBV()); Serial.print(","); Serial.print(intWL); Serial.print(",");
  Serial.print(intensity); Serial.print(","); Serial.print(temp); Serial.print(",");
  Serial.print(moisture * 100); Serial.print(","); Serial.print(conductivity); Serial.print(",");
  Serial.print(permittivity); Serial.print(","); Serial.print(temp2); Serial.print(",");
  Serial.print(moisture2 * 100); Serial.print(","); Serial.print(conductivity2); Serial.print(",");
  Serial.print(permittivity2); Serial.print(","); Serial.print(temp3); Serial.print(",");
  Serial.print(moisture3 * 100); Serial.print(","); Serial.print(conductivity3); Serial.print(","); Serial.println(permittivity3);
#if FLOWMETER
  Serial.print(flowrate); Serial.print(","); Serial.println(totalMilliLitres);
#endif
  ident++;
  String identS = String(ident);
  delay(10000);

  // 2P,Data;datd;etc. is built in the HTTP Libary in the Custom Library Folder
  //xbeeSerial.flush();
  String data = String(http.getData());
  data = identS + "<" + data + "!";
  Serial.print("Data to format is: ");
  Serial.println(data);
  serialFormat(data);
  //xbeeSerial.print(identS + "," + http.getData() + "!");  //place end character for data integrity.
  Serial.println(http.getData());
  delay(1000);
  //xbeeSerial.flush();
  // xbeeSerial.println("P"http.getData());  //backup solution
  Serial.println("Done sending to VIPER");
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


void digitalClockDisplay() {
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.println();
}

void setfileName()
{
  int y = sprintf(fileName, "%s%s%i%s", monthChar, dateChar, year(), ".txt");
  y = sprintf(fileNameLog, "%s%s%i%s", monthChar, dateChar, year(), ".log");
  Serial.print("Filename is: "); Serial.println(fileName);
  Serial.print("Log Filename is: "); Serial.println(fileNameLog);
}

void writeBottleInfo() {


  myFile = SD.open(fileName, FILE_WRITE); //SD CARD CLASS LOOK UP FOR FEATURES
  // open the file for write at end like the Native SD library
  if (!myFile) {
    Serial.println("opening sd file for write failed");
  } else
  {


    delay(10);
    buildDateTime();
    myFile.println(bottleInfo);

    // close the file:
    myFile.close();
    Serial.println("done.");
  }


}

void writeMsgTxt() {


  logFile = SD.open(fileNameLog, FILE_WRITE); //SD CARD CLASS LOOK UP FOR FEATURES
  // open the file for write at end like the Native SD library
  if (!logFile) {
    Serial.println("opening sd file for write failed");
  } else
  {
    Serial.println("File opened");
    // if the file opened okay, write to it:
    Serial.print("Writing to "); Serial.println(fileNameLog);

    delay(10);
    //buildDateTime();
    logFile.print("log--");
    logFile.println(msgTxt);

    // close the file:
    logFile.close();
    Serial.println("Printing to log -- done.");
  }


}


void saveData() {

  if (writeNum == 0)
  {
    buildDateTime();
    setfileName();
  }
  Serial.println("Saving Data--_###---###---###---###---####---###");
  
  char dailyTipsChar[10];
  char intensityChar[10];
  char tipsChar[10];
  String batt = getBV();
  dtostrf(BattVoltage, 3, 2, bVChar);
  sprintf(bottleNumChar, "%i", bottle_number);
  sprintf(dailyTipsChar, "%i", dailyTips);
#if ISCORain
  if (dailyTips > lastDaily && !firstIntensity)
  {
  tips = dailyTips - lastDaily;
  firstIntensity = false;
  }else
  {
  tips = 0;
  }
  lastDaily = dailyTips;
#endif
  Serial.print("tips = "); Serial.println(tips);
  intensity = tips*.254*60;
  intensityArray[intensityIndex] = intensity;

  sprintf(tipsChar, "%i", tips);
  sprintf(intensityChar, "%f", intensity);

  sprintf(levelChar, "%i", levelReading);

  myFile = SD.open(fileName, FILE_WRITE); //SD CARD CLASS LOOK UP FOR FEATURES
  // open the file for write at end like the Native SD library
  if (!myFile) {
    Serial.println("opening sd file for write failed");
  } else
  {

    Serial.println("File opened");

    // if the file opened okay, write to it:
    Serial.print("Writing to "); Serial.println(fileName);
    Serial.print("writeNum = "); Serial.println(writeNum);
    if (writeNum == 0)
    {

      writeNum++;
      Serial.println("Writing header");   //Write headers
      Serial.println("");

      myFile.print("## WET BOARD Unit ");
      myFile.print(unit);
      myFile.println(" ## ");
      myFile.println("");
      //int localHours = hour(local);
      //int utcHours = hour(utc);

      myFile.print("TIMESTAMP(EST),BottleNumber (/24),level Reading (0/1),Battery Voltage (mV),Probe1_temp, Probe1_moisture, Probe1_conductivity, Probe1_permitivity,");
      myFile.println("Probe2_temp, Probe2_moisture, Probe2_conductivity, Probe2_permitivity,Probe3_temp, Probe3_moisture, Probe3_conductivity, Probe3_permitivity,daily_tips,current_tips,rain intensity(mm/hr)");
    }
    //check all data to make sure all VIPER data is being put here also.
    delay(10);
    buildDateTime();
    myFile.print(dateTime);
    myFile.print(",");
    myFile.print(bottleNumChar);
    myFile.print(",");
    myFile.print(levelChar);
    myFile.print(",");
    myFile.print(batt);
    myFile.print(",");
    //myFile.print(rain_level);
    //myFile.print(",");
    myFile.print(temp);
    myFile.print(",");
    myFile.print(moisture);
    myFile.print(",");
    myFile.print(conductivity);
    myFile.print(",");
    myFile.print(permittivity);
    myFile.print(",");
    myFile.print(temp2);
    myFile.print(",");
    myFile.print(moisture2);
    myFile.print(",");
    myFile.print(conductivity2);
    myFile.print(",");
    myFile.print(permittivity2);
    myFile.print(",");
    myFile.print(temp3);
    myFile.print(",");
    myFile.print(moisture3);
    myFile.print(",");
    myFile.print(conductivity3);
    myFile.print(",");
    myFile.print(permittivity3);
    myFile.print(",");
    myFile.print(dailyTipsChar);
    myFile.print(",");
    myFile.print(tipsChar);
    myFile.print(",");
    myFile.print(intensityChar);
    myFile.println(" ");

    // close the file:
    myFile.close();
    Serial.println("done.");
  }


}


void buildDateTime()
{

  if (month() < 10)
  {
    int y = sprintf(monthChar, "%c%i", '0', month());
  } else
  {
    int y = sprintf(monthChar, "%i", month());
  }

  if (day() < 10)
  {
    int y = sprintf(dateChar, "%c%i", '0', day());
  } else
  {
    int y = sprintf(dateChar, "%i", day());
  }

  if (hour() < 10)
  {
    int y = sprintf(hourChar, "%c%i", '0', hour());
  } else
  {
    int y = sprintf(hourChar, "%i", hour());
  }

  if (minute() < 10)
  {
    int y = sprintf(minuteChar, "%c%i", '0', minute());
  } else
  {
    int y = sprintf(minuteChar, "%i", minute());
  }

  if (second() < 10)
  {
    int y = sprintf(secondChar, "%c%i", '0', second());
  } else
  {
    int y = sprintf(secondChar, "%i", second());
  }


  //int y = sprintf(dateTimeVIPER, "%i%s%s%s%s%s%s%s%s%s%s%s%i%s", year(localNow), "-", localMonthChar, "-", localDateChar, "T", localHourChar, ":", minuteChar, ":", secondChar, "-0",tzOffset,":00");
  //Serial.println("In build");
  //Serial.print("DTV -- "); Serial.println(dateTimeVIPER);
  sprintf(dateTime, "%i%s%s%s%s%s%s%s%s%s%s", year(), "-", monthChar, "-", dateChar, "T", hourChar, ":", minuteChar, ":", secondChar);
  Serial.print("Time -- && -- &&   : "); Serial.println(dateTime);
}

time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/*  code to process time sync messages from the serial port   */
#define TIME_HEADER  "T"   // Header tag for serial time sync message

unsigned long processSyncMessage() {
  unsigned long pctime = 0L;
  const unsigned long DEFAULT_TIME = 1357041600; // Jan 1 2013

  if (Serial.find(TIME_HEADER)) {
    pctime = Serial.parseInt();
    return pctime;
    if ( pctime < DEFAULT_TIME) { // check the value is a valid time (greater than Jan 1 2013)
      pctime = 0L; // return 0 to indicate that the time is not valid
    }
  }
  return pctime;
}



float getWaterIntensity() //parse the Water Intensity from ISCO input
{
  char waterString [10];
  int commaNumber1 = 0;
  float water_intensity = 0;
  sprintf(waterString, "%s", "");
  for (int x = 0; x < 210 ; ++x)
  {
    if (iscoData[x] == ',')
    {
      ++commaNumber1;
      //Serial.print(commaNumber);
    }
    if (commaNumber1 == 12)//We do not know this yet
    {
      float y = sprintf(waterString, "%c%c%c%c", iscoData[x + 2], iscoData[x + 3], iscoData[x + 4], iscoData[x + 5]);
      water_intensity = atoi(bottleNumChar);
      Serial.print("Your next line will be "); Serial.print(iscoData[x + 1]);
      Serial.print(iscoData[x + 2]); Serial.print(iscoData[x + 3]); Serial.println(iscoData[x + 4]);
      Serial.print("Water intensity is ");
      Serial.println(water_intensity);
      //return bottleNumber;
      return water_intensity;
    }

  }
  return 0;
}


/*
   THE FUNCTIONS BEYOND THIS POINT ARE UPDATES TO THE ISCO'S FUNCTIONALITY
*/

/*
   Daylight ST checks the clock of the Teensy and moves the hour to Daylight Savings if valid here
   No params

*/
void DaylightST() { //check to see if DST is active
  time_t eastern, utc;
  TimeChangeRule *tcr;
  TimeChangeRule usEDT = {"EDT", Second, Sun, Mar, 2, -240};  //UTC - 4 hours
  TimeChangeRule usEST = {"EST", First, Sun, Nov, 2, -300};   //UTC - 5 hours
  Timezone usEastern(usEDT, usEST);
  utc = now();  //current time from the Time Library
  eastern = usEastern.toLocal(utc, &tcr);
  Serial.print("The time zone is: ");
  Serial.println(tcr -> abbrev);
}
/*
   compileTime helps the clock account for the time lag during compile/startup time
   no params
*/
time_t compileTime()
{
  const time_t FUDGE(10);     // fudge factor to allow for compile time (seconds, YMMV)
  const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char chMon[3], *m;
  tmElements_t tm;

  strncpy(chMon, compDate, 3);
  chMon[3] = '\0';
  m = strstr(months, chMon);
  tm.Month = ((m - months) / 3 + 1);
  tm.Day = atoi(compDate + 4);
  tm.Year = atoi(compDate + 7) - 1970;
  tm.Hour = atoi(compTime);
  tm.Minute = atoi(compTime + 3);
  tm.Second = atoi(compTime + 6);
  time_t t = makeTime(tm);
  return t + FUDGE;           // add fudge factor to allow for compile time
}

int droplet_read() {
  if (analogRead(A14) < 300) rain_level = 3;
  else if (analogRead(A14) < 500) rain_level = 2;
  else rain_level = 1;
  Serial.print("Analog Reading is: ");
  Serial.println(analogRead(A14));
  return rain_level;
}
#if DAVIS
char davisIn[50];
char intensityIn[10];
char tipsIn[10];
void I2C_rain() {

  Serial3.println("R");
  Serial.println("Sending data request");

  unsigned long timeout = 0;
  do {
    ++timeout;
    delay(1);
  } while (!Serial3.available() && timeout < 2000);
  delay(400);

  if (Serial3.available())
  {
    Serial.println("Reading in Davis");
    int index = 0;
    do {
      davisIn[index] = Serial3.read();
      Serial.print(davisIn[index]);
      ++index;

    } while (Serial3.available());
    davisIn[index] = '\0';

  } else
  {
    Serial.println("Serial not available");
  }

  char *intensityChar = subStr(davisIn, ",", 1);
  char *tipsChar = subStr(davisIn, ",", 2);
  char *dailyTipsChar = subStr(davisIn, ",", 3);

  Serial.println(intensityChar);
  Serial.println(tipsChar);
  Serial.println(dailyTipsChar);

  intensityArray[intensityIndex] = atof(intensityChar);
  ++intensityIndex;
  
  tips = atoi(tipsChar);
  dailyTips = atoi(dailyTipsChar);
  


}


#endif

char* subStr(char* input_string, char *separator, int segment_number)
{
  char *act, *sub, *ptr;
  static char copyStr[100];
  int i;

  strcpy(copyStr, input_string);
  for (i = 1, act = copyStr; i <= segment_number; i++, act = NULL)
  {
    sub = strtok_r(act, separator, &ptr);
    if (sub == NULL) break;
  }
  return sub;
}



void serialFormat(String post) {

  int len = post.length();
  double char_speed = 1 / 115200;
  float total_time = char_speed * len;
  Serial.print("Total send time is: ");
  Serial.println(total_time);
  int d = len / 3;

  String one = post.substring(0, d);
  String two = post.substring(d, d * 2);
  String three = post.substring(d * 2);

  char a;

  xbeeSerial.print('$'); //send handshake character

  int serialTimeout = 0; //wait for handshake character
  do {
    delay(1);
    ++serialTimeout;
  } while (!xbeeSerial.available() && serialTimeout < 3000);

  if (serialTimeout > 2998)
  {
    Serial.println("No handshake received, timeout");
  }

  serialTimeout = 0;

  a = xbeeSerial.read();

  if (a) //if handshake received, sent first portion of data
  {
    delay(50);
    Serial.println("Received handshake-1");
    xbeeSerial.print(one);
  } else
  {
    Serial.print("Received(1) - ");
    Serial.println(a);
  }

  xbeeSerial.flush();
  delay(10);

  do {
    delay(1);
    ++serialTimeout;
  } while (!xbeeSerial.available() && serialTimeout < 3000);


  if (serialTimeout > 2998)
  {
    Serial.println("No handshake received, timeout");
  }

  serialTimeout = 0;
  a = xbeeSerial.read();
  if (a) //if handshake received, sent second portion of data
  {
    delay(50);
    Serial.println("Received handshake-2");
    xbeeSerial.print(two);
  }
  else
  {
    Serial.print("Received(2) - ");
    Serial.println(a);
  }

  xbeeSerial.flush();
  delay(10);

  do {
    delay(1);
    ++serialTimeout;
  } while (!xbeeSerial.available() && serialTimeout < 3000);


  if (serialTimeout > 2998)
  {
    Serial.println("No handshake received, timeout");
  }
  a = xbeeSerial.read();
  serialTimeout = 0;

  if (a) //if handshake received, sent last portion of data (including !)
  {
    delay(50);
    Serial.println("Received handshake-3");
    xbeeSerial.print(three);
  }
  else
  {
    Serial.print("Received(3) - ");
    Serial.println(a);
  }
  // xbeeSerial.print(post);
  //delay(500);
  Serial.println("Sent data across BUS to Xbee:");
  Serial.println(one);
  Serial.println(two);
  Serial.println(three);
}


void connectToISCO()
{
  int iscoTimeout = 0;
  do {
    iscoSerial.write('?');
    //Serial4.write(13);
    delay(100);
    //Serial.print(".");
    ++iscoTimeout;
  } while (iscoSerial.available() == 0 && iscoTimeout < 300);

  if (iscoSerial.available())
  {
    Serial.println(iscoSerial.readString());
    Serial.println("Connected to ISCO");
    iscoConnected = true;
  } else
  {
    Serial.println("Could not connect to ISCO");
    iscoConnected = false;
  }


}

void checkHydraProbes()
//Checks all Hydraprobes and grabs 4 data points (temperature, Moisture,conductivity and permittivity
{
  if (moistureSensor.getHPStatus())
  {
    Serial.println("---------------Sensor 1-----------------");
    moistureSensor.parseResponse();
    temp = moistureSensor.getTemp();
    moisture = moistureSensor.getMoisture();
    conductivity = moistureSensor.getConductivity();
    permittivity = moistureSensor.getPermittivity();

    Serial.print("Temp = ");
    Serial.println(temp);
    Serial.print("Moisture = ");
    Serial.println(moisture);
    Serial.print("Conductivity = ");
    Serial.println(conductivity);
    Serial.print("Permittivity = ");
    Serial.println(permittivity);
    Serial.println("");
  }
  else
  {
    Serial.println("Could not communicate with HydraProbe (1).");
    Serial.println("Check Connection.");
  }


  if (moistureSensor2.getHPStatus())
  {
    Serial.println("---------------Sensor 2-----------------");
    moistureSensor2.parseResponse();


    temp2 = moistureSensor2.getTemp();
    moisture2 = moistureSensor2.getMoisture();
    conductivity2 = moistureSensor2.getConductivity();
    permittivity2 = moistureSensor2.getPermittivity();

    Serial.print("Temp 2 = ");
    Serial.println(temp2);
    Serial.print("Moisture 2 = ");
    Serial.println(moisture2);
    Serial.print("Conductivity 2 = ");
    Serial.println(conductivity2);
    Serial.print("Permittivity 2 = ");
    Serial.println(permittivity2);
    Serial.println("");
  }
  else
  {
    Serial.println("Could not communicate with HydraProbe (2).");
    Serial.println("Check Connection.");
  }



  if (moistureSensor3.getHPStatus())
  {

    Serial.println("---------------Sensor 3-----------------");
    moistureSensor3.parseResponse();


    temp3 = moistureSensor3.getTemp();
    moisture3 = moistureSensor3.getMoisture();
    conductivity3 = moistureSensor3.getConductivity();
    permittivity3 = moistureSensor3.getPermittivity();
    Serial.print("Temp 3 = ");
    Serial.println(temp3);
    Serial.print("Moisture 3 = ");
    Serial.println(moisture3);
    Serial.print("Conductivity 3 = ");
    Serial.println(conductivity3);
    Serial.print("Permittivity 3 = ");
    Serial.println(permittivity3);
    Serial.println("");
  }
  else
  {
    Serial.println("Could not communicate with HydraProbe (3).");
    Serial.println("Check Connection.");
  }
}

int getCurrentRain()
{
  //Serial.println(inputData);
  int str_len = inputData.length() + 1;
  int index = 0;
  int loops = 0;
  char data[str_len];
  inputData.toCharArray(data, str_len);
  do {
    ++loops;
    index = getIndex(data, 'R', 2, str_len);
    //Serial.print("R index is ");
    if (index == 0)
    {
      //Serial.println ("not found");
      return -1;

    } else
    {

      //Serial.println(index);
      if (data[index + 1] == 'A')
      {
        //Serial.println("RA found");
        char number[3] = {data[index + 3], data[index + 4], data[index + 5]};
        int crNum = atoi(number);

        if (crNum > -1 && crNum < 256)
        {
          //Serial.println("Valid number of tips");
          return crNum;
        } else
        {
          return -1;
        }

      } else
      {
        return -1;
      }
    }
  } while (index < str_len && loops < 3);
  return -1;

}

int getDaysRain()
{
  //Serial.println(inputData);
  int str_len = inputData.length() + 1;
  char data[str_len];
  int index = 0;
  int loops = 0;
  inputData.toCharArray(data, str_len);
  do {
    ++loops;
    index = getIndex(data, 'C', 1, str_len);
    //Serial.print("C index is ");
    if (index == 0)
    {
      //Serial.println ("not found");
      return -1;

    } else
    {

      //Serial.println(index);
      if (data[index + 1] == 'R')
      {
        //Serial.println("CR found");
        char number[3] = {data[index + 3], data[index + 4], data[index + 5]};
        int drNum = atoi(number);

        if (drNum > -1 && drNum < 256)
        {
          //Serial.println("Valid number of tips");
          return drNum;
        } else
        {
          return -1;
        }

      } else
      {
        return -1;
      }
    }
  } while (index < str_len && loops < 3);
  return -1;
}


int getBottleNumber()
{
  inputData = Serial4.readString();
  Serial.println(inputData);
  int str_len = inputData.length() + 1;
  char data[str_len];
  inputData.toCharArray(data, str_len);
  int index = getIndex(data, 'B', 1, str_len);
  Serial.print("index is ");
  if (index == 0)
  {
    Serial.println ("not found");
    return 0;

  } else
  {
    Serial.println(index);
    char number[2] = {data[index + 1], data[index + 2]};
    int botNum = atoi(number);
    Serial.print("Bottle number is "); Serial.println(botNum);

    if (botNum > 0 && botNum < 25)
    {
      Serial.println("Valid bottle Number");
      int startTimeIndex = index + 3;
      char iscoTime[15]; //holds last bottle sample time from ISCO memory

      if (botNum > 9)
      {
        startTimeIndex++;;
      }
      for (int y = 0; y < 11; ++y)
      {
        iscoTime[y] = data[startTimeIndex];
        ++startTimeIndex;
      }
      Serial.println(iscoTime);
      double sampledTime = atof(iscoTime);
      time_t unixTime;
      Serial.print("Sampled time "); Serial.println(sampledTime, 6);
      unixTime = (sampledTime - 25569) * 86400;
      Serial.print("unix time "); Serial.println(unixTime);
      unsigned long sampled = unixTime;

      if (unixTime > 1615412441) //date of upload
      {
        Serial.println("VALID");

        if (month(sampled) < 10)
        {
          int y = sprintf(monthChar, "%c%i", '0', month(sampled));
        } else
        {
          int y = sprintf(monthChar, "%i", month(sampled));
        }

        if (day(sampled) < 10)
        {
          int y = sprintf(dateChar, "%c%i", '0', day(sampled));
        } else
        {
          int y = sprintf(dateChar, "%i", day(sampled));
        }

        if (hour(sampled) < 10)
        {
          int y = sprintf(hourChar, "%c%i", '0', hour(sampled));
        } else
        {
          int y = sprintf(hourChar, "%i", hour(sampled));
        }

        if (minute(sampled) < 10)
        {
          int y = sprintf(minuteChar, "%c%i", '0', minute(sampled));
        } else
        {
          int y = sprintf(minuteChar, "%i", minute(sampled));
        }

        if (second(sampled) < 10)
        {
          int y = sprintf(secondChar, "%c%i", '0', second(sampled));
        } else
        {
          int y = sprintf(secondChar, "%i", second(sampled));
        }

        sprintf(bottleInfo, "%s%i%s%s/%s/%i %s:%s:%s", "Sampled Bottle ", botNum, " @ ", monthChar, dateChar, year(sampled), hourChar, minuteChar, secondChar);
        Serial.println(bottleInfo);
        if (strcmp(bottleInfo, lastBottleInfo))
        {
          if (SDcard) writeBottleInfo();
        }

        sprintf(lastBottleInfo, "%s", bottleInfo);
        newBottle = true;
        return botNum;

      } else {
        newBottle = false;
        return 0;
      }
    }
    return 0;
  }


}

int getIndex(char data[], char delimiter, int num, int str_len)
{
  int delimNum = 0;
  int strIndex = -1;
  do {
    ++strIndex;
    if (data[strIndex] == delimiter)
    {
      ++delimNum;
      //Serial.print("c,");Serial.println(strIndex);
    }

    //Serial.println(strIndex);
  } while (delimNum < num && strIndex < str_len);

  if (strIndex < str_len && delimNum == num)
  {
    return strIndex;
  } else {
    return 0;
  }

}
