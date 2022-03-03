
/******************************************/
/*Trigger Scope v. 200 for Arduino microscope control by 
 ADVANCED RESEARCH CONSULTING @ 2015
 for info contact Austin Blanco
 austin@austinblanco.com c. 510-708-2995
 
  V612-ts4 changes
  - addressed char readin of data from SD card
  - replaced var type for RANGE DAC assignment & TRIG line assignment for suppoprting SD access( non const)
  - Added RESET Command to serial-reboot board. 
  - 

  TODO:
  -data card readin is OK but needs to be a tre ascii/dec conversion. 
  
  
 */

/*****************************
Contact Advanced Research Consulting for Driver libraries! www.advancedresearch.comsulting
 ******************************/
#include <SD.h>
#include <SPI.h>
//#include <i2c_driver_wire.h>
#include <Wire.h>
#include "Linduino.h"

#include "Adafruit_MCP23017.h"

#define focus 15     //sets focus line #
#define pwrLed 11    //POWER indication
#define dacLed 12    //POWER indication
#define ttlLed 13    //POWER indication
#define trigLed 14   //POWER indication
#define readyLed 15  //POWER indication
#define ttlblock2OE 10
#define ttlblock2DIR 8
#define ttlblock1OE 9
#define ttlblock1DIR 7

File myFile; //create settings file
const int chipSelect = BUILTIN_SDCARD; //set SD card access CS line

Adafruit_MCP23017 mcp; //create mux object
//set up menu modes and vars
byte opMode = 1;            //sets operation mode, 1=MANUAL control, 2 = PC CONTROL, 3=TTL Slave via Programmed Steps
volatile int vOut=0;        //voltage output digital value - change this to change the voltage output on the analog BNC
boolean trigArmed=false;    //enables while loop for just the high speed trig sequencing
unsigned long debugT=0;     //debugger flag - not used in a few versions
unsigned long trigInterval; //tracks frequency of inputs from camera or external source
int trigStep=0;             //optionally used to sequence more than 1 PROG line at a time, not fully implemented
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

//PIN ASSIGNMENTS
const byte DAC[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; //MOVE THESE FOR CUSTOMERS IF NEEDED!
byte ttl[16] = {5,6,7,8,14,15,16,17,20,0,1,2,3,4,5,6}; //ttl pin #'s (NOTE PINS 10-16 = GPIO
byte trig[4] = {0,1,2,3};

/*HIGH SPEED PROGRAMMING MODE MEMORY BLOCK*/
int     wArray[32767] = {};  //Arbitrary wave definer
int     tArray[32767] = {};  //Arbitrary wave definer
int     dacArray[1200][16] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}; // DACprogram list
boolean ttlArray[500][16] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}}; // TTL program list
int     delArray[500]; //time delay array for high speed sequences
int     focArray[6]; //array for focus stacks = start, step, #loops,direction,slave,current step
int    waveArray[2][8] = {{0,0,0,0,0,0,0,0}}; //wave function generator, see PROG_WAVE for details
uint16_t ttlActive=0;
int timeCycles = 1; //used for high speed switching inside of a loop
int runCycles = 0; //holds running position vs total cycles for timelapse

int dacVal[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //these store the DAC assigned numbers!
byte RNG[16] = {3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3}; //range saves
byte TRG[4] = {4,4,4,4}; //trigger state saves 0 = LOW 1 = HIGH 2 = RISING 3 = FALLING 4 = CHANGE

volatile boolean inTrigger=false;
volatile boolean tState=false; //Martyn Addition
unsigned long timeOut = 5000; //timeout for sequence (set to 10 seconds by default)
unsigned long tLed = 0;
volatile boolean indicatorChange = 0; //a time tracker for LED status indications
boolean runonce = false; //Martyn addition
byte program=0; //this sets the active program # used in the Mode 3 high speed sequencer
byte maxProgram=0; //this holds the maximum program value entered, and is used to set the maxium sequence step.
byte stepMode = 1; //1 = waits for TTL IN, 2=runs continually
unsigned long tStart = 0; //sequence start timer
unsigned long trigLedTimer = 0;
boolean reportTime = 0;
boolean umArmIgnore = 0; //this handles micromanagers multiple ARM commands which are issued back-to-back on MDA acqtivation. Ignores after a short wait. 
boolean usepwm = false;
byte pChannel =0; //number of channels micromanager has attempted to control
byte lastPT=20;
String idname = "ARC TRIGGERSCOPE 16 R4 BOARD 4v.612F - Experimental Wave Function";

void setup() {
  mcp.begin(0x27);   //turn on MUX comms
  //Wire.setClock(1100000);
  for(int i=0;i<16;++i) {  //configure MCP pins as outputs
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i,LOW);
  }
  
  //configure TTL outputs 5-12
  mcp.digitalWrite(ttlblock2OE,LOW); //low for enable
  mcp.digitalWrite(ttlblock2DIR,HIGH); //high to enable 3.3v -> 5V output
  
  //configure TTL outputs 13-16 & TRIG 1-4
  mcp.digitalWrite(ttlblock1OE,LOW); //low for enable
  mcp.digitalWrite(ttlblock1DIR,LOW); //high to enable 3.3v -> 5V output
  delay(10);
  configureTrigger(TRG[0]); //will attach interrupt
  for(byte i=0;i<9;++i) { pinMode(ttl[i],OUTPUT); digitalWrite(ttl[i],LOW); } //SET OUTPUT PINS ON TTL AND CAMERA LINES
  for(byte i=9;i<16;++i) { 
    mcp.pinMode(ttl[i],OUTPUT); 
    delay(5); 
    mcp.digitalWrite(ttl[i],LOW); 
    delay(10);
  } //SET OUTPUT PINS ON TTL AND CAMERA LINES
  mcp.digitalWrite(pwrLed,HIGH); //indicate setup complete

  Serial.begin(115200); // start serial @ 115,200 baud
  //randomSeed(analogRead(A0));
  while (!Serial) { ; } // wait for serial port
 
  //read from SD card
  if(loadSet()) {Serial.println("SD Settings Loaded");}
  else{Serial.println("No Settings Loaded from SD, File Missing or No Card Found");}
  
  //printID(); //issue identifier so software knows we are running   
  /***Dac startup ***/
  pinMode(9,OUTPUT); //CLR pin must stay high!
  digitalWrite(9,LOW); //CLR Stays high ALWAYAS
  delay(50);
  digitalWrite(9,HIGH); //CLR Stays high ALWAYAS
  delay(50);
  
  SPI.begin();
  pinMode(10,OUTPUT); // DAC CS
  SPI.beginTransaction(SPISettings(30000000, MSBFIRST, SPI_MODE0)); //teensy can do 30000000 !! 
  
  //Drive All DACs & TTLs to 0
   for(int i=1;i<=16;++i) {
      setTTL(i,0);
      pcDac(i,0);
    }
  opMode=3; //HYBRID mode default HYBRID=3 / MANUAL=0 /  
  printID(); //issue identifier so software knows we are running 

  for(byte d=0;d < 16; ++d) {
    //span all DAC lines to 0-10V range
    setDacRange(d,RNG[d] ); 
  }
  
  delay(10);
  mcp.digitalWrite(15,HIGH); //indicate setup complete
  //byte a=0;
  //volatile int f=0;
  //Wire.setClock(400000);
  
 }

void loop()
{
  
  //************************   DEVICE CONTROL & COMMAND CODE          ***********************//
  //************************  SERIAL COMMUNICATION CODE    ******************///
  if (!Serial) {reboot();} 
  if (!trigArmed && inTrigger) {trigLEDHandler();}
        
  if (stringComplete)  //Check whatever came in on serial
  {
        mcp.digitalWrite(readyLed,LOW);
      if(inputString == "*\n"){ printID();} //return ID line if queried

      if(inputString.substring(0,3) == "DAC" && inputString.substring(0,5) != "DACPR" ){                                                        
        char dIn[2] = {inputString[3],inputString[4]}; //grab DAC Line # 
        byte dacNum=0;
        if(dIn[2] == 44) { dacNum = atoi(&dIn[1]); } //IF less than 10, assign single digit value
        else {dacNum = atoi(dIn); }  //IF greater than 10, assign both chars to integer
        boolean inRange=true; //flag to check valid entry
       
        byte sT=0;
        while (inputString[sT] != ',') { ++sT; } //CHECKS WHERE THE START OF THE NUMERIC DAC VALUE BEGINS
        ++sT;
        char fstring[8] = {}; //create a string to hold char ascii for later conversion to integer value       
        byte bVal = sT;
        while (sT <= inputString.length() ) 
          {
            fstring[sT -bVal] = inputString[sT];
            ++sT;
          }
        int userdac = atol(fstring); //convert char table to useable integer for DAC level
        if(dacNum < 1 || dacNum > 16) {inRange=false;} //if entry is outside valid DAC's then throw flag as false
        
        if (inRange){ //if inputs are valid{  
          Serial.print("!DAC"); //print recieve message to operator
          Serial.print(dacNum); //print recieve message to operator
          Serial.print("=");
          Serial.println(userdac);
          pcDac(dacNum,userdac);
          //led indication
          byte dTotal=0;
          for(byte d=0;d<16;++d) {if(dacVal[d] > 0){ ++dTotal;} }
          if(dTotal > 0) { mcp.digitalWrite(dacLed,1);} //turn on DAC LED
          else  {mcp.digitalWrite(dacLed,0);} //turn off LED 
        }
        if(!inRange) //if inputs are bad
          {
          Serial.println("Outside DAC Channel Range, valid ranges are 1-16....");
          }  
      }

        if(inputString.substring(0,5) == "FOCUS"){                                                        
        char fstring[15] = {}; //create a string to hold char ascii for later conversion to integer value
        for(byte sl=0;sl<inputString.length()-6;++sl)
          { //figure out length of string and calc # characters which (should be) numbers for DAC
          fstring[sl] = inputString[sl+6]; //assign 7th and farther numbers to char table
          //Serial.print(fstring[sl]);
          }
        int focusIn = atol(fstring); //convert char table to useable integer for DAC level
        Serial.print("!FOCUS,"); //print recieve message to operator
        Serial.println(focusIn);
        pcDac(DAC[focus]+1,focusIn);
      }         
      
      if(inputString.substring(0,3) == "TTL")
        {                                                        
        char dIn[2] = {inputString[3],inputString[4]}; //grab TTL Line # 
        byte ttlNum=0;
        if(dIn[2] == 44) { ttlNum = atoi(&dIn[1]); } //IF less than 10, assign single digit value
        else {ttlNum = atoi(dIn); }  //IF greater than 10, assign both chars to integer
        
        byte ttlStat=0;
        if(ttlNum < 10) { ttlStat = atoi(&inputString[5]);} //snag ttl value if < 10
        else { ttlStat = atoi(&inputString[6]);} //snag ttl Value if > 10
        //byte ttlNum = atoi(&inputString[3]); //snag ttl line #
        if(ttlStat > 1) {ttlStat=1;} //force to boolean range
        boolean goodRange=true;
        if(ttlNum < 1 || ttlNum > 16) {goodRange=false;} //confirm if input channel range is valid
        if(goodRange) //if range is OK perform command
          {
          Serial.print("!TTL"); //print recieve message to operator
          Serial.print(ttlNum);
          Serial.print(",");
          Serial.println(ttlStat);
          setTTL(ttlNum,ttlStat);
          mcp.digitalWrite(ttlLed, ttlActive > 0);
          
          clearSerial();
          }
        if(!goodRange) //if range is outside return failure msg to user
          {
          Serial.println("TTL channel out of Range. Valid is 1-16...");
          }  
      }
        if(inputString.substring(0,3) == "CAM"){                                                        
        byte ttlNum = atoi(&inputString[3]); //snag ttl line #
        byte ttlStat = atoi(&inputString[5]); //snag ttl value
        if(ttlStat > 1) {ttlStat=1;} //force to boolean range
        boolean goodRange=true;
        if(ttlNum < 1 || ttlNum > 2) {goodRange=false;} //confirm if input channel range is valid
        if(goodRange) //if range is OK perform command
          {
          Serial.print("!CAM"); //print recieve message to operator
          Serial.print(ttlNum);
          Serial.print(",");
          Serial.println(ttlStat);
          setTTL(ttlNum+9,ttlStat);
          clearSerial();
          }
        if(!goodRange) //if range is outside return failure msg to user
          {
          Serial.println("Camera channel out of Range. Valid is 1-2...");
          }
        }  
        if(inputString.substring(0,8) == "TRIGMODE"){                                                        
          byte tRNum = atoi(&inputString[9]); //snag ttl line #
          Serial.println(tRNum);
          noInterrupts();
          configureTrigger(tRNum);
          Serial.print("!TRIGMODE,"); //print recieve message to operator
          Serial.println(tRNum);
          TRG[0] = tRNum; //assign change to trigger array
          clearSerial();
          interrupts();
        } 
        if(inputString.substring(0,10) == "TIMECYCLES"){                                                        
          byte tNum = atoi(&inputString[10]); //snag ttl line #
          char fstring[15] = {}; //create a string to hold char ascii for later conversion to integer value
          byte sublength = 11;
          for(byte sl=0 ;sl<inputString.length()-sublength;++sl) { //figure out length of string and calc # characters which (should be) numbers for DAC
            fstring[sl] = inputString[sl+sublength]; //assign 7th and farther numbers to char table
           // Serial.print(fstring[sl]);
            }
          timeCycles = atol(fstring); //convert char table to useable integer for DAC level
          Serial.print("!TTIMECYCLES,"); //print recieve message to operator
          Serial.println(timeCycles);
          clearSerial();
        } 
        
       if(inputString.substring(0,5) == "RANGE"){                                                        
        //byte dline = atoi(&inputString[5]); //snag dac line #
        byte dline = inputString.substring(5).toInt();
        byte pp = 7;
        if(dline >9) {pp=8;}
        byte rangeval = inputString.substring(pp).toInt();
        boolean goodRange=true;
        if(rangeval < 1 || rangeval > 5) {goodRange=false;} //force to max range
        if(dline < 1 || dline > 16) {goodRange=false;} //confirm if input channel range is valid
        if(goodRange) //if range is OK perform command
          {
          Serial.print("!RANGE"); //print recieve message to operator
          Serial.print(dline);
          Serial.print(",");
          Serial.println(rangeval);
          setDacRange(dline-1,rangeval-1);
          clearSerial();
          }
        if(!goodRange) //if range is outside return failure msg to user
          {
          Serial.println("COmmand out of range, DAC=1-16, Range = 1-5...");
          }  
      }
      
      //status commands
      if(inputString == "STAT?\n")                     {debug();                                        }
      if(inputString == "SAVESETTINGS\n")              {if(saveSet()){Serial.println("!SAVESETTINGS"); }}
      if(inputString == "TEST?\n")                     {diagTest();                                     }
      if(inputString == "CLEAR_ALL\n")                 {clearTable();                                   }
      if(inputString == "RESET\n")                     {reboot();                                       }
      if(inputString == "STARTWAVE\n")                 {waveFunction();                                 }
      if(inputString == "CLEAR_FOCUS\n")               {clearFocus();                                   }
      if(inputString == "CLEAR_DELAY\n")               {clearDelay();                                   }
      if(inputString == "SPEED_TEST\n")                {speedtest();                                    }
      if(inputString.substring(0,9) == "CLEAR_DAC")    {clearDac();                                     }
      if(inputString.substring(0,9) == "CLEAR_TTL")    {clearTtl();                                     }
      
      
      if(inputString == "ARM\n")                       {
        Serial.println("!ARM"); 
        //if(focArray[1] != 0 && focArray[4] != 1) {++maxProgram;}
        if(timeCycles == 0){timeCycles=1;}
        runCycles = 0; // sets total loops to # ccyles * program lines
        trigArmed = true;
        }        
        
 /****Code Below controls how to program sequnces****/
        if(inputString.substring(0,10) == "PROG_FOCUS")
          {
          int start=0; //holds the starting position of the stack 
          int stepV=0; //step size
          int loopN=0; //loop number
          int dir=1; //1=up, 0 = down(negative)
          int slave=0; // 0 = no, all points run then returned to wave. 1 = one point run then returned to wave
          int charPos=11; //default position and index of the character counter for the string input
          byte stepper=0; //used for looking at the text and finding the values
          String nums = ""; //character array to hold numbers

          for(byte i=0;i<5;++i) //loop this function to pull 5 integers from our string, 
            {
            while(inputString[charPos] != ',' || inputString[charPos] != '\n' ) 
              {
              if(isDigit(inputString[charPos])) //make sure it's a real #
                {
                 nums += (char)inputString[charPos];
                 ++charPos;
                 } //search for a comma       
                else break; 
                if(charPos >100) {break;} //in case the user forgot a comma
              }
            if(i==0) {start = nums.toInt() ;} //send to program #                      
            if(i==1) {stepV = nums.toInt();} //send to DAC #    
            if(i==2) {loopN = nums.toInt() ;} //convert the string to an integer value 
            if(i==3) {dir   = nums.toInt() ;} //convert the string to an integer value 
            if(i==4) {slave = nums.toInt() ;} //convert the string to an integer value 
             
            ++charPos; //bump up one to move to next char past comma
            nums = ""; //clear string buffer                     
            }
            //we have collected values, and now will error check and assign
            boolean valid = true;
           if(dir  < 0 || dir  > 1) {valid=false;} //check dac level

           if(valid) 
            {
              focArray[0] = start;
              focArray[1] = stepV;
              focArray[2] = loopN;
              focArray[3] = dir;
              focArray[4] = slave;
            }
            if(!valid){Serial.println("one or more values out of range...");}
           
           //Report collected values
           if(valid)
            {
            char out[200];
            sprintf(out,"!PROG_FOCUS,%u,%u,%u,%u,%u\n",start,stepV,loopN,dir,slave);
            Serial.print(out);
            ++pChannel;
            }
           }
          //example string to this entry = "PROG_WAVE,1,1,10,0,100,10"
          if(inputString.substring(0,9) == "PROG_WAVE")
          {
          byte dline=0; //Dac line to assign for this waveform, only 2x forms may be generated!
          byte tline=0;
          byte wstep=0;  //number of steps to run befor restarting  
          int wtrig=0;    // wave trigger value = 0= free run, 1= use prog del array # 1, 2 = change of TRIG 1, 3 = rising TRIG1 4 = falling fo trig 1
          byte charPos=10;//default position and index of the character counter for the string input
          byte stepper=0; //used for looking at the text and finding the values
          String nums=""; //character array to hold numbers
          byte freq=0; //delay time for each loop
          byte cycle=0;
          byte arrayPos=0;
          for(byte i=0;i<6;++i){ //loop this function to pull 5 integers from our string, 
            while(inputString[charPos] != ',' || inputString[charPos] != '\n' ) 
              {
              if(isDigit(inputString[charPos])) //make sure it's a real #
                {
                 nums += (char)inputString[charPos];
                 ++charPos;
                 } //search for a comma       
                else break; 
                if(charPos >200) {break;} //in case the user forgot a comma
              }
            //if(i==0) {dline = nums.toInt() ;} //DAC line #                      
            //if(i==1) {wstep   = nums.toInt() ;} //step value
            //if(i==2) {wtrig   = nums.toInt() ;} //convert the string to an integer value 
            if(i==0) {dline = nums.toInt() ;} //DAC line #          
            if(i==1) {tline = nums.toInt() ;} //TTL line #            
            if(i==2) {wstep   = nums.toInt() ;} //step value
            if(i==3) {wtrig   = nums.toInt() ;} //convert the string to an integer value 
            if(i==4) {freq   = nums.toInt() ;} //convert the string to an integer value
            if(i==5) {cycle   = nums.toInt() ;}            
            ++charPos; //bump up one to move to next char past comma
            nums = ""; //clear string buffer                     
            }

            //we have collected values, and now will error check and assign
           boolean valid = true;
           if(dline  < 1 || dline  > 16) {valid=false;} //dac line test
           //if(dline  < 1 || dline  > 16) {valid=false;} //check table assignment
           if(tline  < 1 || tline  > 16) {valid=false;} //ttl line test
           //if(tline  < 1 || tline  > 16) {valid=false;} //check table assignment           
           if(valid) 
            {
              byte wArr = 0;
              waveArray[wArr][0] = dline-1;
              waveArray[wArr][1] = freq;
              waveArray[wArr][2] = tline;
              waveArray[wArr][3] = cycle;
              waveArray[wArr][4] = wstep;
              waveArray[wArr][5] = wtrig;
            }
            if(!valid){Serial.println("one or more values out of range...");}
           //Report collected values
            else
              {
              char out[200];
              sprintf(out,"!PROG_WAVE,%u,%u,%u,%u,%u,%u\n",dline,tline,wstep,wtrig,freq,cycle);
              Serial.print(out);
              ++pChannel;
              }
           //NEED WAVEFORM COLLECTOR HERE!
           for(int i=0;i<wstep;++i){ //collect all wave values 
              String DACst = "";
              String TTLst = "";
              char inChar = 'A';
              while(inChar != ','){
                while (!Serial.available()){} //wait for serial data
                  inChar = (char)Serial.read();  // get the new byte:
                  DACst += inChar; // add it to the inputString:
              }
              //place value in array here
              wArray[i] =  DACst.toInt(); //no error checking yet TODO 
              while(inChar != '\n'){
                while (!Serial.available()){} //wait for serial data
                  inChar = (char)Serial.read();  // get the new byte:
                  TTLst += inChar; // add it to the inputString:
              }
              //place value in array here
              tArray[i] =  TTLst.toInt(); //no error checking yet TODO  
           }

           for(int i=0;i<wstep;++i){ //repot all wave values 
            char out[200];
            sprintf(out,"!%u,DAC,%u,TTL,%u\n",i,wArray[i],tArray[i]);
            Serial.print(out);       
           }         
          }
        //example string to this entry = "PROG_TTL,1,13,1"
        if(inputString.substring(0,8) == "PROG_TTL")
          {
          byte progNum=0; //holds the program # 
          byte ttlNum=0;
          byte ttlVal=0;
          int charPos=9; //default position and index of the character counter for the string input
          byte stepper=0; //used for looking at the text and finding the values
          String nums = ""; //character array to hold numbers

          for(byte i=0;i<3;++i) //loop this function to pull 3 integers from our string, 
            {
            while(inputString[charPos] != ',' || inputString[charPos] != '\n' ) 
              {
              if(isDigit(inputString[charPos])) //make sure it's a real #
                {
                 nums += (char)inputString[charPos];
                 ++charPos;
                 } //search for a comma       
                else break; 
                if(charPos >40) {break;} //in case the user forgot a comma
              }
            if(i==0) {progNum = nums.toInt() - 1;} //send to program #                      
            if(i==1) {ttlNum =  nums.toInt() - 1;} //send to DAC #    
            if(i==2) {ttlVal = nums.toInt();} //convert the string to an integer value    
            ++charPos; //bump up one to move to next char past comma
            nums = ""; //clear string buffer                     
            }
            //we have collected values, and now will error check and assign
            boolean valid = true;
           if(progNum < 0 || progNum > 50) { valid=false; } //check max program
           if(ttlNum  < 0 || ttlNum  > 16) { valid=false; } //check dac line
           if(ttlVal  < 0 || ttlVal  > 1) {valid=false;} //check dac level

           if(valid) {ttlArray[progNum][ttlNum] = ttlVal;} 
           if(!valid){Serial.println("one or more values out of range...");}
           
           //array handling homework
           if(progNum > maxProgram) {maxProgram = progNum;}

           //Report collected values
           if(valid)
            {
            char out[200];
            sprintf(out,"!PROG_TTL,%i,%i,%u\n",progNum+1,ttlNum+1,ttlVal);
            if(ttlNum !=lastPT) { ++ pChannel; } //increase # channels for uarmignore only if programmed channel is different than the last time
            lastPT=ttlNum; //always update to last channel #
            
            Serial.print(out);
            }
           }

        //example string to this entry = "PROG_DAC,1,13,65000"
        if(inputString.substring(0,8) == "PROG_DAC")
          {
          byte progNum=0; //holds the program # 
          byte dacNum=0;
          int dacVal=0;
          int charPos=9; //default position and index of the character counter for the string input
          byte stepper=0; //used for looking at the text and finding the values
          String nums = ""; //character array to hold numbers

          for(byte i=0;i<3;++i) //loop this function to pull 3 integers from our string, 
            {
            while(inputString[charPos] != ',' || inputString[charPos] != '\n' ) 
              {
              if(isDigit(inputString[charPos])) //make sure it's a real #
                {
                 nums += (char)inputString[charPos];
                 ++charPos;
                 } //search for a comma       
                else break; 
                if(charPos >40) {break;} //in case the user forgot a comma
              }
            if(i==0) {progNum = nums.toInt() - 1;} //send to program #                      
            if(i==1) {dacNum =  nums.toInt() - 1;} //send to DAC #    
            if(i==2) {dacVal = nums.toInt();} //convert the string to an integer value    
            ++charPos; //bump up one to move to next char past comma
            nums = ""; //clear string buffer                     
            }
            //we have collected values, and now will error check and assign
            boolean valid = true;
           if(progNum < 0 || progNum > 50) { valid=false; } //check max program
           if(dacNum  < 0 || dacNum  > 16) { valid=false; } //check dac line
           if(dacVal  < 0 || dacVal  > 65535) {valid=false;} //check dac level

           if(valid) 
            {
            dacArray[progNum][dacNum] = dacVal;
            char out[200];
            sprintf(out,"!PROG_DAC,%i,%i,%u\n",progNum+1,dacNum+1,dacVal);
            Serial.print(out);
            }

           if(!valid){Serial.println("one or more values out of range...");}
           
           //array handling homework
           if(progNum > maxProgram) {maxProgram = progNum;}
           
          clearSerial();
        }
      clearSerial();
   //   }  
 //   }
    if(opMode == 0 || opMode ==2)
    {
      Serial.println("Message Ignored - Check Operation Mode!");
      inputString = "";     // clear the string:
      stringComplete = false;
    }
  clearSerial();
  mcp.digitalWrite(readyLed,HIGH);
  } //EXIT LOOP FOR SERIAL HERE

                              /***************************************************************This block runs the high speed control interface ****/
/****checks the acquisition order
 * mode 0 == channel first eg set ch1 stweep Z
 * mode 1 == Z first EG step Z then Ch1 Ch2 then Step Z ...
 */ 
 if(trigArmed){ //just sit here and wait for the next command until armed is off, which can only happen @ end of sequence
  boolean pd = 0; //program debug - flag on for extended output but slower performance
  int startFoc = dacVal[15]; //collect starting focus value
  tStart = millis() + timeOut; //set timeout position 
  tLed = 0;
  // focus commands = start, step, #loops,direction,slave,current step

  for(int tRuns = 0; tRuns < timeCycles;++tRuns) { //looper for time
    if(pd) {Serial.print(tRuns); Serial.print(" vs " ) ; Serial.println(timeCycles);}

    if( (focArray[4] == 0) && (focArray[2] != 0) ) { // Mode = 0 Channel 1 - Z 1, Z2, Z3
      //all_off();  
      for(program=0; program <= maxProgram ; ++program) {
        if(pd) { Serial.print("Program = "); Serial.println(program);}
        
        for(focArray[5]=0; focArray[5] < focArray[2]; focArray[5]) {// Loop Focus (array: 0=start,1=step size , 2=#loops, 3=direction, 4=slave, 5=current step
          if(pd) { Serial.print("FOC="); Serial.println(focArray[5]); }
          fastFocus();
          while ( waitTrigger(1) == 0){ if(millis() > tStart) {program = maxProgram +1; break;}  }   //WAIT FOR NEXT TRIGGER
          setLambda(); //enable channels as needed
          //delay(delArray[program]);  //wait for specified delay          
          while ( waitTrigger(0) == 0){ if(millis() > tStart) {program = maxProgram +1;break;}  }   //WAIT FOR NEXT TRIGGER
          all_off();  
        }
      }
    }
  
    if( ( focArray[4] == 1) && (focArray[2] != 0)) { // mode = 1 Z 1 - CH1, CH2, Z2, Ch1, Ch2 
      all_off();
      for(focArray[5]=0; focArray[5] < focArray[2]; focArray[5]) {// Loop Focus (array: 0=start,1=step size , 2=#loops, 3=direction, 4=slave, 5=current step
        if(pd) { Serial.print("FOC="); Serial.println(focArray[5]); }
        fastFocus();
        for(program=0; program <= maxProgram ; ++program) {
          if(pd) { Serial.print("Program = "); Serial.println(program);}
          while ( waitTrigger(1) == 0){ if(millis() > tStart) {program = maxProgram +1;break;}  }   //WAIT FOR NEXT TRIGGER
          setLambda(); //enable channels as needed
          //delay(delArray[program]);  //wait for specified delay          
          while ( waitTrigger(0) == 0){ if(millis() > tStart) {program = maxProgram +1;break;}  }   //WAIT FOR NEXT TRIGGER
          all_off();  
        }   
      }
    }

    if(focArray[2] == 0 ) { //no z is used
      for(program=0; program <= maxProgram ; ++program) {
          if(pd) { Serial.print("Program = "); Serial.println(program);}
          while ( waitTrigger(1) == 0){ if(millis() > tStart) {program = maxProgram +1;break;}  }   //WAIT FOR NEXT TRIGGER
          setLambda(); //enable channels as needed
          //delay(delArray[program]);  //wait for specified delay          
          while ( waitTrigger(0) == 0){ if(millis() > tStart) {program = maxProgram +1;break;}  }   //WAIT FOR NEXT TRIGGER
          all_off();  
        }   
    }  
  }

 pcDac(16,startFoc); //reset focus to original value 
 trigArmed=false;
 } //close trigarmed
} //close main loop

void waveFunction(){ // waveArray [wave 1/2] [0:DAC, 1:Form (sine,saw,triangle,square),2: center, 3:amplitude, 4:step per cycle, 5: duty (if used), 6: phase, 7: trigger type ), 
  clearSerial();
//while(!Serial.available()) {//continuous operation until serial event recieved
//    for(int i = 0;i < waveArray[0][4];++i){ //loop through # of steps
//      dac_write(10,0, waveArray[0][0],wArray[i]); // Set DAC Lines
for(int n = 0;n < waveArray[0][3]; n++) {//continuous operation until serial event recieved
    for(int i = 0;i < waveArray[0][4]; i++){ //loop through # of steps
      dac_write(10,0, waveArray[0][0],wArray[i]); // Set DAC Lines
      setTTL(waveArray[0][2],tArray[i]);
      //delay(1);
      delay(waveArray[0][1]);
    }
  }
}

void setLambda() {
  byte walker=0;
  for(walker = 0 ; walker < 9 ; ++walker ){  //sets DACs 1-16 
    dac_write(10,0, DAC [walker], dacArray [program] [walker]); // Set DAC Lines
    digitalWriteFast( ttl [walker] , ttlArray[program] [walker] );
    //if(walker < 9) {digitalWriteFast( ttl [walker] , ttlArray[program] [walker] );}//set TTL lines
    //else if(walker>8) {mcp.digitalWrite(ttl [walker],ttlArray[program] [walker] );}
  }
  //mcp.digitalWrite(ttl [walker], ttlArray[program] [walker] ); //set 16th TTL line - need to avoid 16 for focus line
}
  
boolean waitTrigger(boolean trState) { //this waits for a trigger, then continues if one is recieved
  boolean match = false;
  if(program > maxProgram || !trigArmed) {return 1;}  
  while(!inTrigger && trigArmed){ //wait for next pulse
    if(millis() > tStart) {
      trigArmed=false;
      program = maxProgram+1; 
      Serial.println("Timeout Exceeded IN WAITTRIGGER");
      all_off();
      break;
    } 
    //we hit the timeout so end the sequence   
    //if(tLed < millis() ) { mcp.digitalWrite(readyLed,!mcp.digitalRead(readyLed));  tLed = millis() + 30;}
  } 
  if(tState == trState) { match = true; }
  inTrigger=false; //turn off trigger
  tStart = millis() + timeOut; //update timeout delta for next run
  return match;   
}  

void fastFocus() {
 //array for focus stacks::  0= start, 1 = step size , 2 = #loops, 3 = direction, 4 = slave, 5 = current step
 if(focArray[3] == 0) {dac_write(10,0, DAC [focus], focArray[0] - (focArray[1]*focArray[5])  );} //set positive
 if(focArray[3] == 1) {dac_write(10,0, DAC [focus], focArray[0] + (focArray[1]*focArray[5])  );} //set negative
 ++focArray[5]; //  update active position
 //if(focArray[5] >= focArray[2]) { focArray[5] = 0;} //reset position to beginning  
}

void all_off() {
  for(byte walker=0;walker<9;++walker){  //sets DACs 1-16                
    //dac_write(10,0, DAC [walker], 0); // Set DAC Lines
    digitalWriteFast( ttl [walker] , 0 );
    //if(walker < 9) {digitalWriteFast( ttl [walker] , 0 );}//set TTL lines
    //else if(walker>8) {mcp.digitalWrite(ttl [walker],0 );}
  }
  mcp.writeGPIO(0x00,0); //write all off for GPIOA use hex 0x80 for LS2 direction OUTPUT 
}

byte loadSet() {
   if (!SD.begin(chipSelect)) {
    Serial.println("SD Access failure, contact ARC");
    return;
  }
  myFile = SD.open("DAC.txt");
  if (myFile) {
    byte d=0;
    byte n=0;
    while (myFile.available() && n < 17) {
      d = myFile.read();
      d = d-48;
      //byte dline = inputString.substring(5).toInt();
      //Serial.println(d);
      RNG[n] = d; //assign read value to active range array
      n++;
      }
    myFile.close();
    
  } else { return 0;}

  myFile = SD.open("TRG.txt");
  if (myFile) {
    byte d=0;
    byte n=0;
    while (myFile.available()) {
      d = myFile.read();
      d = d-48;
      //Serial.print("TRIG = ");
      //Serial.println(d);
      TRG[n] = d; //assign the trigger flag to the array
      n++;
    }
    myFile.close();
    return 1;
  } else { return 0;}
}

void trigLEDHandler(){
  if(millis() > trigInterval) { //Trigger is for display only if ! trigarmed, so cycle the LED
    mcp.digitalWrite(trigLed,!mcp.digitalRead(trigLed));
    inTrigger=false;
    trigInterval = millis() + 30; //update new delay
  }    
}

void reboot(){  //resets the board
  mcp.digitalWrite(15,LOW); 
  delay(100); 
  setup(); 
}

byte saveSet() {
   if (!SD.begin(chipSelect)) {
    Serial.println("SD Access failure, contact ARC");
    return;
  }
  
  // open the file. 
  myFile = SD.open("DAC.txt", O_WRITE|O_CREAT|O_TRUNC);
  if (myFile) { //assuming it opened properly...
    for(byte f = 0;f < 16; ++f) {myFile.print(RNG[f]); } //save in all active dac range values
    myFile.close();
  } 
  else {return 0; }
  
  myFile = SD.open("TRG.txt", O_WRITE| O_CREAT|O_TRUNC);
  if (myFile) { //assuming it opened properly...
    for(byte f = 0;f < 4; ++f) {myFile.print(TRG[f]); } 
    myFile.close();
    return 1;
  } 
  else {return 0; }
}

void clearSerial(){
    //STUFF TO CLEAN OUT THE SERIAL LINE
    inputString = "";     // clear the string:
    stringComplete = false;
  }
  
/*PC DAC CONTROL*/
void pcDac(byte dNum,int dVal){  
  vOut = dVal;
  dac_write(10,0, dNum-1,vOut); // Send dac_code
  dacVal[dNum-1] = vOut;
}

/*SET TTL CONTROL*/
void setTTL(byte t1,boolean t2){
  bitWrite(ttlActive,t1-1,t2); //assign ttlActive new output value
  if(t1 < 10) {digitalWriteFast(ttl[t1-1],t2);} //case where the write operation is using existing uController pins
  
  else if(t1 > 9) {

    //mcp.digitalWrite(ttl[t1-1],t2);
    uint8_t mask = highByte(ttlActive); //mask covers GPIOA of TTL values
    mask = mask >> 1; //shift left to provide room for last bit, should be a zero as this is the ttl13-16 output
    
    if( (t1 > 12) && t2) {
      bitWrite(mask,7,1); //if output is on last shifter && positive set register as output
      mcp.writeGPIO(mask,0);
      bitWrite(mask,7,0);
      mcp.writeGPIO(mask,0); //write all off for GPIOA use hex 0x80 for LS2 direction OUTPUT 
    }
    
    else if( (t1 > 12) && !t2) {
      bitWrite(mask,7,1); //if output is on last shifter && positive set register as output
      mcp.writeGPIO(mask,0);
      bitWrite(mask,7,0);
      mcp.writeGPIO(mask,0); //write all off for GPIOA use hex 0x80 for LS2 direction OUTPUT 
    }
    else if(t1 < 13){ // TTL 8-12 send a simple update
      //bitWrite(mask,7,0);
      bitWrite(mask,7,0);
      mcp.writeGPIO(mask,0); //write all off for GPIOA use hex 0x80 for LS2 direction OUTPUT 
    }
  
  }

}
/*SERIAL COMMO*/
void serialEvent() {
  trigArmed = false;
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}

/*INTERRUPT CODE FOR TTL INPUT ***************/
void sigIn()     //sigin is the input response line - this recieves the trigger input from an external source
  {
   //Serial.write("T");
   inTrigger=true;
   if(digitalRead(trig[0]) ) {tState=true;} //Martyn Addition
   else if(!digitalRead(trig[0])){tState=false;} //Martyn Addition 
  }
  
void configureTrigger(byte tOption) {
    TRG[0] = tOption; //assign input value to global
    switch (TRG[0]) {
    case 0:
      //attachInterrupt(digitalPinToInterrupt(trig[0]),sigIn,LOW);
      //deprecated for teensy
      break;
    case 1:
      //attachInterrupt(digitalPinToInterrupt(trig[0]),sigIn,HIGH);
      //depricated on teensy
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(trig[0]),sigIn,RISING);
      break;
    case 3:
      attachInterrupt(digitalPinToInterrupt(trig[0]),sigIn,FALLING);
      break;
    case 4:
      attachInterrupt(digitalPinToInterrupt(trig[0]),sigIn,CHANGE);
      break;
  }
}
  
void debug()
  {
  if(debugT < millis() ) //if this is the first time debug has run, this will execute, otherwise has enough time elapsed?
    {  
    Serial.print("TTL STATES:");
    //REPORT TTL
    for(int i=0;i<16;++i) 
      {
        char sOut[100];
        sprintf(sOut,"%d=%d,",i+1,digitalRead(ttl[i]));
        Serial.print(sOut);
      }
    Serial.println("");
    Serial.print("TTL Pins ( 9+ = MCU # ):");
    for(int i=0;i<16;++i) 
      {
        char sOut[100];
        sprintf(sOut,"%d=%d,",i+1,ttl[i]);
        Serial.print(sOut);
      }
    Serial.println(""); 
    //REPORT DAC
    Serial.print("DAC:");
    for(int i=0;i<16;++i) 
    {
     char sOut[200];
     sprintf(sOut,"%d=%u,",i+1,dacVal[i]);
     Serial.print(sOut); //used to print sOut
    }
    Serial.println();
    Serial.print("DAC Ranges: ");
    for(int i=0;i<16;++i) {
      Serial.print(RNG[i]) ; //range saves
      Serial.print(", ");
    }
    Serial.println();
    Serial.print("TRIG Settings: ");
    Serial.print(TRG[0]); Serial.print(", ");
    Serial.print(TRG[1]); Serial.print(", ");
    Serial.print(TRG[2]); Serial.print(", ");
    Serial.print(TRG[3]); Serial.println();
    
    
    Serial.print("Focus Position = ");
    Serial.print(dacVal[focus]); //focus is ALWAYS on line 15!
    Serial.println("");
    char sOut[255];
      //start,step,loop,direction,slave
    sprintf(sOut,"FOCUS ARRAY = Start: %u Step: %u Loop: %u Direction: %u Slave: %u Current Step: %u\n",
      focArray[0],focArray[1],focArray[2],focArray[3],focArray[4],focArray[5]);
    Serial.print(sOut);   

    Serial.print("Focus DAC Line = ");
    Serial.println(DAC[15]+1);
  
    //Report program arrays 
     Serial.println("***Sequencer Programming Status***");
     Serial.print("MaxProgram = ");
     Serial.println(maxProgram);
     //report DACs
     Serial.println("PROG, DAC1, DAC2, DAC3, DAC4, DAC5, DAC6, DAC7, DAC8, DAC9,DAC10,DAC11,DAC12,DAC13,DAC14,DAC15,DAC16/FOCUS");
     for(int p=0;p<maxProgram+1;++p) //list all recorded programs
      {      
     char sOut[200];
     sprintf(sOut,"P:%2d,",p+1);
     Serial.print(sOut);
        for(byte nVal=0;nVal<16;++nVal)
        {
          sprintf(sOut,"%05u,",dacArray[p][nVal]);
          Serial.print(sOut);
        }
        Serial.println("");   
      }
     
     //Report TTL's 
     Serial.println("PROG, TTL1, TTL2, TTL3, TTL4, TTL5, TTL6, TTL7, TTL8, TTL9,TTL10,TTL11,TTL12,TTL13,TTL14,TTL15,TTL16");
     for(int p=0;p<maxProgram+1;++p) //list all recorded programs
      {      
     char sOut[200];
     sprintf(sOut,"P:%2d,",p+1);
     Serial.print(sOut);
        for(byte nVal=0;nVal<16;++nVal)
        {
          sprintf(sOut,"    %d,",ttlArray[p][nVal]);
          Serial.print(sOut);
        }
        Serial.println("");   
      }
      
      debugT = millis() + 500; // wait 500ms before the next report 
    }
}

void diagTest()
  {
  
}

void printID()
{
  Serial.println(idname);
}

void clearTable()
  {
  for(byte i=0;i<16;++i) //all 16 channels
    {
      for(byte n=0;n<50;++n) //all program lines
      {
      dacArray[n][i] = 0; //clear dac
      ttlArray[n][i] = 0; //clear ttl
      delArray[n]    = 0; //clear delay
      }
    }
   clearFocus();  //clear focus
   maxProgram=0; //reset program max
   digitalWrite(9,LOW); //CLR Stays high ALWAYAS
   delay(10);
   digitalWrite(9,HIGH); //CLR Stays high ALWAYAS
    
   Serial.println("!CLEAR_ALL"); 
  }

void clearDac()
  {
  byte offSet = 10;
  char instring[15] = {}; //create a string to hold char ascii for later conversion to integer value
  for(byte sl=0;sl<inputString.length()-offSet;++sl)
  { //figure out length of string and calc # characters which (should be) numbers for DAC
  instring[sl] = inputString[sl+offSet]; //assign 7th and farther numbers to char table
  //Serial.print(fstring[sl]);
  }
  byte inLine = atoi(instring); //convert char table to useable integer for DAC level
  inLine = inLine - 1;
  for(byte i = 0 ; i<50;++i)
    {
    dacArray[i][inLine] = 0;  
    }
  Serial.print("!CLEAR_DAC,"); //print recieve message to operator
  Serial.println(inLine+1);
  }

void clearTtl()
  {
  byte offSet = 10;
  char instring[15] = {}; //create a string to hold char ascii for later conversion to integer value
  for(byte sl=0;sl<inputString.length()-offSet;++sl)
  { //figure out length of string and calc # characters which (should be) numbers for DAC
  instring[sl] = inputString[sl+offSet]; //assign 7th and farther numbers to char table
  //Serial.print(fstring[sl]);
  }
  byte inLine = atoi(instring); //convert char table to useable integer for DAC level
  inLine = inLine - 1;
  for(byte i = 0 ; i<50;++i)
    {
    ttlArray[i][inLine] = 0;  
    }
  Serial.print("!CLEAR_TTL,"); //print recieve message to operator
  Serial.println(inLine+1);
  }

void clearDelay()
  {
  for(byte i = 0 ; i<50;++i)
    {
    delArray[i]= 0;  
    }
  Serial.println("!CLEAR_DELAY"); //print recieve message to operator
    
  }

void clearFocus()
  {
  for(byte i = 0 ; i<6;++i)
    {
    focArray[i]= 0;  
    }
  Serial.println("!CLEAR_FOCUS"); //print recieve message to operator
  }

void setDacRange(byte dacLine, byte range)
  {
    /*dacLine 0 indexed   
     * range 0-5 corresponds to the following:
        SPAN_0_TO_5V             0x0000
        SPAN_0_TO_10V            0x0001
        SPAN_PLUS_MINUS_5V       0x0002
        SPAN_PLUS_MINUS_10V      0x0003
        SPAN_PLUS_MINUS_2V5      0x0004
     */
  dac_write(10,1,dacLine,range); //issue span command
  dac_write(10,2,0,0); //issue update all lines command
  RNG[dacLine] = range;
  }


void spanTest()
  {
  
  }
     

void speedtest() {
  Serial.println("Starting Speed Test...");
  clearSerial();

  for ( byte n=0;n<10;++n){
    setTTL(16,1);
    //delay(1);
    setTTL(16,0);
    //delay(0);
    //delayMicroseconds(10);
  }


  /*
  while(Serial.available() == 0) {
   for(volatile int f=10000;f<40001;f=f+5000){
        while(!inTrigger){}//wait...
        dac_write(10,0, 0, f); // Set DAC Lines
        dac_write(10,0, 1, 40000-f); // Set DAC Lines    
        inTrigger=false;
      } 
  }

  */
}

int8_t dac_write(uint8_t cs, byte command, uint8_t dac_address, uint16_t dac_code) {
// Write the 16-bit dac_code 
  /*
  Serial.print(" DW=");
  Serial.print( dac_address);
  Serial.print(",");
  Serial.println(dac_code);
  */
  static uint8_t last_data_array[4];
  uint8_t data_array[4], rx_array[4];
  int8_t ret;
  LT_union_int16_2bytes data;

  data.LT_int16 = dac_code;                              // Copy DAC code
  data_array[3] = 0;                                     // Only required for 32 byte readback transaction
  if(command == 0) { data_array[2] = 0x30 | dac_address;}            // Build command / address byte
  if(command == 1) { data_array[2] = 0x60 | dac_address;}            // span dac
  if(command == 2) { data_array[2] = 0xA0 | dac_address;}
  // ***NOT SURE WHY BUT THIS MUST BE OFF! data_array[2] = 0x30 | dac_address;             // Build command / address byte
  data_array[1] = data.LT_byte[1];                       // MS Byte
  data_array[0] = data.LT_byte[0];                       // LS Byte
/*
#define  LTC2668_CMD_WRITE_N              0x00  //!< Write to input register n
#define  LTC2668_CMD_UPDATE_N             0x10  //!< Update (power up) DAC register n
#define  LTC2668_CMD_WRITE_N_UPDATE_ALL   0x20  //!< Write to input register n, update (power-up) all
#define  LTC2668_CMD_WRITE_N_UPDATE_N     0x30  //!< Write to input register n, update (power-up) 
#define  LTC2668_CMD_POWER_DOWN_N         0x40  //!< Power down n
#define  LTC2668_CMD_POWER_DOWN_ALL       0x50  //!< Power down chip (all DAC's, MUX and reference)
#define  LTC2668_CMD_SPAN                 0x60  //!< Write span to dac n
#define  LTC2668_CMD_CONFIG               0x70  //!< Configure reference / toggle
#define  LTC2668_CMD_WRITE_ALL            0x80  //!< Write to all input registers
#define  LTC2668_CMD_UPDATE_ALL           0x90  //!< Update all DACs
#define  LTC2668_CMD_WRITE_ALL_UPDATE_ALL 0xA0  //!< Write to all input reg, update all DACs
#define  LTC2668_CMD_MUX                  0xB0  //!< Select MUX channel (controlled by 5 LSbs in data word)
#define  LTC2668_CMD_TOGGLE_SEL           0xC0  //!< Select which DACs can be toggled (via toggle pin or global toggle bit)
#define  LTC2668_CMD_GLOBAL_TOGGLE        0xD0  //!< Software toggle control via global toggle bit
#define  LTC2668_CMD_SPAN_ALL             0xE0  //!< Set span for all DACs
#define  LTC2668_CMD_NO_OPERATION         0xF0  //!< No operation
*/


  spi_transfer_block(cs, data_array, rx_array, (uint8_t) 4);
  // Compare data read back to data that was sent the previous time this function was called
  if ((rx_array[2] == last_data_array[2]) && (rx_array[1] == last_data_array[1]) && (rx_array[0] == last_data_array[0]))
  {
    ret = 0;
  }
  else
  {
    ret = 1;
  }

  last_data_array[0] = data_array[0]; // Copy data array to a static array to compare
  last_data_array[1] = data_array[1]; // the next time the function is called
  last_data_array[2] = data_array[2];

  return(ret);
}

void spi_transfer_block(uint8_t cs_pin, uint8_t *tx, uint8_t *rx, uint8_t length)
{
  int8_t i;
  output_low(cs_pin);                 //! 1) Pull CS low

  for (i=(length-1);  i >= 0; i--)
    rx[i] = SPI.transfer(tx[i]);    //! 2) Read and send byte array

  output_high(cs_pin);                //! 3) Pull CS high
}



  
/*******************LICENSING INFO**************

 * Copyright (c) 2018, Advanced Research Consulting 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 * 
 *******************/
