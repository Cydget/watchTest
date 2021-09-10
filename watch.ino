/*

  Udp NTP Client

  Get the time from a Network Time Protocol (NTP) time server
  Demonstrates use of UDP sendPacket and ReceivePacket
  For more on NTP time servers and the messages needed to communicate with them,
  see http://en.wikipedia.org/wiki/Network_Time_Protocol

  created 4 Sep 2010
  by Michael Margolis
  modified 9 Apr 2012
  by Tom Igoe
  updated for the ESP8266 12 Apr 2015
  by Ivan Grokhotkov

  This code is in the public domain.

*/
#define DEBUG false  //set to true for debug output, false for no debug output
#define flipUpsideDown true
#include "uart_register.h"
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <time.h>
#include <Wire.h>
#define ALARMMAXTIME 20
#ifndef STASSID
#define STASSID "Kodi"
#define STAPSK  "pinkmoon963"
#endif
#define DEBUG_SERIAL if(DEBUG)Serial
#define DISPLAYI2CDATA false
//#define DEBUG_SERIAL.end() DEBUG_SERIAL.end(); pinMode(3,INPUT)

const char * ssid = STASSID; // your network SSID (name)
const char * pass = STAPSK;  // your network password

 
unsigned int localPort = 2390;      // local port to listen for UDP packets
#define NTP_PACKET_SIZE 48 // NTP time stamp is in the first 48 bytes of the message
IPAddress timeServerIP; // time.nist.gov NTP server address
const char* ntpServerName = "time.nist.gov";
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
time_t rawtime;
time_t alarmTime;
bool alarmSet;
bool alarmTriggered;
time_t alarmTriggeredTime;
bool firstRunUpdate;
unsigned int secondsSinceLastNTPTimeUpdate;
const int maxSecondsBeforeNextNTPUpdate=2000;
// A UDP instance to let us send and receive packets over UDP
WiFiUDP udp;
byte displayValue;
bool is24HRmode;
bool decimalPointFlag;
int currentDisplayDigit;
byte LEDBRIGHTNESS;
const byte RTCAddress=0b1101000;
bool turnedOnSWEOUT;
const byte displayDigit[]={ 0b11111100,//0
                      0b01100000,//1
                      0b11011010,//2
                      0b11110010,//3
                      0b01100110,//4
                      0b10110110,//5
                      0b10111110,//6
                      0b11100000,//7
                      0b11111110,//8
                      0b11110110,//9
                      0b11101110,//A
                      0b00111110,//b
                      0b00011010,//c
                      0b01111010,//d
                      0b10011110,//E
                      0b10001110,//F
                      0b00000000//all empty
                    };

enum Buttons{NONE,TOP,MIDDLE,BOTTOM};

Buttons buttonState;
void flashLEDS(int seconds){
  int delayTime=100;
  for(int i=0;i<1000/(seconds*delayTime);i++){
    LEDBRIGHTNESS=255;
    displayValue=8;
    updateSegments();
    delay(delayTime>>1);
    LEDBRIGHTNESS=0; 
    updateSegments();
    delay(delayTime>>1);
    removeAlarmCheck();
  }
}
void removeAlarmCheck(){
    if(rawtime-alarmTriggeredTime>=ALARMMAXTIME){
      alarmTriggered=true;
    }
    processButtonPress(analogRead(A0));
    switch(buttonState){
      case TOP:
      case MIDDLE:
      case BOTTOM:
        alarmTriggered=true;
        break;
      default:
        break;
    }
}
void doAlarmAction(){
  flashLEDS(5);//will have to check button state in 
}
void checkAlarm(){
  if(alarmSet && !alarmTriggered && rawtime>=alarmTime){
    alarmTriggeredTime=rawtime;
    while(!alarmTriggered){
      doAlarmAction();
    }
  }
}
void setAlarm(int Hours,int Minutes,int Seconds, bool fromNow){//if fromNow is true, it adds to current time. Otherwise it will happen at that time
  alarmSet=true; //needs some work to handle roll over #TODO
  alarmTriggered=false; 
  
  struct tm* timeinfo;
  timeinfo = localtime(&rawtime);
  alarmTime=mktime(timeinfo );
  if(fromNow){
    alarmTime+=Seconds;
    alarmTime+=Minutes*60;
    alarmTime+=Hours*60*60;
  }
  else{//assumes same year month and day
    timeinfo->tm_hour=Hours;
    timeinfo->tm_min=Minutes;
    timeinfo->tm_sec=Seconds;
    //timeinfo->tm_year = Year-1900;
    //timeinfo->tm_mon = Month-1;
    //timeinfo->tm_mday = Date;
    alarmTime=mktime(timeinfo );
  }
}

void setup() {
  Wire.begin();//pins 4 and 5
  delay(100);
  writeRTCAddress(0x7, 0b10000000);//start with rtc swe pulled high.
  firstRunUpdate=false;
  secondsSinceLastNTPTimeUpdate=0;
  decimalPointFlag=false;
  displayValue=0;
  LEDBRIGHTNESS=50;
  buttonState=NONE;
  rawtime=41;
  pinMode(12,OUTPUT);//clock shift
  pinMode(15,OUTPUT);//led enable
  
  pinMode(13,OUTPUT);//shock enable
  pinMode(14,OUTPUT);//charge enable
  pinMode(A0,INPUT);
  digitalWrite(13,LOW);
  digitalWrite(14,LOW);

  //pinMode(D0,INPUT);

  updateSegments();
  delay(100);
  writeRTCAddress(0x7, 0b10000000);//start with rtc swe pulled high.
  turnedOnSWEOUT=false;
  //pinMode(3,FUNCTION_3);

  currentDisplayDigit=0;
  is24HRmode=false;

  Serial.begin(115200);
  install_uart_tout();
}
void ledDisplayTime(){
  int returnbrightness=255;
  LEDBRIGHTNESS=returnbrightness;
  struct tm* timeinfo;
  for(int onDigit=0;onDigit<6;onDigit++){
      timeinfo = localtime(&rawtime);
      int hr=timeinfo->tm_hour;
      if(!is24HRmode){
        if(hr>12){hr-=12;
            //decimalPointFlag=true;
        }
        else{decimalPointFlag=false;
        }
      }
      switch(onDigit){
        case 0: displayValue=(hr-(hr%10))/10; break;
        case 1: displayValue= hr%10; break;
        case 2: displayValue=(timeinfo->tm_min-(timeinfo->tm_min%10))/10; break;
        case 3: displayValue= timeinfo->tm_min%10; break;  
        case 4: displayValue=(timeinfo->tm_sec-(timeinfo->tm_sec%10))/10; break;
        case 5: displayValue= timeinfo->tm_sec%10; break;  
        default: displayValue=15; break;
      }
      updateSegments();
      delay(300);
      LEDBRIGHTNESS=0;
      updateSegments();
      delay(100);
      if(onDigit%2==1){
        delay(400);
      }
      LEDBRIGHTNESS=returnbrightness;

  }
  LEDBRIGHTNESS=0;
  updateSegments();
}
void RTCIntCallback(){
//   incDisplayVal();
   rawtime++;
   //updateSegments();//disable later?
}
void loop() {
  //get a random server from the pool
  if(firstRunUpdate||(millis()/1000-secondsSinceLastNTPTimeUpdate)>=maxSecondsBeforeNextNTPUpdate){//update time if we didnt do it for a long time
    if(WiFi.status()!=WL_CONNECTED){
      connect2Wifi();
    }
    if(doPacketThing()){
      WiFi.disconnect();
    };
  }
  mainLoopButtonLogic();
  //serialPrintTime();
  if(!turnedOnSWEOUT && (millis()>10000)){
    writeRTCAddress(0x7, 0b10010000);//start with rtc swe pulled high.
    turnedOnSWEOUT=true;
    setAlarm(0,0,20, true);
    ledDisplayTime();
  }  
  if(!turnedOnSWEOUT){//we can only get time by reading from what is
    readTimeFromRTC();
  }
  checkAlarm();
  //prints time to leds on every minute
  //incDisplayVal();
  //updateSegments();
  idelDisplay();
  delay(1);
  //ESP.deepSleep(20e6);
  if(millis()>20000){
    enterDeepSleep();
  }
}
void enterDeepSleep(){
      writeRTCAddress(0x7, 0b10000000);//start with rtc swe pulled high.
      displayValue=16;
      LEDBRIGHTNESS=0;
      updateSegments();
      pinMode(1,FUNCTION_3);
      pinMode(1,OUTPUT);
      digitalWrite(1,0);
      digitalWrite(12,LOW);
      digitalWrite(12,HIGH);
      delay(1);
      ESP.deepSleep(ESP.deepSleepMax());
}
void mainLoopButtonLogic(){
    processButtonPress(analogRead(A0));
  switch(buttonState){
    case TOP:
      firstRunUpdate=true;
      break;
    case MIDDLE:
      setAlarm(0,0,20, true);
      break;
    case BOTTOM:
      ledDisplayTime();
      break;
    default:
      break;
  }
  
  }
void idelDisplay(){
  if(rawtime%60==59) ledDisplayTime();
  }
void incDisplayVal(){
    displayValue++;
  if(displayValue>=16){
    displayValue=0;
  }
}
void connect2Wifi(){
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.print("Connecting to ");
  DEBUG_SERIAL.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_SERIAL.print(".");
  }
  DEBUG_SERIAL.println("");

  DEBUG_SERIAL.println("WiFi connected");
  DEBUG_SERIAL.println("IP address: ");
  DEBUG_SERIAL.println(WiFi.localIP());

  DEBUG_SERIAL.println("Starting UDP");
  udp.begin(localPort);
  DEBUG_SERIAL.print("Local port: ");
  DEBUG_SERIAL.println(udp.localPort());
  DEBUG_SERIAL.end();
  }
void processButtonPress(int val){
    if(val<=20){
    buttonState=TOP;
    }
  else if(val<=720){
    buttonState=MIDDLE;
    }
  else if(val<=900){
    buttonState=BOTTOM;
    }
   else{
    buttonState=NONE;
    }
  }
void writeRTCAddress(byte WordAddr, byte data){
  Wire.beginTransmission(RTCAddress);
  Wire.write(WordAddr);
  Wire.write(data);
  Wire.endTransmission();
  }
void writeRTCNDATA(byte WordAddr, byte* data,int count){
  Wire.beginTransmission(RTCAddress);
  Wire.write(WordAddr);
  for(int i=0;i<count;i++){
     Wire.write(*(data+i));
  }  
  Wire.endTransmission();
  }
void readRTCAddress(byte WordAddr, int bytecount, byte ** dataRet){

  byte *data;
  data = (byte *) malloc(bytecount);
  memset(data,0,bytecount);
  Wire.beginTransmission(RTCAddress);
  Wire.write(WordAddr);
  Wire.endTransmission();
  Wire.requestFrom(RTCAddress,bytecount);
  byte offset=0;
  while(Wire.available()){
    *(data+offset)=Wire.read();
    offset+=1;
  }
  if(DISPLAYI2CDATA){
      DEBUG_SERIAL.begin(115200);
      DEBUG_SERIAL.print("\n");
      for(byte i=0;i<bytecount;i++)
      {
        for(int j=7;j>=0;j--){
              if((*(data+i))&(1<<j)){DEBUG_SERIAL.print("| 1 ");}
              else{DEBUG_SERIAL.print("| 0 ");}
        }
        DEBUG_SERIAL.print("\n-------------------------------\n");
      }
      DEBUG_SERIAL.print("\n");
      DEBUG_SERIAL.end();
  }
  *dataRet=data;
  //free(data);
}

void readTimeFromRTC(){
  byte * data;
  readRTCAddress(0x0, 7,&data);
  byte * TimeZone;
  readRTCAddress(0x8,1,&TimeZone);
  int seconds = 10*((data[0]&0b01110000)>>4) + (data[0]&0b00001111);
  int minutes = 10*((data[1]&0b01110000)>>4) + (data[1]&0b00001111);
  int hours;
  if(data[2]&0b01110000){//12 hr format may need to flip control bit
      hours = 12*((data[2]&0b00100000)>>5) + 10*((data[2]&0b00010000)>>4) + (data[2]&0b00001111);
  }
  else{//24 hr format
      hours = 10*((data[2]&0b00110000)>>4) + (data[2]&0b00001111);
  }
  int DayOfWeek = data[3]&0b00000111;//unused
  int Date =  10*((data[4]&0b00110000)>>4) + (data[4]&0b00001111);
  
  int Month = 10*((data[5]&0b00010000)>>4) + (data[5]&0b00001111);
  
  int Year =  10*((data[6]&0b11110000)>>4) + (data[6]&0b00001111)+2000;
  struct tm* timeinfo;
  timeinfo = localtime(&rawtime);
 
  timeinfo->tm_year = Year-1900;
  timeinfo->tm_mon = Month-1;
  timeinfo->tm_mday = Date;
  timeinfo->tm_hour = hours;
  timeinfo->tm_min = minutes;
  timeinfo->tm_sec = seconds;
  rawtime=mktime(timeinfo );
  
  free(TimeZone);
  free(data);
}


void updateSegments(){
  //return;
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("                                                                   ");
  DEBUG_SERIAL.println("                                                                   ");
  DEBUG_SERIAL.end();
  pinMode(1,FUNCTION_3);
  pinMode(1,OUTPUT);
  //delay(2);
  digitalWrite(15,HIGH);
  byte data=displayDigit[displayValue]|decimalPointFlag;
  
  if(flipUpsideDown){
    byte back=data&0b11100000;
    data&=0b00011111;
    data|= (data&0b00011100)<<3;
    data&= 0b11100011;
    data|= back>>3;
  }
  
  for(int i=0;i<8;i++){
    digitalWrite(12,LOW);
    //delay(2);
    digitalWrite(1,(data&1<<i)>>i);
    //delay(1);
    digitalWrite(12,HIGH);
    //delay(2);
  }
  digitalWrite(12,LOW);
  analogWrite(15,255-LEDBRIGHTNESS);
  pinMode(1,FUNCTION_0);
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("                                                                    ");
  DEBUG_SERIAL.println("                                                                    ");
  DEBUG_SERIAL.end();
  }
bool doPacketThing(){    //returns true if sucessful else false 
    DEBUG_SERIAL.begin(115200);
    WiFi.hostByName(ntpServerName, timeServerIP);
    sendNTPpacket(timeServerIP); // send an NTP packet to a time server
    delay(1000);  // wait to see if a reply is available
    int cb = udp.parsePacket();
      if (!cb) {
        delay(5000);
        DEBUG_SERIAL.println("no packet yet");
        DEBUG_SERIAL.end();
        return false;
      }
      else {    
        secondsSinceLastNTPTimeUpdate=millis()/1000;
        DEBUG_SERIAL.print("packet received, length=");
        DEBUG_SERIAL.println(cb);
        updateTimeFromNTP();
        serialPrintTime();
        setRTCDate();
        firstRunUpdate=false;
        DEBUG_SERIAL.end();
        return true;
      }
   DEBUG_SERIAL.end();
   return false;   
  }
void setRTCDate(){//sunday 11am: 56 min : 59 sec
    //writeRTCAddress(0x0, 0b00000000);//59 seconds
    //writeRTCAddress(0x1, 0b00000000);//56 minutes
    //writeRTCAddress(0x2, 0b00000000);//12hr am  hr 11
    //writeRTCAddress(0x3, 0b00000000);//sunday
    //writeRTCAddress(0x4, 0b00000000);//8
    //writeRTCAddress(0x5, 0b00000000);//sep
    //writeRTCAddress(0x6, 0b00000000);//
    //writeRTCAddress(0x7, 0b00000000);//output off and stuff
    
    struct tm* timeinfo;
    timeinfo = localtime(&rawtime);
    writeRTCAddress(0x0, (timeinfo->tm_sec%10+(((timeinfo->tm_sec-(timeinfo->tm_sec%10))/10)<<4))&0b01111111);
    writeRTCAddress(0x1, (timeinfo->tm_min%10+(((timeinfo->tm_min-(timeinfo->tm_min%10))/10)<<4))&0b01111111);
    writeRTCAddress(0x2, ((timeinfo->tm_hour%10+(((timeinfo->tm_hour-(timeinfo->tm_hour%10))/10)<<4))&0b00111111)|0b01000000);
    writeRTCAddress(0x3,(timeinfo->tm_wday)&0b00000111);
    writeRTCAddress(0x4,(timeinfo->tm_mday%10+(((timeinfo->tm_mday-(timeinfo->tm_mday%10))/10)<<4))&0b00111111);
    writeRTCAddress(0x5,((1+timeinfo->tm_mon)%10+(((timeinfo->tm_mon-(timeinfo->tm_mon%10))/10)<<4))&0b00011111);
    int yr=(1900+timeinfo->tm_year)-2000;
    writeRTCAddress(0x6,(yr%10)+(((yr-(yr%10))/10)<<4));
    byte mstr[]="lol fucker";
    writeRTCNDATA(0x9,mstr ,sizeof(mstr));
  }
void updateTimeFromNTP(){
    udp.read(packetBuffer, NTP_PACKET_SIZE); 
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    const unsigned long seventyYears = 2208988800UL;
    const unsigned long hourCount=3600UL;
    int timezoneOffset= -4;
    rawtime = secsSince1900 - seventyYears+timezoneOffset*hourCount;//may need to record offset from packet request
  }
void serialPrintTime(){
    DEBUG_SERIAL.begin(115200);
    // print Unix time:
    DEBUG_SERIAL.println("The Time is: ");
    DEBUG_SERIAL.println(ctime (&rawtime));
    DEBUG_SERIAL.end();
  }
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address) {
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("sending NTP packet...");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
//  DEBUG_SERIAL.end(115200);
}



void uart0_rx_intr_handler(void *para){
  uint8_t RcvChar;
  uint8_t uart_no = UART0;
  uint8_t fifo_len = 0;
  uint8_t buf_idx = 0;
  uint32_t uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no));//get uart intr status
  while(uart_intr_status != 0x0) {
    if (UART_FRM_ERR_INT_ST == (uart_intr_status & UART_FRM_ERR_INT_ST)){ // if it is caused by a frm_err interrupt
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_FRM_ERR_INT_CLR);
      DEBUG_SERIAL.println("caused by a frm_err interrupt");
    } else if (UART_RXFIFO_FULL_INT_ST == (uart_intr_status & UART_RXFIFO_FULL_INT_ST)) { //if it is caused by a fifo_full interrupt
      DEBUG_SERIAL.println("caused by a fifo_full interrupt");
      fifo_len = (READ_PERI_REG(UART_STATUS(uart_no)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT; //read rf fifo length
      char r[fifo_len];
      buf_idx = 0;
      while (buf_idx < fifo_len) {
        r[buf_idx] = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
        buf_idx++;
      }
      r[fifo_len] = '\0';
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_FULL_INT_CLR); //clear full interrupt state
    } else if (UART_RXFIFO_TOUT_INT_ST == (uart_intr_status & UART_RXFIFO_TOUT_INT_ST)) { //if it is caused by a time_out interrupt
      //Serial.println("caused by a time_out interrupt");
      RTCIntCallback();
      fifo_len = (READ_PERI_REG(UART_STATUS(uart_no)) >> UART_RXFIFO_CNT_S) & UART_RXFIFO_CNT; //read rf fifo length
      char r[fifo_len];
      buf_idx = 0;
      while (buf_idx < fifo_len) {
        r[buf_idx] = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
        buf_idx++;
      }
      r[fifo_len] = '\0';
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_TOUT_INT_CLR); //clear full interrupt state
    } else if (UART_TXFIFO_EMPTY_INT_ST == (uart_intr_status & UART_TXFIFO_EMPTY_INT_ST)) { //if it is caused by a tx_empty interrupt
      DEBUG_SERIAL.println("caused by a tx_empty interrupt");
      WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_TXFIFO_EMPTY_INT_CLR);
      CLEAR_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_TXFIFO_EMPTY_INT_ENA);
    } else {

    }
    uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)); //update interrupt status
  }
}

static void install_uart_tout(){
  ETS_UART_INTR_DISABLE();
  ETS_UART_INTR_ATTACH(uart0_rx_intr_handler, NULL);

  WRITE_PERI_REG(UART_CONF1(0), UART_RX_TOUT_EN |
    ((0x2 & UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S));

  WRITE_PERI_REG(UART_INT_CLR(0), 0xffff);
  SET_PERI_REG_MASK(UART_INT_ENA(0), UART_RXFIFO_TOUT_INT_ENA);
  CLEAR_PERI_REG_MASK(UART_INT_ENA(0), UART_RXFIFO_FULL_INT_ENA);

  ETS_UART_INTR_ENABLE();
}
