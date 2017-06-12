#include "cc1101.h"

// The LED is wired to the Arduino Output 4 (physical panStamp pin 19)
#define LEDOUTPUT 9
//Status LEDs
#define LEDOUTPUT1 4
#define LEDOUTPUT2 5
#define LEDOUTPUT3 6
#define LEDOUTPUT4 7


//Number of Sensor Readings to save in Memory
#define SENSORDBSIZE 10

//Skip packets where crc fails
boolean SKIP_FAILED_CRC = true;

//Automatically cleanup sensor data array
boolean AUTO_CLEAN_DB = true;

//Max Age of Sensor Reading for auto cleanup (default 1h = 3600000 ms) 
#define MAXDATAAGE 3600000 

//Auto Cleanup Interval (default: clean once every 30 min = 3600000 ms)
#define AUTOCLEANINTERVAL 1800000 
unsigned long lastCleanTime = 0;

// The connection to the hardware chip CC1101 the RF Chip
CC1101 cc1101;
CCPACKET packet;

// a flag that a wireless packet has been received
volatile boolean packetAvailable = false;

//Packet counter
unsigned long pcount;

//Data Structure for a Sensor Reading
typedef struct
{
  word sensorID;// 2bytes Sensor ID
  unsigned long timestamp; // 4bytes
  byte packetCount; // 1byte
  byte moistLevel; //1byte
  byte battLevel; //1byte
  bool crc;
  byte rssi;
  byte lqi;
} sensorData_type;

//Array of past sensor readings 90bytes
sensorData_type sensorData[SENSORDBSIZE];


// Current position in the sensorDataDB
int currentSensorDataPos = 0;


/*
   Lookup Sensor ID in Sensor Data
   Used to update a sensor Reading in the data structure
*/

int lookupSensorInDB(short sID) {
  for (int i = 0; i < SENSORDBSIZE; i++)
    if (sensorData[i].sensorID == sID)
      return i;

  //value not found: return SENSORDBSIZE + 1
  return SENSORDBSIZE + 1;

}

/*
   Update sensor Data for a Sensor ID
   Update an existing Sensor Reading with new data
*/

void updateSensorDataInDB(int pos, sensorData_type sData) {
  sensorData[pos] = sData;
}

/*
   Add Sensor Reading and ID to Sensor Data Array
*/
void addSensorDataToDB(sensorData_type sData) {
  //store Data
  sensorData[currentSensorDataPos] = sData;
  //increase data pointer
  currentSensorDataPos++;
  //reset pointer value if too large
  if (currentSensorDataPos > SENSORDBSIZE)
    currentSensorDataPos = 0;
}


/*
   Add or Update Sensor Reading in Sensor Data Array
*/

void addOrUpdateSensorData(sensorData_type sData) {
  //check if SensorID already exists in Data Array
  int dpos = lookupSensorInDB(sData.sensorID);
  //Serial.print("add dpos:");
  //Serial.println(dpos);
  //check if sensor ID already exists
  if (dpos != SENSORDBSIZE + 1)
  {
    //Serial.println("Update");
    //sensor ID exists - > update existing record
    updateSensorDataInDB (dpos, sData);
  }
  else
  {
    //Serial.println("Add");
    //sensor ID is not present in Data Array -> add new record
    addSensorDataToDB(sData);
  }
}



/*
 * dump all data from Sensor Data Array
*/

void dumpSensorDataArray() {
  //sanity check
  if (currentSensorDataPos > SENSORDBSIZE) {
    currentSensorDataPos = 0;
    Serial.println(F("Sensor Data corrupt. Erasing..."));
    return;
  }
  Serial.print(F("Num saved records: "));
  Serial.println(currentSensorDataPos);
  for (int i = 0; i < currentSensorDataPos; i++) {
    Serial.print(F("Sensor ID: "));
    printHex(sensorData[i].sensorID, 4);
    Serial.print(F(", Packet count: "));
    Serial.print(sensorData[i].packetCount, DEC);
    Serial.print(F(", Moisture level: "));
    Serial.print(sensorData[i].moistLevel, DEC);
    Serial.print(F(", Battery level: "));
    Serial.print(sensorData[i].battLevel, DEC);
    Serial.print(F(", CRC: "));
    Serial.print(sensorData[i].crc, DEC);
    Serial.print(F(", RSSI: "));
    Serial.print(sensorData[i].rssi, DEC);
    Serial.print(F(", LQI: "));
    Serial.print(sensorData[i].lqi, DEC);
    Serial.print(F(", Age: "));
    Serial.println(uptime(millis() - sensorData[i].timestamp));
  }
  Serial.println();

}

/*
 * Cleans Sensor Data Array. Removes all Values with Age > 1hour 
 */
void cleanupSensorDataArray() {
  //define new temporary array
  sensorData_type sensorDataTemp[SENSORDBSIZE];
  Serial.println(F("Old Sensor Data Array entries: "));
  Serial.println(currentSensorDataPos);
  
  int tempDataPos = 0;
  //loop through existing array
  for (int i = 0; i < currentSensorDataPos; i++) {
    //check age for every item
    //if age less than maxage
    //move to temp array

    if (millis() - sensorData[i].timestamp < MAXDATAAGE) {
      sensorDataTemp[tempDataPos] = sensorData[i];
      tempDataPos++;
    }
  }

  //set Sensor Data Array = TempArray
  memcpy(sensorData, sensorDataTemp, sizeof(sensorDataTemp[0])*tempDataPos);
  currentSensorDataPos = tempDataPos;
  Serial.println(F("New Sensor Data Array entries: "));
  Serial.println(currentSensorDataPos);
  
}

void blinker(byte ledPin) {
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);
}

char *uptime() // Function made to millis() be an optional parameter
{
  return (char *)uptime(millis()); // call original uptime function with unsigned long millis() value
}

char *uptime(unsigned long milli)
{
  static char _return[32];
  unsigned long secs = milli / 1000, mins = secs / 60;
  unsigned int hours = mins / 60, days = hours / 24;
  milli -= secs * 1000;
  secs -= mins * 60;
  mins -= hours * 60;
  hours -= days * 24;
  sprintf(_return, "%dd %2.2d:%2.2d:%2.2d.%3.3d", (byte)days, (byte)hours, (byte)mins, (byte)secs, (int)milli);
  return _return;
}



/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
void cc1101signalsInterrupt(void) {
  // set the flag that a package is available
  packetAvailable = true;
}

void setup()
{
  Serial.begin(115200);

  // setup the blinker output
  pinMode(LEDOUTPUT, OUTPUT);
  digitalWrite(LEDOUTPUT, LOW);

  // blink every LED once to signal the setup
  blinker(LEDOUTPUT);
  blinker(LEDOUTPUT1);
  blinker(LEDOUTPUT2);
  blinker(LEDOUTPUT3);
  blinker(LEDOUTPUT4);

  // initialize the RF Chip
  cc1101.init();
  cc1101.setCarrierFreq(CFREQ_868);
  cc1101.disableAddressCheck(); //if not specified, will only display "packet received"

  // print status information
  Serial.print(F("CC1101_PARTNUM ")); //cc1101=0
  Serial.println(cc1101.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
  Serial.print(F("CC1101_VERSION ")); //cc1101=4
  Serial.println(cc1101.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
  Serial.print(F("CC1101_MARCSTATE "));
  Serial.println(printMARCSTATE(cc1101.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f));

  packetAvailable = false;
  pcount = 0;
  attachInterrupt(digitalPinToInterrupt(GDO0), cc1101signalsInterrupt, FALLING);

  Serial.println(F("device initialized"));

}

String printMARCSTATE(byte state) {
  switch (state) {
    case 0: return F("(0x00) SLEEP SLEEP");
    case 1: return F("(0x01) IDLE IDLE");
    case 2: return F("(0x02) XOFF XOFF");
    case 3: return F("(0x03) VCOON_MC MANCAL");
    case 4: return F("(0x04) REGON_MC MANCAL");
    case 5: return F("(0x05) MANCAL MANCAL");
    case 6: return F("(0x06) VCOON FS_WAKEUP");
    case 7: return F("(0x07) REGON FS_WAKEUP");
    case 8: return F("(0x08) STARTCAL CALIBRATE");
    case 9: return F("(0x09) BWBOOST SETTLING");
    case 10: return F("(0x0A) FS_LOCK SETTLING");
    case 11: return F("(0x0B) IFADCON SETTLING");
    case 12: return F("(0x0C) ENDCAL CALIBRATE");
    case 13: return F("(0x0D) RX RX");
    case 14: return F("(0x0E) RX_END RX");
    case 15: return F("(0x0F) RX_RST RX");
    case 16: return F("(0x10) TXRX_SWITCH TXRX_SETTLING");
    case 17: return F("(0x11) RXFIFO_OVERFLOW RXFIFO_OVERFLOW");
    case 18: return F("(0x12) FSTXON FSTXON");
    case 19: return F("(0x13) TX TX");
    case 20: return F("(0x14) TX_END TX");
    case 21: return F("(0x15) RXTX_SWITCH RXTX_SETTLING");
    case 22: return F("(0x16) TXFIFO_UNDERFLOW TXFIFO_UNDERFLOW");
    default: return F("MARCSTATE unknown");
  }
}

String printPKTSTATUS(byte state) {
  //bitwise check values of state
  String pktstatus = "";

  pktstatus = "GDO0:";
  pktstatus += (state & (1 << 0)) ? true : false;
  pktstatus += " GDO2:";
  pktstatus += (state & (1 << 2)) ? true : false;
  pktstatus += " SFD:";
  pktstatus += (state & (1 << 3)) ? true : false;
  pktstatus += " CCA:";
  pktstatus += (state & (1 << 4)) ? true : false;
  pktstatus += " PQT_REACHED:";
  pktstatus += (state & (1 << 5)) ? true : false;
  pktstatus += " CS:";
  pktstatus += (state & (1 << 6)) ? true : false;
  pktstatus += " CRC:";
  pktstatus += (state & (1 << 7)) ? true : false;

  return pktstatus;

}

/*
   Decodes a Kärcher soil moisture sensor packet, prints the information on the serial interface and returns a sensorData struct
   Packet length is 6 bytes
   byte[0] = Packet counter / num packets transmitted by sensor. Seems to increase up to FF (255) but no further.
   byte[1] = Moisture level. Range 0-7
   byte[2] = Battery level. Range 0-3
   byte[3] = Sensor ID LSB (least significant byte)
   byte[4] = Sensor ID MSB (most significant byte)

*/
void decodePacket(CCPACKET * packet, sensorData_type* sData)
{
  //boolean set to true when the packet has an error
  bool packetError = false;


  Serial.print(F("Sensor ID: "));
  //printHex(packet->data[3],2);
  //printHex(packet->data[4],2);


  word sID = word(packet->data[3], packet->data[4]);
  printHex(sID, 4);

  Serial.print(F(", Packet count: "));
  Serial.print(packet->data[0], DEC);

  Serial.print(F(", Moisture level: "));
  if (packet->data[1] <= 7)
    Serial.print(packet->data[1], DEC);
  else {
    Serial.print(F("ERR"));
    packetError = true;
  }



  Serial.print(F(", Battery level: "));
  if (packet->data[2] <= 3)
    Serial.print(packet->data[2], DEC);
  else {
    Serial.print(F("ERR"));
    packetError = true;
  }

  if (!packetError) {
    sData->sensorID = sID;
    sData->timestamp = millis();
    sData->packetCount = packet->data[0];
    sData->moistLevel = packet->data[1];
    sData->battLevel = packet->data[2];
    sData->crc = packet->crc_ok;
    sData->rssi = packet->rssi;
    sData->lqi = packet->lqi;
  }
  else {
    //in case of packet error, return sensor id zero
    sData->sensorID = 0;
  }

}


void updateStatusLED(byte MARCSTATE, byte PKTSTATUS, byte BytesInRXFIFO) {
  /*
     LEDOUTPUT1: MARCSTATE=RX
     LEDOUTPUT2: GDO0
     LEDOUTPUT3: RXFIFO Bytes > 0
     LEDOUTPUT4: GDO2

  */
  if (MARCSTATE == 13)
    digitalWrite(LEDOUTPUT1, HIGH);
  else
    digitalWrite(LEDOUTPUT1, LOW);

  if (PKTSTATUS & (1 << 0))
    digitalWrite(LEDOUTPUT2, HIGH);
  else
    digitalWrite(LEDOUTPUT2, LOW);
  if (BytesInRXFIFO > 0)
    digitalWrite(LEDOUTPUT3, HIGH);
  else
    digitalWrite(LEDOUTPUT3, LOW);
  if (PKTSTATUS & (1 << 2))
    digitalWrite(LEDOUTPUT4, HIGH);
  else
    digitalWrite(LEDOUTPUT4, LOW);
}


void ReadLQI()
{
  byte lqi = 0;
  byte value = 0;
  lqi = (cc1101.readReg(CC1101_LQI, CC1101_STATUS_REGISTER));
  value = 0x3F - (lqi & 0x3F);
  Serial.print(value);
}

void ReadRSSI()
{
  byte rssi = 0;
  byte value = 0;

  rssi = (cc1101.readReg(CC1101_RSSI, CC1101_STATUS_REGISTER));

  if (rssi >= 128)
  {
    value = 255 - rssi;
    value /= 2;
    value += 74;
  }
  else
  {
    value = rssi / 2;
    value += 74;
  }
  //Serial.print("CC1101_RSSI ");
  Serial.print(value);
}

void printHex(int num, int precision) {
  char tmp[16];
  char format[128];

  sprintf(format, "%%.%dX", precision);

  sprintf(tmp, format, num);
  Serial.print(tmp);
}

/*
 * print help - prints out help information
 */

void printHelp() {
   Serial.println(F("Help:"));
   Serial.println(F("d : dump Sensor Data Array"));
   Serial.println(F("s : Display CC1101 Packet Status"));
   Serial.println(F("m : Display CC1101 Marcstate"));
   Serial.println(F("c : toggle skipping of failed CRC packets"));
   Serial.println(F("a : cleanup sensor data array"));
   Serial.println(F("x : toggle auto cleanup of sensor data array"));
   Serial.println();
}

/*
 * toggle skipping of failed crc check received packets
 * 
 */

void toggleCRCCheck() {
  SKIP_FAILED_CRC = !SKIP_FAILED_CRC;
  Serial.println(F("Skip failed CRC Packets: "));
  Serial.println(SKIP_FAILED_CRC);
}

/*
 * toggle automatic cleanup of sensor data array
 * 
 */

void toggleAutoCleanup() {
  AUTO_CLEAN_DB = !AUTO_CLEAN_DB;
  Serial.println(F("Auto clean Sensor Data Array: "));
  Serial.println(AUTO_CLEAN_DB);
}


void loop()
{
  if (packetAvailable) {

    // Disable wireless reception interrupt
    detachInterrupt(digitalPinToInterrupt(GDO0));

    // clear the flag
    packetAvailable = false;

    // increase packet counter
    pcount++;
    Serial.print (uptime());
    Serial.print(F(" RX pcount "));
    Serial.println(pcount);
    // receive the packet from RX Fifo
    if (cc1101.receiveData(&packet) > 0) {

      if ((!SKIP_FAILED_CRC && !packet.crc_ok) || packet.crc_ok )
      {
        Serial.print(F("RSSI: "));
        Serial.print(packet.rssi);
        //ReadRSSI();
        Serial.print(F(" LQI: "));
        //ReadLQI();
        Serial.print(packet.lqi);
        Serial.print(" ");
        if (packet.length > 0)
        {
          Serial.print(F("CRC:"));
          Serial.print(packet.crc_ok);
          Serial.print(F(" packet: len "));
          Serial.print(packet.length);
          Serial.print(F(": "));
          Serial.print(F(" data: "));
          for (int j = 0; j < packet.length; j++)
          {
            printHex(packet.data[j], 2);
            Serial.print(" ");
          }
          Serial.println("");


          if (packet.length == 5)
          {
            sensorData_type sData;
            decodePacket(&packet, &sData);
            if (sData.sensorID != 0)
              //valid Data received
              addOrUpdateSensorData(sData);
          }
        }

      }
    }
    Serial.println("");

    //blink once to signal packet reception
    blinker(LEDOUTPUT);

    //set cc1101 back to RX mode
    cc1101.cmdStrobe(CC1101_SRX);

    //re-attach interrupt
    attachInterrupt(digitalPinToInterrupt(GDO0), cc1101signalsInterrupt, FALLING);



  }

  //check serial input
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case 'd': dumpSensorDataArray(); break;
      case 'h': printHelp(); break;
      case 's': Serial.println(printPKTSTATUS(cc1101.readReg(CC1101_PKTSTATUS, CC1101_STATUS_REGISTER))); break;
      case 'm': Serial.println(printMARCSTATE(cc1101.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER))); break;
      case 'c': toggleCRCCheck(); break;
      case 'a': cleanupSensorDataArray(); break;
      case 'x': toggleAutoCleanup(); break;
   }
   // auto clean Sensor Data Array if enabled
   if (AUTO_CLEAN_DB && millis() - lastCleanTime > AUTOCLEANINTERVAL)
    cleanupSensorDataArray();
    
   
  }

  //updateStatusLED((cc1101.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f), (cc1101.readReg(CC1101_PKTSTATUS, CC1101_STATUS_REGISTER) & 0x1f), (cc1101.readReg(CC1101_RXBYTES, CC1101_STATUS_REGISTER) & 0x1f));
}
