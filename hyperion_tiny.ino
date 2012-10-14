/*
Arduino code used on Hyperion-tiny flight computer 
Payload consisted of:
 * Arduino Pro Mini 3.3v
 * RFM22b
 * Ublox Max6 GPS (http://ava.upuaut.net/store/)
 * 2x DS18B20 temp sensors

Code by James Coxon (jacoxon@googlemail.com) based on previous code as well
as Arduino examples

Minor modifications by Costyn van Dongen
*/
#include <TinyGPS.h>
#include <OneWire.h>
#include <stdio.h>
#include <util/crc16.h>
#include <SPI.h>
#include <RFM22.h>
#include <SoftwareSerial.h>
 
#define ONE_WIRE_BUS 9

TinyGPS gps;
OneWire ds(ONE_WIRE_BUS); // DS18x20 Temperature chip i/o One-wire

//Tempsensor variables
byte address0[8] = {0x28, 0xD5, 0x34, 0x9A, 0x03, 0x00, 0x00, 0xB4};
byte address1[8] = {0x28, 0x4F, 0x1F, 0x9A, 0x03, 0x00, 0x00, 0x42};

int temp0 = 0, temp1 = 0 ;

int count = 1, nightloop = 0;
byte navmode = 99;
float flat, flon;
unsigned long date, time, chars, age;

int hour = 0 , minute = 0 , second = 0, oldsecond = 0;
char latbuf[12] = "0", lonbuf[12] = "0", altbuf[12] = "0";
long int ialt = 123;
int numbersats = 99;

// Voltage divider stuff:
const int voltPin = 3;
float denominator;
int resistor1 = 9790;
int resistor2 = 2157;
float vccVoltage = 3.4;
char voltbuf[4] = "0";

// Software serial for debugging. Connect FTDI 3.3v RX pin to pin 5 to get debugging so GPS can get a clean hardware serial.
SoftwareSerial mySerial(4, 5);
 
//Setup radio on SPI with NSEL on pin 10
rfm22 radio1(10);
 
void setupRadio(){
  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71, 0x00); // unmodulated carrier
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up. 0b and 0c are swapped because GPIO0 and GPIO1 are swapped connected to TX_ANT and RX_ANT
  radio1.write(0x0b,0x15);
  radio1.write(0x0c,0x12);
  radio1.setFrequency(434.650);  // frequency
  radio1.write(0x07, 0x08); // turn tx on
  radio1.write(0x6D, 0x04);// turn tx low power 14db = 25mW
 
}

// RTTY Functions - from RJHARRISON's AVR Code
void rtty_txstring (char * string)
{
 
	/* Simple function to sent a char at a time to 
	** rtty_txbyte function. 
	** NB Each char is one byte (8 Bits)
	*/
	char c;
	c = *string++;
	while ( c != '\0')
	{
		rtty_txbyte (c);
		c = *string++;
	}
}
 
void rtty_txbyte (char c)
{
	/* Simple function to sent each bit of a char to 
	** rtty_txbit function. 
	** NB The bits are sent Least Significant Bit first
	**
	** All chars should be preceded with a 0 and 
	** proceded with a 1. 0 = Start bit; 1 = Stop bit
	**
	** ASCII_BIT = 7 or 8 for ASCII-7 / ASCII-8
	*/
	int i;
	rtty_txbit (0); // Start bit
	// Send bits for for char LSB first	
	for (i=0;i<8;i++)
	{
		if (c & 1) rtty_txbit(1); 
			else rtty_txbit(0);	
		c = c >> 1;
	}
	rtty_txbit (1); // Stop bit
        rtty_txbit (1); // Stop bit
}
 
void rtty_txbit (int bit)
{
		if (bit)
		{
		  // high; 0x073 is least significant bit of frequency register, 2x156Hz; 156Hz = smallest frequency adjustment possible on the RFM22b
                  radio1.write(0x073, 0x03);
		}
		else
		{
		  // low
                  radio1.write(0x073, 0x00);
		}
                delayMicroseconds(19500); // 10000 = 100 BAUD ; 20150 = 50
 
}

uint16_t gps_CRC16_checksum (char *string)
{
	size_t i;
	uint16_t crc;
	uint8_t c;
 
	crc = 0xFFFF;
 
	// Calculate checksum ignoring the first two $s
	for (i = 2; i < strlen(string); i++)
	{
		c = string[i];
		crc = _crc_xmodem_update (crc, c);
	}
 
	return crc;
}
  
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    Serial.write(MSG[i]);
  }
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
	uint8_t b;
	uint8_t ackByteID = 0;
	uint8_t ackPacket[10];
        Serial.flush();
	unsigned long startTime = millis();
 
	// Construct the expected ACK packet    
	ackPacket[0] = 0xB5;	// header
	ackPacket[1] = 0x62;	// header
	ackPacket[2] = 0x05;	// class
	ackPacket[3] = 0x01;	// id
	ackPacket[4] = 0x02;	// length
	ackPacket[5] = 0x00;
	ackPacket[6] = MSG[2];	// ACK class
	ackPacket[7] = MSG[3];	// ACK id
	ackPacket[8] = 0;		// CK_A
	ackPacket[9] = 0;		// CK_B
 
	// Calculate the checksums
	for (uint8_t i=2; i<8; i++) {
		ackPacket[8] = ackPacket[8] + ackPacket[i];
		ackPacket[9] = ackPacket[9] + ackPacket[8];
	}
 
	while (1) {
 
		// Test for success
		if (ackByteID > 9) {
				// All packets in order!
                                navmode = 1;
				return true;
		}
 
		// Timeout if no valid response in 3 seconds
		if (millis() - startTime > 3000) { 
                        navmode = 0;
			return false;
		}
 
		// Make sure data is available to read
		if (Serial.available()) {
			b = Serial.read();

			// Check that bytes arrive in sequence as per expected ACK packet
			if (b == ackPacket[ackByteID]) { 
				ackByteID++;
                                //Serial.print(ackPacket[ackByteID], HEX);
                                //Serial.print(" ");
			} else {
				ackByteID = 0;	// Reset and look again, invalid order
			}
 
		}
	}
}

//Function to poll the NAV5 status of a Ublox GPS module (5/6)
//Sends a UBX command (requires the function sendUBX()) and waits 3 seconds
// for a reply from the module. It then isolates the byte which contains 
// the information regarding the NAV5 mode,
// 0 = Pedestrian mode (default, will not work above 12km)
// 6 = Airborne 1G (works up to 50km altitude)
//Adapted by jcoxon from getUBX_ACK() from the example code on UKHAS wiki
// http://wiki.ukhas.org.uk/guides:falcom_fsa03
boolean checkNAV(){
  uint8_t b, bytePos = 0;
  uint8_t getNAV5[] = { 0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84 }; //Poll NAV5 status
  
  Serial.flush();
  unsigned long startTime = millis();
  sendUBX(getNAV5, sizeof(getNAV5)/sizeof(uint8_t));
  
  while (1) {
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();
      
      if(bytePos == 8){
        navmode = b;
        return true;
      }
                        
      bytePos++;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      navmode = 0;
      return false;
    }
  }
}

void setupGPS() {
  //Turning off all GPS NMEA strings apart on the uBlox module
  Serial.println("$PUBX,40,GLL,0,0,0,0*5C");
  Serial.println("$PUBX,40,GGA,0,0,0,0*5A");
  Serial.println("$PUBX,40,GSA,0,0,0,0*4E");
  Serial.println("$PUBX,40,RMC,0,0,0,0*47");
  Serial.println("$PUBX,40,GSV,0,0,0,0*59");
  Serial.println("$PUBX,40,VTG,0,0,0,0*5E");
  
  delay(3000); // Wait for the GPS to process all the previous commands
  
 // Check and set the navigation mode (Airborne, 1G)
  uint8_t setNav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC};
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  
  getUBX_ACK(setNav);
  
}

// gets temperature data from onewire sensor network, need to supply byte address, it'll check to see what type of sensor and convert appropriately
int getTempdata(byte sensorAddress[8]) {
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole;
  byte data[12], i, present = 0;
  
  ds.reset();
  ds.select(sensorAddress);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(sensorAddress);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }
 LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  
  if (sensorAddress[0] == 0x10) {
    Tc_100 = TReading * 50;    // multiply by (100 * 0.0625) or 6.25
  }
  else { 
    Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25
  }
  
  
  Whole = Tc_100 / 100;  // separate off the whole and fractional portions

  if (SignBit) // If its negative
  {
     Whole = Whole * -1;
  }
  return Whole;
}

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  
  mySerial.println("Waiting for GPS to boot");
 
  delay(5000); // We have to wait for a bit for the GPS to boot otherwise the commands get missed
  
  setupGPS();
  mySerial.println("Setup GPS...");
  
  setupRadio() ;
  mySerial.println("Setup Radio...");

  denominator = (float)resistor2 / (resistor1 + resistor2);
}

void loop() { 
    char superbuffer [120];
    char checksum [10];
    int n;
    
    if((count % 10) == 0) {
     if(navmode != 6){
       setupGPS();
       delay(1000);
     }
   }
   
    Serial.println("$PUBX,00*33"); //Poll GPS
    
    while (Serial.available())
    {
      int c = Serial.read();
      if (gps.encode(c))
      {
        //Get Data from GPS library
        //Get Time and split it
        gps.get_datetime(&date, &time, &age);
        hour = (time / 1000000);
        minute = ((time - (hour * 1000000)) / 10000);
        second = ((time - ((hour * 1000000) + (minute * 10000))));
        second = second / 100;
      
      //Get Position
      gps.f_get_position(&flat, &flon);
  
      //convert float to string
      dtostrf(flat, 7, 4, latbuf);
      dtostrf(flon, 7, 4, lonbuf);
      
      //just check that we are putting a space at the front of lonbuf
      if(lonbuf[0] == ' ')
      {
        lonbuf[0] = '+';
      }
      
      // +/- altitude in 
      ialt = (gps.altitude() / 100);   
      itoa(ialt, altbuf, 10);
    }
    }
    
    temp0 = getTempdata(address0);
    temp1 = getTempdata(address1);

    numbersats = gps.satellites();

    float voltage;
    voltage = analogRead(voltPin);
    mySerial.println( voltage ) ;
    voltage = (voltage / 1024) * vccVoltage;
    voltage = voltage / denominator;
    dtostrf(voltage,4,2,voltbuf); // convert to string
    
    n=sprintf (superbuffer, "$$HYPERION,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%d,%d,%d,%s", count, hour, minute, second, latbuf, lonbuf, ialt, numbersats, navmode, temp0, temp1, voltbuf );
    if (n > -1){
      n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
//      radio1.write(0x07, 0x08); // turn tx on
      rtty_txstring(superbuffer);
      mySerial.println(superbuffer); 
//      radio1.write(0x07, 0x01); // turn tx off
    }
    count++;

}

