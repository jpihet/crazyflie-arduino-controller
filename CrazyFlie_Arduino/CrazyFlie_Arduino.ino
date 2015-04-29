/*   CrazyFlie Controller
 *   version 0.29  7/05/13 tbitson
 *
 * This program demonstarts controlling a CrazyFlie Quadcopter
 * using an Arduino. Based on the RF24 library by maniacbug at
 * https://github.com/maniacbug
 *
 * this is a very early prototype just to show its possible
 *
 *     TODO: 
 *     Get ACK packet to work
 *     Use 4 pushbuttons to adjust trim
 *     Add Logging & display remote battery capacity on LCD
 *     Fix thrust smoothing
 *     Convert LCD routines to a Arduinbo Libray
 *     Eliminate the RF24 library with direct nRF24L01 comm
 */


#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
//#include "printf.h"
#include <SoftwareSerial.h>

// Uncomment this for serial LCD support
//#define LCD

// my defines
#define ulong uint32_t
#define uint  uint16_t

// enable debug prints by setting to 1
const boolean DEBUG = 0;

// version
#define VERSION 0.29

// connections
#define JS1_V   A0    // Thrust
#define JS1_H   A1    // Yaw
#define JS1_S   A2    // Joystick 1 button

#define JS2_V   A3    // Pitch
#define JS2_H   A4    // Roll
#define JS2_S   A5    // Joystick 2 button

#ifdef LCD
#define LCD_RX  2    // Rx
#define LCD_TX  3    // not connected
#endif

#define RF24_MOSI  11
#define RF24_MISO  12
#define RF24_SCK   13
#define RF24_IRQ   8   // not connected
#define RF24_CE    9
#define RF24_CSN   10

#define SW1    4    // switches unused in this version
#define SW2    5
#define SW3    6
#define SW4    7

#define LED  8

// from crazyflie firmware commander.c
#define MIN_THRUST  10000
#define MAX_THRUST  50000
#define MIN_PITCH -30.0
#define MAX_PITCH  30.0
#define MIN_ROLL  -30.0
#define MAX_ROLL   30.0
#define MIN_YAW   -200.0
#define MAX_YAW    200.0

// hard coded offset values TODO: use switches to adjust
const int JS1_V_OFFSET = 497;    // thrust
const int JS1_H_OFFSET = 508;    // yaw
const int JS2_V_OFFSET = 510;    // pitch
const int JS2_H_OFFSET = 514;    // roll

// program defines
#define LOOP_TIME    100  // 100ms = 10 updates per second
#define SLEEP_MODE   0
#define PWR_ON_MODE  1

// joystick full scale value. Range 0 +/- 512 counts
const float JS_RANGE = 512.0;

// user set trim values TODO: store in EEPROM
int pitchOffset = 0;
int rollOffset = 0;
int yawOffset = 0;

// experimental thrust smoothing levels
const float ALPHA_UP = .50;
const float ALPHA_DOWN = .90;
float smoothedThrust = 0.0;


// Init RF24 lib. Pass CE and CSN pins
RF24 radio(RF24_CE, RF24_CSN);

#ifdef LCD
// init LCD Serial port. Pass Rx and Tx pins
SoftwareSerial lcd(2, 3);
#endif

// define the packet we want to send
typedef struct
{
  byte addr;
  float roll;
  float pitch;
  float yaw;
  uint  thrust;
} cmdPacket;

// create an instance of the packet
cmdPacket crtp; 

// TODO: create a union with cmpPacket and byte array 
// send & receive array. Lame way to send struct
char payload[15];

// define a struct to store current state of I/O
typedef struct
{
  int thrust;
  int pitch;
  int yaw;
  int roll;
  boolean sw1;
  boolean sw2;
}  controls;

//make an instance of the controls struct
controls cntr;

typedef struct
{
  byte version;
  unsigned int js1_v;
  unsigned int js1_h;
  unsigned int js2_v;
  unsigned int js2_h;
} eepromValues;

eepromValues cal;

// program vars
ulong lastLoopTime = 0;
byte  mode = PWR_ON_MODE;





void setup(void)
{
  // let everything powerup & init
  delay(2000);

  Serial.begin(57600);
  Serial.print("CrazyFlie Arduino Controller ver ");
  Serial.println(VERSION);
  //printf_begin();

#ifdef LCD
  lcd.begin(57600);
  initSerialLCD();
  setLCDColor(0xFF, 0xFF, 0xFF);  // white
  lcd.print("CrazyFlie v");
  lcd.print(VERSION);
#endif

  // Init nRRF24L01
  radio.begin();

  // set up pin modes
  pinMode(LED, OUTPUT);
  pinMode(JS1_S, INPUT);
  pinMode(JS2_S, INPUT);

  // enable internal pullups on joystick switches
  digitalWrite(JS1_S, HIGH);
  digitalWrite(JS2_S, HIGH);

  // TODO: read control offsets if JS button is pushed
  // and store in EEPROM
  //readControls();
  //if (!cntr.sw2)
  //  doCalibrate();

  // Calibrate at every startup
  doCalibrate();

  // define initial values for packet
  crtp.addr   = 0x30;
  crtp.roll   = 0.0;
  crtp.pitch  = 0.0;
  crtp.yaw    = 0.0;
  crtp.thrust = 0;

  // enable dynamic payloads, ch 10, data rate 250K
  radio.enableDynamicPayloads();
  radio.setChannel(10);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(5,3);
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(0xE7E7E7E7E7LL);
  radio.openReadingPipe(1, 0xE7E7E7E7E7LL);
  delay(100);

  // Dump the configuration of the rf unit for debugging
  if (DEBUG) radio.printDetails();

  // Start listening
  radio.startListening();

  Serial.println("setup done");
#ifdef LCD
  setLCDCursor(1, 2);
  lcd.print("Ready");
#endif
}




void loop(void)
{
  // loop if desired time has passed
  if ((millis() - lastLoopTime) > LOOP_TIME)
  {
    lastLoopTime = millis();

    // read controls
    readControls();

    // check if we should enter/exit low power mode
    //checkPowerStatus();

    // check if calibration is needed
    checkCalibrationNeeded();

    // if low power mode, just exit
    if (mode == SLEEP_MODE) return;

    // scale controls to expected CF values
    processControls();

    if (DEBUG) printCntrValues();
    if (DEBUG) printCRTPValues();

    // send data to Cazyflie
    sendData();

    // check various stuff
    checkStatus();

#ifdef LCD
    // show current status and info on LCD
    updateLCD();
#endif
  }
}


void readControls()
{
  // read pitch and roll on joystick 1
  cntr.thrust = analogRead(JS1_V);
  cntr.yaw    = analogRead(JS1_H);
  cntr.sw1    = !digitalRead(JS1_S);

  // read throttle & yaw on joystick 2
  cntr.pitch = analogRead(JS2_V);
  cntr.roll  = analogRead(JS2_H);
  cntr.sw2   = !digitalRead(JS2_S);

  // TODO: add debouce to switches if pressed
}


void processControls()
{
  // thrust is 10000 to 50000
  // pitch is -30.00 to +30.00
  // roll is -30.00 to +30.00
  // yaw is -199.99 to +199.99

  float alpha;

  // first subtract offsets from joystick calibration
  cntr.thrust -= cal.js1_v;     //JS1_V_OFFSET;
  cntr.yaw    -= cal.js1_h;     //JS1_H_OFFSET;
  cntr.pitch  -= cal.js2_v;     //JS2_V_OFFSET;
  cntr.roll   -= cal.js2_h;     //JS2_H_OFFSET;

  // scale to crazyflie values
  float thrust = ((float)cntr.thrust - 10.0) * MAX_THRUST/512.0;
  crtp.pitch = (float)cntr.pitch * MAX_PITCH/512.0 * -1.0;
  crtp.roll =  (float)cntr.roll * MAX_ROLL/512.0 * -1.0;
  crtp.yaw =   (float)cntr.yaw * MAX_YAW/512.0;

  // experimental filter to smooth thrust power
  if (thrust < 0) thrust = 0;
  if (thrust < smoothedThrust)
    alpha = ALPHA_DOWN;
  else
    alpha = ALPHA_UP;

  // running average filter
  smoothedThrust = alpha * smoothedThrust + (1.0 - alpha) * thrust;
  crtp.thrust = (uint)smoothedThrust;

  if (DEBUG)
  {
    Serial.print("thrust = ");
    Serial.print(thrust);
    Serial.print("   smoothed thrust = ");
    Serial.println(smoothedThrust);
  }

  // check if in bounds
  if (crtp.thrust < MIN_THRUST) crtp.thrust = 0;
  if (crtp.thrust > MAX_THRUST) crtp.thrust = MAX_THRUST;

  constrain(crtp.pitch, MIN_PITCH, MAX_PITCH);
  constrain(crtp.roll, MIN_ROLL, MAX_ROLL);
  constrain(crtp.yaw, MIN_YAW, MAX_YAW);
}




void checkStatus(void)
{
  // TODO: check battery voltage
  // TODO: check link status
}


void doCalibrate()
{
#define AVG_LEN     250

  ulong sum1_v = 0, sum1_h = 0, sum2_v = 0, sum2_h = 0;

  Serial.println("Calibrating ...");
#ifdef LCD
  // update LCD
  setLCDCursor(1, 2);
  lcd.print("Calibrating...");
#endif

  // Let some time to release the josyticks
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);

  for(int i = 0; i < AVG_LEN; i++) {
    sum1_v += analogRead(JS1_V);
    sum1_h += analogRead(JS1_H);
    sum2_v += analogRead(JS2_V);
    sum2_h += analogRead(JS2_H);
  }

  cal.js1_v = sum1_v / AVG_LEN;
  cal.js1_h = sum1_h / AVG_LEN;
  cal.js2_v = sum2_v / AVG_LEN;
  cal.js2_h = sum2_h / AVG_LEN;

  //if (DEBUG)
  {
    Serial.print("JS1_V Offset = ");
    Serial.println(cal.js1_v);
    Serial.print("JS1_H Offset = ");
    Serial.println(cal.js1_h);
    Serial.print("JSs_V Offset = ");
    Serial.println(cal.js2_v);
    Serial.print("JS2_H Offset = ");
    Serial.println(cal.js2_h);
  }

  // store in EEPROM

  digitalWrite(LED, LOW);
}

#ifdef LCD
void updateLCD()
{
  setLCDCursor(1, 2);

  lcd.print("T:");
  lcd.print(cntr.thrust);
  lcd.print(" P:");
  lcd.print(cntr.pitch);

  lcd.print(" R:");
  lcd.print(cntr.roll);
  lcd.print(" ");
}
#endif

void sendData(void)
{
  // turn on LED to show we're sending
  digitalWrite(LED, HIGH);

  // First, stop listening so we can talk.
  radio.stopListening();

  // copy the struct into a byte array
  memcpy(&payload, &crtp, sizeof(crtp));

  // send the packet. Blocks until sent
  if (DEBUG) Serial.println("Sending...");
  radio.write(payload, sizeof(crtp) );

  // turn off LED
  digitalWrite(LED, LOW);

  // start listening for an ACK
  radio.startListening();

  // Wait here until we get a response, or timeout
  ulong start = millis();
  boolean timeout = false;
  while ( !radio.available() && !timeout )
  {
    if (millis() - start > 10)
      timeout = true;
  }

  // Did we timeout?
  if (timeout)
  {
    if (DEBUG) Serial.println("response timed out");
  }
  else
  {
    // read response
    uint8_t len = radio.getDynamicPayloadSize();
    radio.read(payload, len );

    // display response
    Serial.print("Got response: size= ");
    Serial.println(len);
  }

  // delay for debugging
  //delay(1000);
}



void printCntrValues()
{
  Serial.print("Thrust = ");
  Serial.print(cntr.thrust);
  Serial.print("  Yaw = ");
  Serial.print(cntr.yaw);
  Serial.print("  Pitch = ");
  Serial.print(cntr.pitch);
  Serial.print("  Roll = ");
  Serial.print(cntr.roll);
  Serial.print("  SW1 = ");
  Serial.print(cntr.sw1);
  Serial.print("  SW2 = ");
  Serial.println(cntr.sw2);
}


void printCRTPValues()
{
  Serial.print("ADDR = ");
  Serial.print(crtp.addr, HEX);
  Serial.print("  Roll = ");
  Serial.print(crtp.roll);
  Serial.print("  Pitch = ");
  Serial.print(crtp.pitch);
  Serial.print("  Yaw = ");
  Serial.print(crtp.yaw);
  Serial.print("  Thrust = ");
  Serial.println(crtp.thrust);
}

void checkCalibrationNeeded(void)
{ byte i;

  // check if both joystick switches are on
  if (cntr.sw1 && cntr.sw2) {
    // Keep checking for 2 seconds
    for (i = 0; i < 20; i++) {
        delay(100);
        readControls();
        // If button released, exit
        if (!cntr.sw1 || !cntr.sw2)
            return;
    }

    // if the buttons are still down, calibrate
    if (cntr.sw1 && cntr.sw2)
        doCalibrate();
  }
}

void checkPowerStatus()
{
  // check if both joystick switches are on
  if (cntr.sw1 && cntr.sw2)
  {
    // wait 2 seconds and check again
    delay(2000);
    readControls();

    // if the buttons are still down, switch modes
    if (cntr.sw1 && cntr.sw2)
    { 
      if (mode == PWR_ON_MODE)
      {
#ifdef LCD
        // power down
        setLCDDisplayOn(0);
#endif
        mode = SLEEP_MODE;
        // TODO: Put nRF24L01 & arduino to low power mode
      }
      else
      {
        // power up
#ifdef LCD
        setLCDDisplayOn(1);
#endif
        mode = PWR_ON_MODE;
      }
    }
  }
}



#ifdef LCD

// LCD Routines - TODO: convert this to a lib ----------

void initSerialLCD()
{
  setLCDSize(16, 2);
  setLCDContrast(200);      
  setLCDBrightness(255);     
  setLCDDisplayOn(1);

  // turn off cursors
  lcd.write(0xFE);
  lcd.write(0x4B);

  // block curson off
  lcd.write(0xFE);
  lcd.write(0x54);

  // turn off autoscroll
  lcd.write(0xFE);
  lcd.write(0x52);

  clearLCD();
}


void clearLCD()
{
  // clear screen
  lcd.write(0xFE);
  lcd.write(0x58);
  delay(1);
}


void setLCDSize(byte cols, byte rows)
{
  lcd.write(0xFE);
  lcd.write(0xD1);
  lcd.write(cols);  // default = 16 columns
  lcd.write(rows);  // default = 2 rows
  delay(10);
}


void setLCDDisplayOn(byte val)
{
  lcd.write(0xFE);

  if (val > 0)
  {
    lcd.write(0x42);
    lcd.write(0xFF);
  }
  else
    lcd.write(0x46);

}


void setLCDBrightness(byte val)
{
  lcd.write(0xFE);
  lcd.write(0x99);
  lcd.write(val);
}

void setLCDContrast(byte val)
{
  lcd.write(0xFE);
  lcd.write(0x50);
  lcd.write(val);
}

void setLCDBaud(byte val)
{
  // 0x10 = 57600
  // 0x22 = 28800
  // 0x33 = 19200
  // 0x67 = 9600 (default)

  lcd.write(0xFE);
  lcd.write(0x39);
  lcd.write(val);
  delay(10);
}


void homeLCD()
{
  // go 'home'
  lcd.write(0xFE);
  lcd.write(0x48);
  delay(1);  
}

void setLCDCursor(byte col, byte row)
{
  // set curson to col & row position
  lcd.write(0xFE);
  lcd.write(0x47);
  lcd.write(col);
  lcd.write(row);
}


void setLCDColor(byte r, byte g, byte b)
{
  lcd.write(0xFE);
  lcd.write(0xD0);
  lcd.write(r); 
  lcd.write(g);
  lcd.write(b);
  delay(1);
}

#endif
