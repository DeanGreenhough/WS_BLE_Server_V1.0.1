/*
Author: Dean Greenhough 2019
*/

//version control
//v1.0.0
//V1.0.1  added ina219, tx voltage, adjusted arrays

#include "Wire.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_INA219.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define   DEBUG 1
#define   SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define   CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define   MUX_Address 0x70                    // TCA9548A ADDRESS
#define   DONEPIN        25                   // TPL5110 SWITCH OFF PIN

Adafruit_VL53L0X left  = Adafruit_VL53L0X();
Adafruit_VL53L0X right = Adafruit_VL53L0X();  //ADDED A SECOND INSTANCE
Adafruit_INA219 ina219;                       //INA219

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
byte TX_ARRAY [8];                            //TX DATA ARRAY BLE
uint16_t  TX_LEFT             = 0;
uint16_t  TX_RIGHT            = 0;
unsigned int Distance_LEFT    = 0;
unsigned int Distance_RIGHT   = 0;


//INA219
uint16_t  TX_VOLTAGE          = 0;
uint16_t  TX_CURRENT          = 0;
float busvoltage              = 0;
float current_mA              = 0;
//int shuntvoltage  = 0;
//int loadvoltage   = 0;
//int power_mW      = 0;

//TIMERS
unsigned long loopTime = 0;                //LOOP TIMER
unsigned long startMillis;                 //LOOP TIMER
unsigned long currentMillis;               //LOOP TIMER



class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

// Initialize I2C buses using TCA9548A I2C Multiplexer
void tcaselect(uint8_t i2c_bus) {
  if (i2c_bus > 7) return;
  Wire.beginTransmission(MUX_Address);
  Wire.write(1 << i2c_bus);
  Wire.endTransmission();
}


void setup() {
  startMillis = millis();
  pinMode(DONEPIN, OUTPUT);                                     //MUST BE FIRST TO KEEP POWER ON
  digitalWrite(DONEPIN, HIGH);
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.println(__DATE__);
  Serial.println(__TIME__);
  ina219.begin();                                               //INIT INA219
  ina219.setCalibration_16V_400mA();                            //SET CALIBRATION RANGE
  BLEDevice::init("WSoft");                                     // CREATE BLE DEVICE ***MAX 5 LETTERS***

  BLEServer *pServer = BLEDevice::createServer();               // CREATE BLE SERVER
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);  // CREATE BLE SERVICE

  pCharacteristic = pService->createCharacteristic(             // CREATE BLE CHARACTERISTIC
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  pCharacteristic->addDescriptor(new BLE2902());                 // CREATE BLE DESCRIPTOR
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);       //important allows connection notfications

  pService->start();                                             // START THE SERVICE
  pServer->getAdvertising()->start();                            // START ADVERTISING
  delay(500);
}


void loop() {

  INA219();                                                 //GET POWER MEASUREMENTS
  MUXleft();                                                //HAS TO BE HERE AS MISSES DATA ON FIRST PASS
  MUXright();
  MUXleft();

  if (deviceConnected) {
    //LEFT
    TX_LEFT = Distance_LEFT;
    if (DEBUG)Serial.print("TX_LEFT     ");
    if (DEBUG)Serial.println(TX_LEFT);
    byte loByte_TX_LEFT;                                    //DECLARE
    byte hiByte_TX_LEFT;                                    //DECLARE
    hiByte_TX_LEFT = highByte(TX_LEFT);                     //CONVERT INT READING TO BYTES FOR TRANSMISSION
    loByte_TX_LEFT = lowByte(TX_LEFT);                      //CONVERT INT READING TO BYTES FOR TRANSMISSION
    TX_ARRAY[0] = hiByte_TX_LEFT;                           //STORE VALUE IN ARRAY
    TX_ARRAY[1] = loByte_TX_LEFT;                           //STORE VALUE IN ARRAY

    //RIGHT
    TX_RIGHT = Distance_RIGHT;
    if (DEBUG)Serial.print("TX_RIGHT    ");
    if (DEBUG)Serial.println(TX_RIGHT);
    byte loByte_TX_RIGHT;                                    //DECLARE
    byte hiByte_TX_RIGHT;                                    //DECLARE
    hiByte_TX_RIGHT = highByte(TX_RIGHT);                    //CONVERT INT READING TO BYTES FOR TRANSMISSION
    loByte_TX_RIGHT = lowByte(TX_RIGHT);                     //CONVERT INT READING TO BYTES FOR TRANSMISSION
    TX_ARRAY[2] = hiByte_TX_RIGHT;                           //STORE VALUE IN ARRAY
    TX_ARRAY[3] = loByte_TX_RIGHT;                           //STORE VALUE IN ARRAY

    //VOLTAGE
    TX_VOLTAGE = (busvoltage * 1000);
    if (DEBUG)Serial.print("TX_VOLTAGE  ");
    if (DEBUG)Serial.println(TX_VOLTAGE);
    byte loByte_TX_VOLTAGE;                                    //DECLARE
    byte hiByte_TX_VOLTAGE;                                    //DECLARE
    hiByte_TX_VOLTAGE = highByte(TX_VOLTAGE);                  //CONVERT INT READING TO BYTES FOR TRANSMISSION
    loByte_TX_VOLTAGE = lowByte(TX_VOLTAGE);                   //CONVERT INT READING TO BYTES FOR TRANSMISSION
    TX_ARRAY[4] = hiByte_TX_VOLTAGE;                           //STORE VALUE IN ARRAY
    TX_ARRAY[5] = loByte_TX_VOLTAGE;                           //STORE VALUE IN ARRAY

     //CURRENT
    TX_CURRENT = current_mA;
    if (DEBUG)Serial.print("TX_CURRENT  ");
    if (DEBUG)Serial.println(TX_CURRENT);
    byte loByte_TX_CURRENT;                                    //DECLARE
    byte hiByte_TX_CURRENT;                                    //DECLARE
    hiByte_TX_CURRENT = highByte(TX_CURRENT);                  //CONVERT INT READING TO BYTES FOR TRANSMISSION
    loByte_TX_CURRENT = lowByte(TX_CURRENT);                   //CONVERT INT READING TO BYTES FOR TRANSMISSION
    TX_ARRAY[6] = hiByte_TX_CURRENT;                           //STORE VALUE IN ARRAY
    TX_ARRAY[7] = loByte_TX_CURRENT;                           //STORE VALUE IN ARRAY
    

    pCharacteristic->setValue(TX_ARRAY, 8);                  //SET ARRAY AND AMOUNT OF BYTES
    pCharacteristic->notify();                               //NOTIFY SEND TX_ARRAY
    delay(500); //delay added to see if rx issue             //REQUIRED DUE TO INTERMITENT RX DATA AT CLIENT END

    currentMillis = millis();                                //END LOOPTIME
    loopTime = (currentMillis - startMillis);
    if (DEBUG)Serial.print("looptime =  ");
    if (DEBUG)Serial.println(loopTime);
    if (DEBUG)Serial.println(" ");
    digitalWrite(DONEPIN, LOW);                              //SEND DONE TO TPL5110 TO SHUT DOWN
    delay(10);
  }

  //delay(1000);                                             //TESTING DELAY


}

void MUXleft() {

  tcaselect(0);
  delay(150);               //Could be some timing issues to work thru here
  if (!left.begin())
  {
    Serial.println(F("LEFT  SENSOR FAILED"));
  }

  VL53L0X_RangingMeasurementData_t measure;
  left.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {
    Distance_LEFT    = (measure.RangeMilliMeter);

  } else {
    Serial.println(" LEFT out of range ");
  }


}


void MUXright() {
  tcaselect(1);
  delay(150);
  if (!right.begin())
  {
    Serial.println(F("RIGHT SENSOR FAILED"));
  }

  VL53L0X_RangingMeasurementData_t measure;
  right.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {
    Distance_RIGHT  = (measure.RangeMilliMeter);

  } else {
    Serial.println(" RIGHT  out of range ");
  }


}
