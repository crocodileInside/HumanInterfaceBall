#include <SPI.h>
#include <BLEHIDPeripheral.h>
#include <BLEMouse.h>
#include <BLEKeyboard.h>
#include <BLEMultimedia.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <utility/imumaths.h>

#define MouseBtnLeft   0x01
#define MouseBtnRight  0x02
#define MouseBtnMiddle 0x04

//custom boards may override default pin definitions with BLEHIDPeripheral(PIN_REQ, PIN_RDY, PIN_RST)
BLEHIDPeripheral bleHIDPeripheral = BLEHIDPeripheral();
BLEMouse bleMouse;
BLEKeyboard bleKeyboard;



int buttonState;
int joystickXCenter;
int joystickYCenter;
int Hold_Var=0;
float basePressure = 954.0f;
float accX=0, accY=0,accZ=0;
int pos_acc=0;



Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_BME280 bme;



void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

#define LED 0



void doCalibration(){

  int numSamples = 50;
  float pAvg = 0.0f;

  for(int i=0; i<numSamples; i++){
    pAvg += bme.readPressure() / 100.0f;
    
    delay(10);
  }

  pAvg = pAvg / (float)numSamples;
  basePressure = pAvg;
}

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);
  /*if (buttonState == LOW) {
    Serial.println(F("BLE HID Peripheral - clearing bond data"));

    // clear bond store 
    bleHIDPeripheral.clearBondStoreData();

  //}*/

  bleHIDPeripheral.clearBondStoreData();
  bleHIDPeripheral.setReportIdOffset(1);

  bleHIDPeripheral.setLocalName("HIB1");
  bleHIDPeripheral.addHID(bleMouse);
  bleHIDPeripheral.addHID(bleKeyboard);

  bleHIDPeripheral.begin();

  Serial.println(F("BLE Human Interface Ball"));

  joystickXCenter = 0;
  joystickYCenter = 0;

  Serial.println("starting BNO");
  if(!bno.begin()){
    Serial.println("no BNO055 detected!");
  }
  Serial.println("BNO setCrystal");
  bno.setExtCrystalUse(false);

  if(!bme.begin()){
    Serial.println("no BME280 detected!");
  }

  doCalibration();
  
  Serial.println("Setup complete");
  
}

int i = 0;
int ledstate = 0;


#define ROLL_AVG_SIZE   32
float gyroAvgArrX[ROLL_AVG_SIZE];
float gyroAvgArrY[ROLL_AVG_SIZE];
float gyroAvgArrZ[ROLL_AVG_SIZE];

float rollAvg(float arr[], float newValue){
  memmove(&(arr[1]), arr, (ROLL_AVG_SIZE-1)*sizeof(float));
  arr[0] = newValue;

  float avg = 0.0f;
  for(int i=0; i<ROLL_AVG_SIZE; i++){
    avg += arr[i];
  }

  return (avg / (float)ROLL_AVG_SIZE);
}


float oriX, oriY, oriZ;
float dOriX, dOriY, dOriZ;
float oriXold = 0.0f, oriYold = 0.0f, oriZold = 0.0f;

float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
float gyroXold = 0.0f, gyroYold = 0.0f, gyroZold = 0.0f;
float dX = 0.0f, dY = 0.0f, dZ = 0.0f;

float sx = 0.0f, sy=0.0f, sz=0.0f;
float avgXold = 0.0f, avgYold = 0.0f, avgZold = 0.0f;


int moveLockTimeout = 0;

void loop() {
  Serial.println("BLE getCentral");
  BLECentral central = bleHIDPeripheral.central();

  //displaySensorStatus();

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  ledstate = !ledstate;
  digitalWrite(LED, ledstate);
  delay(200);
  
  
  if (central) {
    // central connected to peripheral
    Serial.print(F("Connected to central: "));
    Serial.println(central.address());
  










    while (central.connected()) {

      float pressure = bme.readPressure()/100.0f;
      float dP = pressure - basePressure;
      int tempButtonState = 0;



      //Middle Button Click for scroll function
      imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      accX= acc.x();
      accY= acc.y();
      accZ= acc.z();

      if((accY >=10.0f)||(accZ >=10.0f)){
        pos_acc=1;
      }
      if(pos_acc == 1 && (accY<=-10.0f||accZ<=-10.0f)){
        tempButtonState=4;
        }     
      
      //Serial.println(dP);
      if(dP>=20){
        tempButtonState=3;        
      }else if(dP > 1.0f && dP < 20.0f){
          tempButtonState = 1;      // Left Button click
          if(dP < 10.0f){
            Hold_Var=0;
            moveLockTimeout = 50;   // mouse freeze
          }else{
            Hold_Var = 1; 
          }
      }else if(dP < 1.0f && pos_acc != 1){
        tempButtonState = 2;
      }
      
      if ((tempButtonState != buttonState)) {
        buttonState = tempButtonState;
        switch(buttonState)
        {
          case 1: { // Press Mouse Btn Left
            //Serial.println(F("Mouse press"));   // Left press Button
            bleMouse.press(MouseBtnLeft);
            bleMouse.release(MouseBtnMiddle);
            pos_acc=0;
            break;
          }                  
          case 2: { // Release Mouse Btn Left
            //Serial.println(F("Mouse release")); // Left release Button
            if(Hold_Var != 1){
            bleMouse.release(MouseBtnLeft);
            }
            break;
          }
          case 3: { // Click Mouse Btn Right    // Right Button click
            moveLockTimeout = 50;
            bleMouse.click(MouseBtnRight);
            moveLockTimeout = 50;
            break;
          }
          case 4: { // Click Mouse Btn Right    // Middle Button press
            moveLockTimeout = 100;
            bleMouse.press(MouseBtnMiddle);
            moveLockTimeout = 100;
            break;
          }
          default: {break;}       
        }
      }

      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      gyroX = gyro.x();
      gyroY = gyro.y();
      gyroZ = gyro.z();
      sx += gyroX;
      sy += gyroY;
      sz += gyroZ;
      float avgX = rollAvg(gyroAvgArrX, sx);
      float avgY = rollAvg(gyroAvgArrY, sy);
      float avgZ = rollAvg(gyroAvgArrZ, sz);
      dX = avgX - avgXold;
      dY = avgY - avgYold;
      dZ = avgZ - avgZold;
      avgXold = avgX;
      avgYold = avgY;
      avgZold = avgZ;
      
      int x = dY/5.0f;
      int y = dX/5.0f;
      int z = dZ/5.0f;

      if(moveLockTimeout > 0){
        x = 0;
        y = 0;
        moveLockTimeout--;
      }

      if (x || y) {
        bleMouse.move(x, y, z);
        //Serial.print("Z:"); Serial.print(z);
      }

      ledstate = !ledstate;
      digitalWrite(LED, ledstate);
      
    }

    // central disconnected
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    bleHIDPeripheral.clearBondStoreData();
  }
}
