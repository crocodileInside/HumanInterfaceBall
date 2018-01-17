#include <SPI.h>
#include <BLEHIDPeripheral.h>
#include <BLEMouse.h>
#include <BLEKeyboard.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <utility/imumaths.h>



//custom boards may override default pin definitions with BLEHIDPeripheral(PIN_REQ, PIN_RDY, PIN_RST)
BLEHIDPeripheral bleHIDPeripheral = BLEHIDPeripheral();
BLEMouse bleMouse;
BLEKeyboard bleKeyboard;

int buttonState;
int joystickXCenter;
int joystickYCenter;

float basePressure = 954.0f;


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


#define ROLL_AVG_SIZE   8
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
      Serial.println(dP);
      if(dP > 3.0f){
        tempButtonState = 1;
        moveLockTimeout = 50;
      }else if(dP < 1.0f){
        tempButtonState = 2;
      }
      
      if (tempButtonState != buttonState) {
        buttonState = tempButtonState;

        if (buttonState == 1) {
          Serial.println(F("Mouse press"));
          bleMouse.press();
        } else if(buttonState == 2){
          Serial.println(F("Mouse release"));
          bleMouse.release();
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

      if(moveLockTimeout > 0){
        x = 0;
        y = 0;
        moveLockTimeout--;
      }

      if (x || y) {
        bleMouse.move(x, y);
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
