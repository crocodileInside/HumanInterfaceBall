#include <SPI.h>
#include <BLEHIDPeripheral.h>
#include <BLEMouse.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <utility/imumaths.h>
#include <utility/quaternion.h>




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

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, 1);


  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)){
    Serial.println("no BNO055 detected!");
  }

  if(!bme.begin()){
    Serial.println("no BME280 detected!");
  }
  
  bno.setExtCrystalUse(false);
  
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

void loop() {

  sensors_event_t event;
  bno.getEvent(&event);
  
  oriX = event.orientation.x;
  oriY = event.orientation.y;
  oriZ = event.orientation.z + 180.0f;

  /*Serial.print(oriX);
  Serial.print(" ");
  Serial.print(oriY);
  Serial.print(" ");
  Serial.print(oriZ);*/
  //Serial.print(" ");

  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  gyroX = gyro.x();
  gyroY = gyro.y();
  gyroZ = gyro.z();
  
  sx+=gyroX;
  sy+=gyroY;
  sz+=gyroZ;

  float avgX = rollAvg(gyroAvgArrX, sx);
  float avgY = rollAvg(gyroAvgArrY, sy);
  float avgZ = rollAvg(gyroAvgArrZ, sz);

  dX = avgX - avgXold;
  dY = avgY - avgYold;
  dZ = avgZ - avgZold;

  avgXold = avgX;
  avgYold = avgY;
  avgZold = avgZ;

  /*Serial.print(avgX);
  Serial.print(" ");
  Serial.print(avgY);
  Serial.print(" ");
  Serial.print(avgZ);

  Serial.print(" ");*/
  Serial.print(dX);
  Serial.print(" ");
  Serial.print(dY);
  Serial.print(" ");
  Serial.print(dZ);



  /*imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> angVel = quat.toEuler();
  Serial.print(angVel.x());
  Serial.print(" ");
  Serial.print(angVel.y());
  Serial.print(" ");
  Serial.print(angVel.z());*/


  /*// Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  // Display the floating point data
  Serial.print(acc.x());
  Serial.print(" ");
  Serial.print(acc.y());
  Serial.print(" ");
  Serial.print(acc.z());
  Serial.print(" ");
 

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print(system, DEC);
  Serial.print(" ");
  Serial.print(gyro, DEC);
  Serial.print(" ");
  Serial.print(accel, DEC);
  Serial.print(" ");
  Serial.print(mag, DEC);

  Serial.print(" ");

  Serial.print(bme.readTemperature());
  Serial.print(" ");
  //Serial.print(bme.readPressure()/100.0f);
  Serial.print("0.0");
  Serial.print(" ");
  Serial.print(bme.readHumidity());


  Serial.print(" ");*/
  

   if(abs(oriXold - oriX) > 300){
    if(oriX < oriXold){
      dOriX = (oriX + 360.0f) - oriXold;
    }else if(oriX > oriXold){
      dOriX = oriX - (oriXold + 360);
    }
  }else{
    dOriX = oriX - oriXold;
  }

  if(abs(oriYold - oriY) > 300){
    if(oriY < oriYold){
      dOriY = (oriY + 360.0f) - oriYold;
    }else if(oriY > oriYold){
      dOriY = oriY - (oriYold + 360);
    }
  }else{
    dOriY = oriY - oriYold;
  }

  if(abs(oriZold - oriZ) > 300){
    if(oriZ < oriZold){
      dOriZ = (oriZ + 360.0f) - oriZold;
    }else if(oriZ > oriZold){
      dOriZ = oriZ - (oriZold + 360);
    }
  }else{
    dOriZ = oriZ - oriZold;
  }

  /*Serial.print(oriY);
  Serial.print(" ");
  Serial.print(dOriZ);*/

  /*Serial.print(dOriX);
  Serial.print(" ");
  Serial.print(dOriY);
  Serial.print(" ");
  Serial.print(dOriZ);*/

  oriXold = oriX;
  oriYold = oriY;
  oriZold = oriZ;
  /*Serial.print(" ");
  Serial.print(bme.readPressure()/100.0f);*/
  Serial.println("");

  ledstate = !ledstate;
  digitalWrite(LED, ledstate);
  delay(50);

  
}
