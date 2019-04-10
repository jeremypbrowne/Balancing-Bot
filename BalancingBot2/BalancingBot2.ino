#include "MPU6050.h"
#include "BasicPIDLibraryClass.h"
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include <Wire.h>
#include "Kalman.h"

double gSetpoint = 0.4;
double Kp= 40, Ki=250, Kd=30;
BasicPIDLibrary myPID(Kp, Ki, Kd);

MPU6050 mpu;


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);

#define MIN_ABS_SPEED 15


Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;


////////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
 return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
 Wire.beginTransmission(IMUAddress);
 Wire.write(registerAddress);
 Wire.write(data, length);
 uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
 if (rcode) {
   Serial.print(F("i2cWrite failed: "));
   Serial.println(rcode);
 }
 return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
 uint32_t timeOutTimer;
 Wire.beginTransmission(IMUAddress);
 Wire.write(registerAddress);
 uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
 if (rcode) {
   Serial.print(F("i2cRead failed: "));
   Serial.println(rcode);
   return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
 }
 Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
 for (uint8_t i = 0; i < nbytes; i++) {
   if (Wire.available())
     data[i] = Wire.read();
   else {
     timeOutTimer = micros();
     while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
     if (Wire.available())
       data[i] = Wire.read();
     else {
       Serial.println(F("i2cRead timeout"));
       return 5; // This error value is not already taken by endTransmission
     }
   }
 }
 return 0; // Success
}

void setup() 
{
  Serial.begin(250000);
  Wire.begin();
  
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
    
  myPID.SetOutputLimits(-255,255);
  myPID.SetSampleTime(5); 
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor1->setSpeed(150);
  myMotor1->run(FORWARD);
  myMotor1->run(RELEASE);
  
  uint8_t i2cData[14]; // Buffer for I2C data
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }   
  
  delay(100); // Wait for sensor to stabilize

  mpu.setXGyroOffset(-51);
  mpu.setYGyroOffset(48);
  mpu.setZGyroOffset(-11);
  mpu.setXAccelOffset(-1730);
  mpu.setYAccelOffset(-1246);
  mpu.setZAccelOffset(1792); // 1688 factory default for my test chip
  
  /* Set kalman and gyro starting angle */
  int16_t accX, accY, accZ;
  int16_t gyroX, gyroY, gyroZ;
  mpu.getMotion6(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ);
  
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  
  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif
  
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
 
}

///////////////////////////////
//////////////////////////////
//////////////////////////////

void loop() {

  unsigned long timeStamp = micros(); 
  double kalAngleY, kalAngleX;

  //uint8_t i2cData[14]; // Buffer for I2C data
  uint32_t timer;

  int16_t accX, accY, accZ;
  int16_t gyroX, gyroY, gyroZ;
  mpu.getMotion6(&accX,&accY,&accZ,&gyroX,&gyroY,&gyroZ);
 
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s  

  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  Serial.println(kalAngleY);
  
  static double dOutput=0.0;

  myPID.Compute(gSetpoint,kalAngleY, dOutput);
  
  if(abs(kalAngleY) >= 40)
  {
    myMotor1->setSpeed(0);
    myMotor2->setSpeed(0);
  }

  int Direction = 1;
  int _currentSpeed;
  double motorSpeedFactorLeft = 1.0;
  double motorSpeedFactorRight = 1.0;
  if (dOutput < 0){
    Direction= -1;
    myMotor1->run(FORWARD);
    myMotor2->run(BACKWARD);
    
    dOutput = min(dOutput, -1*MIN_ABS_SPEED);
    dOutput = max(dOutput, -255);
  }
  else
  {
    Direction = 1;          
    myMotor1->run(BACKWARD);
    myMotor2->run(FORWARD);
    
    dOutput = max(dOutput, MIN_ABS_SPEED);
    dOutput = min(dOutput, 255);
  }
  if (dOutput == _currentSpeed) 
    return;
  
  int realSpeed = max(MIN_ABS_SPEED, abs(dOutput));
  myMotor1->setSpeed(realSpeed * motorSpeedFactorLeft);
  myMotor2->setSpeed(realSpeed * motorSpeedFactorRight);
  
  _currentSpeed = Direction * realSpeed;

}
