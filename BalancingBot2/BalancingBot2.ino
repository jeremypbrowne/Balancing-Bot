#include "MPU6050_6Axis_MotionApps20.h"
#include "BasicPIDLibraryClass.h"
#include <Adafruit_MotorShield.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <Kalman.h>
#include <PID_v1.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define accelgyro 1
#define yaxis     2
#define filters   4
#define output    5

int printType = output;

double gSetpoint = 0.4;
//double Kp= 50, Ki=250, Kd=30;
double Kp= 40, Ki=250, Kd=30;
BasicPIDLibrary myPID(Kp, Ki, Kd);
BasicPIDLibrary myPID2(Kp*2, Ki*2, Kd*2);

MPU6050 mpu;
  int16_t accX, accY, accZ;
  int16_t gyroX, gyroY, gyroZ;
  double gyroXangle, gyroYangle; // Angle calculate using the gyro only
  double compAngleX, compAngleY; // Calculated angle using a complementary filter

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);
  int Direction = 1;
  int _currentSpeed;
  double motorSpeedFactorLeft = 1.0;
  double motorSpeedFactorRight = 1.0;
  #define MIN_ABS_SPEED 15


Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
  double kalAngleY, kalAngleX;

uint8_t i2cData[14]; // Buffer for I2C data
uint32_t timer;


////////////////////////////////////////////
///////////////////////////////////////////
///////////////////////////////////////////

void setup() {
  
  Serial.begin(250000);
  Wire.begin();
  //updateLCD();

  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif  
  //pinMode(buttonPin, INPUT);

  myPID.SetOutputLimits(-255,255);
  myPID.SetSampleTime(5); 
  myPID2.SetOutputLimits(-255,255);
  myPID2.SetSampleTime(5);
 
  AFMS.begin();  // create with the default frequency 1.6KHz
    myMotor1->setSpeed(150);
    myMotor1->run(FORWARD);
    myMotor1->run(RELEASE);
    
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }   

  delay(100); // Wait for sensor to stabilize
/*
   18:47:19.391 -> Sensor readings with offsets: -2  3 16395 2 2 -1
  18:47:19.391 -> Your offsets: -1730 -1246 1792  -51 48  -11
  18:47:19.391 -> 
  18:47:19.391 -> Data is printed as: acelX acelY acelZ giroX giroY giroZ
  18:47:19.391 -> Check that your sensor readings are close to 0 0 16384 0 0 0
 */
  mpu.setXGyroOffset(-51);
  mpu.setYGyroOffset(48);
  mpu.setZGyroOffset(-11);
  mpu.setXAccelOffset(-1730);
  mpu.setYAccelOffset(-1246);
  mpu.setZAccelOffset(1792); // 1688 factory default for my test chip

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

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
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

  timer = micros();
  
}

///////////////////////////////
//////////////////////////////
//////////////////////////////

void loop() {

  unsigned long timeStamp = micros();
  //Serial.print(timeStamp*0.001);  Serial.print("\t"); // print in seconds
  
  unsigned long start = micros();
  
  /* Update all the values */  
  i2cRead(0x3B, i2cData, 14);
  
  unsigned long End = micros();
  //Serial.print((End -start));  Serial.print("\t");

  /* Read in values */
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s  
  
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;  
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  static double dOutput=0.0;

//if (abs(kalAngleY) > 5.0)
//  myPID2.Compute(gSetpoint,kalAngleY, dOutput);
//else

  myPID.Compute(gSetpoint,kalAngleY, dOutput);
//  if( abs(kalAngleY) <= gSetpoint + 0.5){
//   
//    myPID.setIntegral(0.5);    
//  }
  double myIntegral = myPID.getIntegral();
  Serial.print(myIntegral); Serial.print('\t'); Serial.print('\t');

  
if(abs(kalAngleY) >= 40){
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);

}

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


   prints(printType, accX, accY, accZ, gyroX, gyroY, gyroZ, pitch, gyroYangle, compAngleY, dOutput);
   
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
  if (dOutput == _currentSpeed) return;
  
  int realSpeed = max(MIN_ABS_SPEED, abs(dOutput));
  myMotor1->setSpeed(realSpeed * motorSpeedFactorLeft);
  myMotor2->setSpeed(realSpeed * motorSpeedFactorRight);
  
  _currentSpeed = Direction * realSpeed;


  
}

///////////////////////////////////
/////////////////////////////////
/////////////////////////////////
