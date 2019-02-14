#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_MotorShield.h>
#include <LiquidCrystal.h>



#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//button states
#define ButtonState_Up      1
#define ButtonState_Pressed 2 
#define ButtonState_Down    3
#define ButtonState_Released 4

//map button high/low to up/down
#define ButtonRead_Up 1
#define ButtonRead_Down 2


#define buttonPin 6
#define MIN_ABS_SPEED 20

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);
const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 178;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
int moveState=0; //0 = balance; 1 = back; 2 = forth
double Kp = 90;
double Kd = 1.4;
double Ki = 81;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.7;
double motorSpeedFactorRight = 0.8;
int _currentSpeed;
int Direction = 1;
//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

//timers
long time1Hz = 0;
long time5Hz = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}


void setup()
{
   
    pinMode(buttonPin, INPUT);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    
    Serial.println("Adafruit Motorshield v2 - DC Motor test!");

    AFMS.begin();  // create with the default frequency 1.6KHz
    //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

    // Set the speed to start, from 0 (off) to 255 (max speed)
    myMotor1->setSpeed(150);
    myMotor1->run(FORWARD);
    // turn on motor
    myMotor1->run(RELEASE);
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  lcd.begin(16, 2);
  
  lcd.setCursor(0,0);
  lcd.write("P:");
  lcd.setCursor(7,0);
  lcd.write("I:");  
  lcd.setCursor(0,1);
  lcd.write("D:"); 


    
}

/*****************************************************************
 * 
 * 
 * 
 ****************************************************************/


int UpdateButtonState(int iButtonState, int iNewButtonRead)
{
  if(iButtonState == ButtonState_Up && iNewButtonRead == ButtonRead_Down)
    return ButtonState_Pressed;
  else if(iButtonState == ButtonState_Pressed)
    return ButtonState_Down;
  else if(iButtonState == ButtonState_Down && iNewButtonRead == ButtonRead_Up)
    return ButtonState_Released;
  else if(iButtonState == ButtonState_Released)
    return ButtonState_Up;
    
  return iButtonState;
}

int checkButton(int iButtonPin)
{
  int newButtonRead = digitalRead(iButtonPin);
  delay(1);
  int debounceButtonRead = digitalRead(iButtonPin);
  if(newButtonRead != debounceButtonRead)
    return -1;//button is not stable

  if(newButtonRead == LOW) //Button is active low, ie using pullup
    return ButtonRead_Down;
  return ButtonRead_Up;
}

void loop()
{
  static int ButtonState = ButtonState_Up;
  static int speedState = 1;
  static int Speed = 1;

  ButtonState = UpdateButtonState(ButtonState, checkButton(buttonPin));
 
  while (ButtonState == ButtonState_Up)
  {
    myMotor1->setSpeed(0);
    myMotor2->setSpeed(0);
    Serial.println("pressed");
     updateLCD();
     ButtonState = UpdateButtonState(ButtonState, checkButton(buttonPin));

  }


    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors
        
        pid.Compute();
       
        if (output < 0)
        {
          Direction= -1;
          myMotor1->run(BACKWARD);
          myMotor2->run(BACKWARD);
       
          output = min(output, -1*MIN_ABS_SPEED);
          output = max(output, -255);
        }
        else
        {
          Direction = 1;          
          myMotor1->run(FORWARD);
          myMotor2->run(FORWARD);
          
          output = max(output, MIN_ABS_SPEED);
          output = min(output, 255);
        }
    
        if (output == _currentSpeed) return;
    
        int realSpeed = max(MIN_ABS_SPEED, abs(output));
    
//        digitalWrite(_in1, output > 0 ? HIGH : LOW);
//        digitalWrite(_in2,output > 0 ? LOW : HIGH);
//        digitalWrite(_in3, output > 0 ? HIGH : LOW);
//        digitalWrite(_in4, output > 0 ? LOW : HIGH);
//        analogWrite(_ena, realSpeed * _motorAConst);
//        analogWrite(_enb, realSpeed * _motorBConst);

        myMotor1->setSpeed(realSpeed * motorSpeedFactorLeft);
        myMotor2->setSpeed(realSpeed * motorSpeedFactorRight);
        
        _currentSpeed = Direction * realSpeed;
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //  #if LOG_INPUT
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.print("\t");
            Serial.print("out: ");
            Serial.print("\t");
            Serial.println(output);
        //#endif
        input = ypr[1] * 180/M_PI + 180;
   }

}

double updateLCD()
{
  static float sensorValue1 = 0;        // value read from the pot
  static float sensorValue2 = 0;        // value read from the pot
  static float sensorValue3 = 0;        // value read from the pot
//
//  // read the analog in value:
  sensorValue1 = analogRead(A0);
  sensorValue2 = analogRead(A1);
  sensorValue3 = analogRead(A2);
//
//  // map it to the range of the analog out:
  Kp = map(sensorValue1, 0, 1023, 0.00, 255.00);
  Ki = map(sensorValue2, 0, 1023, 0.00, 255.00);
  Kd = map(sensorValue3, 0, 1023, 0.00, 255);
//
 lcd.setCursor(3,0);
lcd.print(Kp,0);
  lcd.setCursor(10,0);
  lcd.print(Ki,0);  
  lcd.setCursor(3,1);
  lcd.print(Kd,0);   

}
