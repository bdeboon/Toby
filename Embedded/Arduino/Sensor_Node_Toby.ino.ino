#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

//ROS-related headers
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>

//DEBUG FLAGS
bool DEBUG_ENC = 0;
bool DEBUG_IMU = 0;

//Encoder pin assignment
#define enc1_a 18
#define enc1_b 26
#define enc2_a 19
#define enc2_b 28

//Init counters for encoders
volatile long int counter_1 = 0; 
volatile long int counter_2 = 0;

//Init IMU
MPU6050 mpu;
int16_t ax, ay, az; //acceleration
int16_t gx, gy, gz; //gyroscope
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double yaw_offset = 0;
volatile bool mpuInterrupt = false;

void setup() {
  Wire.begin(); // join I2C bus
  TWBR = 24; // 400kHz I2C clock
  if(DEBUG_ENC || DEBUG_IMU){Serial.begin(57600); }
  
  pinMode(enc1_a, INPUT_PULLUP);           // set pin to input
  pinMode(enc1_b, INPUT_PULLUP);           // set pin to input
  pinMode(enc2_a, INPUT_PULLUP);           
  pinMode(enc2_b, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(18), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(19), bi0, RISING);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(230);
  mpu.setYGyroOffset(200);
  mpu.setZGyroOffset(50);
  mpu.setZAccelOffset(1360);
  
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  
}

void loop() {
  
  if(DEBUG_ENC){
    Serial.print("Counter 1: ");
    Serial.print(counter_1);
    Serial.print(" Counter 2: ");
    Serial.println(counter_2);
  }

  if (!dmpReady) return;
  
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {mpu.resetFIFO();}
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    //Actual IMU data
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    if(DEBUG_IMU) {
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
    }
   
  }
  
}

void ai0() {if(digitalRead(enc1_b)==LOW) {counter_1++;}else{counter_1--;}}

void bi0() { if(digitalRead(enc2_b)==LOW) {counter_2++;}else{counter_2--;}}
