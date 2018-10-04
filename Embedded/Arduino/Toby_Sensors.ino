
//Includes

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <ros.h>


#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

//Comment one of these out to display IMU data to Serial Monitor
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL
#define OUTPUT_READABLE_QUATERNION

//Arduino Encoder Pins 
#define enc1_a 2
#define enc1_b 3
#define enc2_a 18
#define enc2_b 19

//Arduino Motor Control Pins
#define left_motor 4
#define right_motor 5

// IMU Initialization
MPU6050 mpu;
int16_t ax, ay, az; // define accel as ax,ay,az
int16_t gx, gy, gz; // define gyro as gx,gy,gz 
unsigned long calibrate_check; //Calibration_timing check
int16_t ax_cal = 0, ay_cal = 0, az_cal = 0; //Values after calibration
int16_t gx_cal = 0, gy_cal = 0, gz_cal = 0;
int calibrate = 0; //Calibration Flag
//////////////////////////
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double yaw_offset = 0;
/////////////////////Imu Angular Velocity////////
float old_tilt = 0;
float old_heading = 0;
float tilt_vel;
float heading_vel;
unsigned long old_time = 0;
unsigned long new_time = 0;
////////////// Input Voltage initialization///////////////////
float left_motor_voltage;
float right_motor_voltage;
float in_left = 0;
float in_right = 0;



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



volatile long int counter_1 = 0; 
volatile long int counter_2 = 0;//This variable will increase or decrease depending on the rotation of encoder

unsigned long oldtime = 0;
unsigned long newtime;
long delta_time;
long vel_1;
long vel_2;
long x_vel_1;
long x_vel_2;
long x_vel;
long newposition_1 = 0;
long oldposition_1 = 0;
long newposition_2 = 0;
long oldposition_2 = 0;


//ROS Node Setup//////////////////////////////////////
ros::NodeHandle  nh;

// Input Voltage Callback/////////////////////////////
//void voltageCb(const geometry_msgs::Vector3 &voltage_msg){
//  left_motor_voltage = voltage_msg.x;
//  right_motor_voltage = voltage_msg.y;
//}


//ROS msg setup////////////////////////////////////////
std_msgs::Float64 linear_velocity;
ros::Publisher vel_pub("linear_velocity", &linear_velocity);
//tf::TransformBroadcaster odom_broadcaster;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);
//ros::Subscriber<geometry_msgs::Vector3> input_sub("motor_voltages", &voltageCb );

/////////////////////////////////////////////////////////

void setup() {
  Wire.begin(); // join I2C bus
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  Serial.begin(57600); // initialize serial communication
 
  pinMode(enc1_a, INPUT_PULLUP);           // set pin to input
  pinMode(enc1_b, INPUT_PULLUP);           // set pin to input
  pinMode(enc2_a, INPUT_PULLUP);           // set pin to input
  pinMode(enc2_b, INPUT_PULLUP);

  pinMode(left_motor, OUTPUT);
  pinMode(right_motor, OUTPUT);

  analogWrite(left_motor, 135); //Turn motors 'off'
  analogWrite(right_motor, 135);
  
  mpu.initialize();
  calibrate_check = millis();
  //Serial.println(mpu.testConnection() ? "MPU6050 connection successful"
  //: "MPU6050 connection failed"); 
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(230);
  mpu.setYGyroOffset(200);
  mpu.setZGyroOffset(50);
  mpu.setZAccelOffset(1360); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(digitalPinToInterrupt(2), ai0, RISING);
  attachInterrupt(digitalPinToInterrupt(3), ai1, RISING);
  attachInterrupt(digitalPinToInterrupt(18), bi0, RISING);
  attachInterrupt(digitalPinToInterrupt(19), bi1, RISING);


  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(vel_pub);
  //nh.subscribe(input_sub);

  imu_msg.header.stamp = nh.now();
  //imu_msg.header.frame_id = "/base_link";
  imu_msg.header.frame_id = "/map";
  //imu_msg.header.frame_id = "/fixed_frame";
  imu_msg.header.seq = 0;
  
}

void loop() {
  nh.spinOnce();
  delay(10);
  
  //in_left = round(135 + 120*(left_motor_voltage/22));
  //in_right = round(135 + 120*(right_motor_voltage/22));
  if(in_left > 255){
    in_left = 255;
  }
  if(in_left < 0) {
    in_left = 0;
  }
  if(in_right > 255){
    in_right = 255;
  }
  if(in_right < 0) {
    in_right = 0;
  }
  //analogWrite(left_motor, in_left); //Turn motors 'off'
  //analogWrite(right_motor, in_right);
  
  ///////////////////////////////// IMU /////////////////////
  if (!dmpReady) return;
  // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            //Serial.print("quat\t");
            //Serial.print(q.w);
            //Serial.print("\t");
            //Serial.print(q.x);
            //Serial.print("\t");
            //Serial.print(q.y);
            //Serial.print("\t");
            //Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            //Serial.print("euler\t");
            //Serial.print(euler[0] * 180/M_PI);
            //Serial.print("\t");
            //Serial.print(euler[1] * 180/M_PI);
            //Serial.print("\t");
            //Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw_offset = yaw_offset + 0.00006;
            //Serial.print("ypr\t");
            //Serial.print(ypr[0] * 180/M_PI + yaw_offset - 4.78);
            //Serial.print("\t");
            //Serial.print(ypr[1] * 180/M_PI - 2.12);
            //Serial.print("\t");
            //Serial.println(ypr[2] * 180/M_PI + 1.9);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            //Serial.print("areal\t");
            //Serial.print(aaReal.x);
            //Serial.print("\t");
            //Serial.print(aaReal.y);
            //Serial.print("\t");
            //Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            //Serial.print("aworld\t");
            //Serial.print(aaWorld.x*0.041);
            //Serial.print("\t");
            //Serial.print(aaWorld.y*0.041);
            //Serial.print("\t");
            //Serial.println(aaWorld.z*0.041);
      #endif

      ////// Angular velocity Calculations/////////////
      new_time = millis();
      tilt_vel = float((float(q.x) - old_tilt)/float(new_time - old_time));
      heading_vel = float((float(q.z) - old_heading)/float(new_time - old_time));

      old_tilt = float(q.x);
      old_heading = float(q.z);
      old_time = new_time;
      //////////////////////////////////////////////////
      
      imu_msg.header.stamp = nh.now();
      imu_msg.orientation.x = float(q.x);
      imu_msg.orientation.y = float(q.y);
      imu_msg.orientation.z = float(q.z);
      imu_msg.orientation.w = float(q.w);
      imu_msg.angular_velocity.x = tilt_vel;
      imu_msg.angular_velocity.z = heading_vel;
      imu_msg.linear_acceleration.x = float(aaWorld.x*0.041);
      imu_msg.linear_acceleration.y = float(aaWorld.y*0.041);
      imu_msg.linear_acceleration.z = float(aaWorld.z*0.041);

      

      
      
      imu_pub.publish( &imu_msg );
      
    
  }
  
  newposition_1 = counter_1;
  newposition_2 = counter_2;
  newtime = millis();
  delta_time = (newtime - oldtime);
  vel_1 = (newposition_1 - oldposition_1)/delta_time;
  vel_1 = vel_1*0.2032/7200;
  vel_2 = (newposition_2 - oldposition_2)/delta_time;
  vel_2 = vel_2*0.2023/18000;
  oldposition_1 = newposition_1;
  oldposition_2 = newposition_2;
  oldtime = newtime;

  x_vel = (vel_1 + vel_2)/2;

  linear_velocity.data = float(x_vel);
  delay(1);
  vel_pub.publish( &linear_velocity);
  ///ODOM for ROS
  //current_time = ros::Time::now();

  
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(enc1_b)==LOW) {
    counter_1++;
  }else{
    counter_1--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(enc1_a)==LOW) {
    counter_1--;
  }else{
    counter_1++;
  }
}

void bi0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(enc2_b)==LOW) {
    counter_2++;
  }else{
    counter_2--;
  }
}

void bi1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(enc2_a)==LOW) {
    counter_2--;
  }else{
    counter_2++;
  }
}
