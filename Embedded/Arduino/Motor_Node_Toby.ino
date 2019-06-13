#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#define Motor1_A 15
#define Motor1_B 14
#define Motor1_ENABLE 3

#define Motor2_A 12
#define Motor2_B 13
#define Motor2_ENABLE 2

bool DEBUG = 0;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

ros::NodeHandle nh;

std_msgs::Int16 l_motor;
std_msgs::Int16 r_motor;

int left_motor = 0;
int right_motor = 0;
int offset = 750;
bool stop_motors = 0;

void left_messageCb( const std_msgs::Int16 &l_motor){left_motor = l_motor.data;}
void right_messageCb( const std_msgs::Int16 &r_motor){right_motor = r_motor.data;}
void stop_messageCb( const std_msgs::Bool &stp){stop_motors = stp.data;}

ros::Subscriber<std_msgs::Int16> sub_left("left_motor", &left_messageCb );
ros::Subscriber<std_msgs::Int16> sub_right("right_motor", &right_messageCb );
ros::Subscriber<std_msgs::Bool> sub_stop("stop", &stop_messageCb );

void setup() {
  if(DEBUG) {
    Serial.begin(9600);
    Serial.println("16 channel PWM test!");
  }
  
  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency
  nh.getHardware()->setBaud(57600);
  Wire.setClock(100000);
  nh.initNode();
  nh.subscribe(sub_left);
  nh.subscribe(sub_right);
}

void loop() {
  if(!stop_motors) {
  if((left_motor > 0) && (left_motor < 3244)){forward_M1(left_motor + offset);}
  else if((left_motor < 0) && (left_motor > -3244)){reverse_M1(-1*left_motor + offset);}
  
  if((right_motor > 0) && (right_motor < 3244)){forward_M2(right_motor + offset);}
  else if((right_motor < 0) && (right_motor > -3244)){reverse_M2(-1*right_motor + offset);}
  
  nh.spinOnce();
  }
  else{forward_M1(0);forward_M2(0);}
}

void enable_M1() {
  digitalWrite(Motor1_ENABLE, HIGH);
}

void enable_M2() {
  digitalWrite(Motor2_ENABLE, HIGH);
}

void disable_M1() {
  digitalWrite(Motor1_ENABLE, LOW);
}

void disable_M2() {
  digitalWrite(Motor2_ENABLE, LOW);
}

void forward_M1(int pwm_freq) {
  enable_M1();
  pwm.setPWM(Motor1_B, 0, 0);
  pwm.setPWM(Motor1_A, 0, pwm_freq);
}

void reverse_M1(int pwm_freq) {
  enable_M1();
  pwm.setPWM(Motor1_B, 0, pwm_freq);
  pwm.setPWM(Motor1_A, 0, 0);
}

void forward_M2(int pwm_freq) {
  enable_M2();
  pwm.setPWM(Motor2_B, 0, 0);
  pwm.setPWM(Motor2_A, 0, pwm_freq);
}

void reverse_M2(int pwm_freq) {
  enable_M2();
  pwm.setPWM(Motor2_B, 0, pwm_freq);
  pwm.setPWM(Motor2_A, 0, 0);
}

