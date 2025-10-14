#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;
std_msgs::Int16MultiArray sensor_msg;
ros::Publisher pub("angle_sensor_raw", &sensor_msg);

int sensor_val[19];

int num1_s0 = 8;
int num1_s1 = 9;
int num1_s2 = 10;

int num2_s0 = 5;
int num2_s1 = 6;
int num2_s2 = 7;

int num3_s0 = 11;
int num3_s1 = 12;
int num3_s2 = 13;

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub);

  Serial.println("start");
  pinMode(num1_s0, OUTPUT);
  pinMode(num1_s1, OUTPUT);
  pinMode(num1_s2, OUTPUT);
  pinMode(num2_s0, OUTPUT);
  pinMode(num2_s1, OUTPUT);
  pinMode(num2_s2, OUTPUT);
  pinMode(num3_s0, OUTPUT);
  pinMode(num3_s1, OUTPUT);
  pinMode(num3_s2, OUTPUT);

  sensor_msg.data = sensor_val;
  sensor_msg.data_length = 19;
}

void selectChannel(int num, int ch) {
  int s0_pin = 0;
  int s1_pin = 0;
  int s2_pin = 0;

  if (num == 0) {
    s0_pin = num1_s0;
    s1_pin = num1_s1;
    s2_pin = num1_s2;
  }
  else if (num == 1) {
    s0_pin = num2_s0;
    s1_pin = num2_s1;
    s2_pin = num2_s2;
  }
  else if (num == 2) {
    s0_pin = num3_s0;
    s1_pin = num3_s1;
    s2_pin = num3_s2;
  }
  // ch : 0 ~ 7
  digitalWrite(s0_pin, (ch & 0x01) ? HIGH : LOW);
  digitalWrite(s1_pin, (ch & 0x02) ? HIGH : LOW);
  digitalWrite(s2_pin, (ch & 0x04) ? HIGH : LOW);
  delayMicroseconds(10); // 신호 안정화
}


void loop() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 7; j++) {
      if ((i ==2) && (j >= 3)) break;
      selectChannel(i, j);
      if (i==0) sensor_val[(i*7) + j] = analogRead(A0);
      else if (i==1) sensor_val[(i*7) + j] = analogRead(A1);
      else if (i==2) sensor_val[(i*7) + j] = analogRead(A2);
      
    }
  }
  sensor_val[16] = 0;
  sensor_val[17] = 0;
  sensor_val[18] = 0;

  pub.publish(&sensor_msg);
  nh.spinOnce();
  delay(5);
}
