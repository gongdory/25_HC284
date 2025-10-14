#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

// #include <Wire.h>
// #include <MPU9250.h>
// #include <MadgwickAHRS.h>


ros::NodeHandle nh;

// 모터 제어 핀
#define ENA 6  // 오른 모터 속도 (PWM)
#define IN1 10  //  모터 방향
#define IN2 11  //  모터 방향


#define ENB 5  // 왼쪽 모터 속도 (PWM)
#define IN3 2 //  모터 방향
#define IN4 3  //  모터 방향



// MPU9250 mpu;
// Madgwick filter;

// float ax, ay, az;
// float gx, gy, gz;



const float MAX_LIN_VEL = 0.6; // m/s
const float MAX_ANG_VEL = 1.0; // rad/s

int pwm_left = 0;
int pwm_right = 0;

float joy_left = 0.0;
float joy_right = 0.0;
int joy_flag = 0;
int before_flag = 0;


// cmd_vel 입력 변수
float linear_x = 0.0;
float angular_z = 0.0;

// 콜백 함수: PC에서 온 조이스틱 값 수신
void joyCallback(const std_msgs::Float32MultiArray &msg) {
  int flag = 0;
  if (msg.data_length >= 3) {
    joy_left = msg.data[0]; 
    joy_right = msg.data[1];
    flag = msg.data[2]; 
  }
  joy_left = int(joy_left * 50);
  joy_right = int(joy_right * 50);

  if (flag == 1 && (before_flag != flag)) {
    if (joy_flag == 0) joy_flag = 1;
    else joy_flag = 0;
    

  }
  before_flag = flag;
}


// cmd_vel 콜백
void cmdVelCallback(const geometry_msgs::Twist &msg) {
  linear_x = msg.linear.x;
  angular_z = msg.angular.z;

  if (linear_x > MAX_LIN_VEL) linear_x = MAX_LIN_VEL;
  else if (linear_x < (MAX_LIN_VEL * (-1))) linear_x = (MAX_LIN_VEL * (-1));
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("joy_cmd", &joyCallback);
ros::Subscriber<geometry_msgs::Twist> cmdvel_sub("cmd_vel", &cmdVelCallback);




void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(cmdvel_sub);

  // Serial.begin(115200);
  // Wire.begin();
  // if (!mpu.setup(0x68)) {
  //   Serial.println("MPU9250 not found");
  //   while (1);
  // }
  // mpu.calibrateAccelGyro();
  // filter.begin(200);  // 예상 주기 200Hz
  

  // 핀 설정
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);

  // bool a = true;
  // bool b = false;
  // digitalWrite(IN1, b);
  // digitalWrite(IN2, a);
  // digitalWrite(IN3, b);
  // digitalWrite(IN4, a);
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);

  analogWrite(ENA, 0);  // 0~255 사이 PWM 값
  analogWrite(ENB, 0);
  delay(100);

  pinMode(13, 1);



}




void loop() {
  nh.spinOnce();
  delay(10);


  if (joy_flag == 1) {
    // // left_speed = map(joy_left, -1, 1, -100, 100);  위에서 x100 해서 상관없음
    // // right_speed = map(joy_right, -1, 1, -100, 100);

    bool dir_left = joy_left >= 0 ? 0 : 1;
    bool dir_right = joy_right >= 0 ? 0 : 1;
    // 모터 출력
    digitalWrite(IN1, dir_right);
    digitalWrite(IN2, !dir_right);
    digitalWrite(IN3, dir_left);
    digitalWrite(IN4, !dir_left);
    analogWrite(ENA, abs(joy_right));
    analogWrite(ENB, abs(joy_left));

   digitalWrite(13, 1);
  
  }
  else if (joy_flag == 0) {
    //   // *************
    // // 2) cmd_vel 처리 (선속도 + 각속도 → 바퀴속도)
    // *************
    float v = linear_x;   // m/s
    float w = angular_z;  // rad/s

    // 로봇 파라미터 (축 간 거리)
    const float WHEEL_BASE = (0.2769 * 2.0); // m

    // 좌/우 바퀴 선속도
    float v_left = v - (w * WHEEL_BASE / 2.0);
    float v_right = v + (w * WHEEL_BASE / 2.0);

    // 속도를 PWM 값으로 변환
    pwm_left = int(constrain((v_left / (MAX_LIN_VEL + (MAX_ANG_VEL * WHEEL_BASE / 2.0))) * 250.0, -255, 255));
    pwm_right = int(constrain((v_right / (MAX_LIN_VEL + (MAX_ANG_VEL * WHEEL_BASE / 2.0))) * 250.0, -255, 255));

    // pwm이 양수/음수에 따라 방향 설정
    bool dir_cmd_left = pwm_left >= 0 ? 0 : 1;
    bool dir_cmd_right = pwm_right >= 0 ? 0 : 1;


    // 모터 출력
    // digitalWrite(IN1, !dir_cmd_right);
    // digitalWrite(IN2, dir_cmd_right);
    digitalWrite(IN1, !dir_cmd_left);
    digitalWrite(IN2, dir_cmd_left);
    // digitalWrite(IN3, !dir_cmd_left);
    // digitalWrite(IN4, dir_cmd_left);
    digitalWrite(IN3, !dir_cmd_right);
    digitalWrite(IN4, dir_cmd_right);
    // analogWrite(ENA, abs(pwm_right));
    // analogWrite(ENB, abs(pwm_left));

    analogWrite(ENB, abs(pwm_right));
    analogWrite(ENA, abs(pwm_left));


    digitalWrite(13, 0);
  }
















  // if (mpu.update()) {
  //   ax = mpu.getAccX();
  //   ay = mpu.getAccY();
  //   az = mpu.getAccZ();

  //   gx = mpu.getGyroX() * DEG_TO_RAD;
  //   gy = mpu.getGyroY() * DEG_TO_RAD;
  //   gz = mpu.getGyroZ() * DEG_TO_RAD;

  //   filter.updateIMU(gx, gy, gz, ax, ay, az);

  //   // Euler angles만 사용
  //   float yaw   = filter.getYaw();
  //   float pitch = filter.getPitch();
  //   float roll  = filter.getRoll();

  //   // CSV 출력: ax, ay, az, gx, gy, gz, roll, pitch, yaw
  //   Serial.print(ax); Serial.print(",");
  //   Serial.print(ay); Serial.print(",");
  //   Serial.print(az); Serial.print(",");
  //   Serial.print(gx); Serial.print(",");
  //   Serial.print(gy); Serial.print(",");
  //   Serial.print(gz); Serial.print(",");
  //   Serial.print(roll); Serial.print(",");
  //   Serial.print(pitch); Serial.print(",");
  //   Serial.println(yaw);
  // }

  
}
