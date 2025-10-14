#include <DynamixelWorkbench.h>
#include <CAN.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#define SEND_INTERVAL_MS 1000
#define PI 3.14159265

//객체 선언
DynamixelWorkbench dxl;
HardwareSerial &rosSerial = Serial1;
ros::NodeHandle nh;

uint32_t t_time;
can_message_t tx_msg, rx_msg;
static const uint32_t kExtId_0 = 0x00000600;
static const uint32_t kExtId_1 = 0x00000601;
static const uint32_t kExtId_2 = 0x00000602;


const uint32_t BAUDRATE  = 57600;
const char*    DXL_PORT  = "Serial3";

int angle_to_bin(float angle);
void MX64_get_status(void);
void MX64_set_rad(int mx64_num, float rad);
void AK_motor_get_status(void);
void AK_motor_set_rad(int can_num, float rad);
void XL330_set_rad(int xl_num, float rad);
void jointStateCallback(const sensor_msgs::JointState &msg);


ros::Subscriber<sensor_msgs::JointState> jointStateSub("joint_states", &jointStateCallback);

void setup() {
  Serial.begin(115200);
  delay(300);
  
  rosSerial.begin(57600);
  nh.initNode();
  nh.subscribe(jointStateSub);
  delay(100);

  if (!dxl.begin(DXL_PORT, BAUDRATE)) {
    Serial.println("[ERR] Failed to open DXL port");
    while (1);
  }

  uint16_t model = 0;
  for (int i = 3; i < 23; i++) {
    dxl.ping(i, &model);

    dxl.itemWrite(i, "Torque_Enable", 0);
    bool ok_mode5 = dxl.itemWrite(i, "Operating_Mode", 5);  // 5, 전류 위치 제어

    if (ok_mode5) {
      if (i < 6) dxl.itemWrite(i, "Current_Limit", 1900);
      else dxl.itemWrite(i, "Current_Limit", 1000);
    }

    if (i < 6) {
      // 이동 프로파일
      dxl.itemWrite(i, "Profile_Acceleration", 100);
      dxl.itemWrite(i, "Profile_Velocity",     100);
      dxl.itemWrite(i, "Position_P_Gain", 200);
      dxl.itemWrite(i, "Position_I_Gain", 200);
      dxl.itemWrite(i, "Position_D_Gain", 600);
    }
    else {
      dxl.itemWrite(i, "Profile_Acceleration", 100);
      dxl.itemWrite(i, "Profile_Velocity",     100);
      dxl.itemWrite(i, "Position_P_Gain", 100);
      dxl.itemWrite(i, "Position_I_Gain", 100);
      dxl.itemWrite(i, "Position_D_Gain", 500);
    }


    if (!dxl.torqueOn(i)) {
      Serial.print("[ERR] Torque enable failed @ID "); Serial.println(i);
      continue;
    }

    dxl.goalPosition(i, angle_to_bin(0));
  }




  
  if (CanBus.begin(CAN_BAUD_500K, CAN_EXT_FORMAT) == false)
  {
    Serial.println("CAN open fail!!");
    while (1) { }
  }
  Serial.println("CAN start!!!");
  CanBus.configFilter(0x00000000, 0x00000000, CAN_EXT_FORMAT);
  delay(100);
  for(int i = 0; i < 3; i++) {
    AK_motor_set_rad(i, 0.0);
  }
}

void loop() {
  
  nh.spinOnce();
  delay(10);
}




void jointStateCallback(const sensor_msgs::JointState &msg) {
  for (int i = 0; i < 20; i++) {
    input_rad[i] = msg.position[i];
    if (i<3) AK_motor_set_rad(input_rad[i]);
    else if ((i>2) && (i<6)) MX64_set_rad(input_rad[i]);
    else 
  }
}

void XL330_set_rad(int xl_num, float rad) {
  float xl_deg = rad * 180.0f / PI;
  xl_deg = angle_to_bin(xl_deg);
  dxl.goalPosition(xl_num, xl_deg);
}


void MX64_get_status(void) {
  int32_t MX64_pos=0;
  for(int i = 3l i < 6; i++) {
    if (dxl.itemRead(ID, "Present_Position", &MX64_pos)) {
      Serial.print("Present_Position: ");
      Serial.println(MX64_pos);
    }  
  }
}


void MX64_set_rad(int mx64_num, float rad) {
  float mx64_deg = rad * 180.0f / PI;
  mx64_deg = angle_to_bin(mx64_deg);
  dxl.goalPosition(mx64_num, mx64_deg);
}

void AK_motor_get_status(void) {
  if (CanBus.availableMessage())
  {
    if (CanBus.readMessage(&rx_msg))
    {
      Serial.print("EXT ID : 0x");
      Serial.print(rx_msg.id, HEX);
      Serial.print(", Len : ");
      Serial.print(rx_msg.length);
      Serial.print(", Data : ");
      for (int i = 0; i < rx_msg.length; i++)
      {
        if (rx_msg.data[i] < 0x10) Serial.print("0");
        Serial.print(rx_msg.data[i], HEX);
        Serial.print(" ");
      }

      int16_t value = ((int16_t)rx_msg.data[0] << 8) | rx_msg.data[1];
      Serial.print("Value (decimal): ");
      Serial.println(value / 10);
    }
  }
}

void AK_motor_set_rad(int can_num, float rad) {
  tx_msg.id = 0x00000600 + can_num;
  tx_msg.format = CAN_EXT_FORMAT; 

  float deg = rad * 180.0f / PI;

  int32_t AK_deg = (int32_t)(deg * 10000.0f);
  
  
  // Big Endian으로 바이트 분할
  tx_msg.data[0] = (AK_deg >> 24) & 0xFF;  // MSB
  tx_msg.data[1] = (AK_deg >> 16) & 0xFF;
  tx_msg.data[2] = (AK_deg >> 8)  & 0xFF;
  tx_msg.data[3] = (AK_deg)       & 0xFF;
  tx_msg.length  = 8;
  Serial.println("success");
    if (!CanBus.writeMessage(&tx_msg))
      Serial.println("TX fail");
  }
}

int angle_to_bin(float angle) {
  int bin = map(angle, -180.0, 180.0, 0, 4096);
  return bin;
}