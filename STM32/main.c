#include "main.h"
#include <math.h>
#include <string.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim4;

static constexpr float WHEEL_RADIUS_M   = 0.076f;
static constexpr float WHEEL_BASE_M     = 0.50f;
static constexpr float MAX_RPM          = 300.0f;
static constexpr float KP               = 0.35f;
static constexpr float KI               = 6.0f;
static constexpr float OUT_MIN_DUTY     = 0.0f;
static constexpr float OUT_MAX_DUTY     = 99.0f;
static constexpr float ODOM_PUB_HZ      = 50.0f;
static constexpr float CTRL_HZ          = 1000.0f;
static constexpr uint32_t TIMER_CLK_HZ  = 84000000;
static constexpr uint32_t POLE_PAIRS        = 7;
static constexpr uint32_t PULSES_PER_REV    = POLE_PAIRS * 3;

#define EN_L_GPIO_Port   GPIOB
#define EN_L_Pin         GPIO_PIN_0
#define ZF_L_GPIO_Port   GPIOB
#define ZF_L_Pin         GPIO_PIN_1
#define EN_R_GPIO_Port   GPIOB
#define EN_R_Pin         GPIO_PIN_2
#define ZF_R_GPIO_Port   GPIOB
#define ZF_R_Pin         GPIO_PIN_10

static inline float clampf(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi ? hi : v);
}

struct Motor {
  TIM_HandleTypeDef* tim;
  uint32_t pwm_channel;
  uint32_t ic_channel;
  GPIO_TypeDef* en_port;
  uint16_t     en_pin;
  GPIO_TypeDef* zf_port;
  uint16_t     zf_pin;
  volatile uint32_t last_capture = 0;
  volatile uint32_t period_ticks = 0;
  volatile bool     got_pulse    = false;
  float target_rpm = 0.0f;
  float meas_rpm   = 0.0f;
  float integ      = 0.0f;
  float duty       = 0.0f;

  void updateMeasuredRPM() {
    if (!got_pulse || period_ticks == 0) { meas_rpm = 0.0f; return; }
    uint32_t psc = tim->Instance->PSC;
    float timer_tick_hz = (float)TIMER_CLK_HZ / (float)(psc + 1);
    float freq_hz = timer_tick_hz / (float)period_ticks;
    float rev_per_sec = freq_hz / (float)PULSES_PER_REV;
    meas_rpm = rev_per_sec * 60.0f;
  }
  void setDirection(float rpm_cmd) {
    if (rpm_cmd >= 0.0f) HAL_GPIO_WritePin(zf_port, zf_pin, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(zf_port, zf_pin, GPIO_PIN_RESET);
  }
  void writeDuty(float duty_percent) {
    duty = clampf(duty_percent, OUT_MIN_DUTY, OUT_MAX_DUTY);
    uint32_t arr = tim->Instance->ARR;
    uint32_t ccr = (uint32_t)((duty / 100.0f) * (float)arr);
    __HAL_TIM_SET_COMPARE(tim, pwm_channel, ccr);
  }
  void stepPI(float dt) {
    updateMeasuredRPM();
    float cmd = target_rpm;
    setDirection(cmd);
    float spd_err = fabsf(cmd) - fabsf(meas_rpm);
    float up = KP * spd_err;
    integ += KI * spd_err * dt;
    integ = clampf(integ, -50.0f, 50.0f);
    float u = up + integ;
    if (fabsf(cmd) < 1.0f && meas_rpm < 1.0f) { u = 0.0f; integ = 0.0f; }
    float duty_out = u;
    writeDuty(duty_out);
  }
  void enable(bool en) {
    HAL_GPIO_WritePin(en_port, en_pin, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
};

Motor motorL { &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, EN_L_GPIO_Port, EN_L_Pin, ZF_L_GPIO_Port, ZF_L_Pin };
Motor motorR { &htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, EN_R_GPIO_Port, EN_R_Pin, ZF_R_GPIO_Port, ZF_R_Pin };

ros::NodeHandle nh;
geometry_msgs::Twist cmd_vel_msg;
volatile float cmd_v = 0.0f;
volatile float cmd_w = 0.0f;

void cmdVelCb(const geometry_msgs::Twist& msg) { cmd_v = msg.linear.x; cmd_w = msg.angular.z; }
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", cmdVelCb);
nav_msgs::Odometry odom_msg;
ros::Publisher pub_odom("odom", &odom_msg);

float odom_x = 0.0f, odom_y = 0.0f, odom_th = 0.0f;

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == motorL.tim->Instance && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    uint32_t cap = HAL_TIM_ReadCapturedValue(htim, motorL.ic_channel);
    static uint32_t prev = 0;
    motorL.period_ticks = (cap >= prev) ? (cap - prev) : (htim->Instance->ARR - prev + cap + 1);
    motorL.last_capture = cap;
    prev = cap;
    motorL.got_pulse = true;
  } else if (htim->Instance == motorR.tim->Instance && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    uint32_t cap = HAL_TIM_ReadCapturedValue(htim, motorR.ic_channel);
    static uint32_t prev = 0;
    motorR.period_ticks = (cap >= prev) ? (cap - prev) : (htim->Instance->ARR - prev + cap + 1);
    motorR.last_capture = cap;
    prev = cap;
    motorR.got_pulse = true;
  }
}

static inline float rpm_from_ms(float v) {
  float rps = v / (2.0f * (float)M_PI * WHEEL_RADIUS_M;
  return rps * 60.0f;
}

static inline void diff_kinematics_to_wheels(float v, float w, float &rpmL, float &rpmR) {
  float vL = v - (w * WHEEL_BASE_M * 0.5f);
  float vR = v + (w * WHEEL_BASE_M * 0.5f);
  rpmL = rpm_from_ms(vL);
  rpmR = rpm_from_ms(vR);
  rpmL = clampf(rpmL, -MAX_RPM, MAX_RPM);
  rpmR = clampf(rpmR, -MAX_RPM, MAX_RPM);
}

static inline void wheels_to_body(float rpmL, float rpmR, float &v, float &w) {
  float vL = (rpmL / 60.0f) * (2.0f * (float)M_PI * WHEEL_RADIUS_M);
  float vR = (rpmR / 60.0f) * (2.0f * (float)M_PI * WHEEL_RADIUS_M);
  v = 0.5f * (vL + vR);
  w = (vR - vL) / WHEEL_BASE_M;
}

extern "C" void SystemClock_Config(void);
extern "C" void MX_GPIO_Init(void);
extern "C" void MX_USART3_UART_Init(void);
extern "C" void MX_TIM3_Init(void);
extern "C" void MX_TIM4_Init(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  HAL_TIM_PWM_Start(motorL.tim, motorL.pwm_channel);
  HAL_TIM_PWM_Start(motorR.tim, motorR.pwm_channel);
  HAL_TIM_IC_Start_IT(motorL.tim, motorL.ic_channel);
  HAL_TIM_IC_Start_IT(motorR.tim, motorR.ic_channel);
  motorL.enable(true);
  motorR.enable(true);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  nh.advertise(pub_odom);
  const uint32_t ctrl_dt_ms  = (uint32_t)(1000.0f / CTRL_HZ);
  const uint32_t odom_dt_ms  = (uint32_t)(1000.0f / ODOM_PUB_HZ);
  uint32_t t_prev_ctrl = HAL_GetTick();
  uint32_t t_prev_odom = HAL_GetTick();
  while (1) {
    uint32_t tnow = HAL_GetTick();
    if ((tnow - t_prev_ctrl) >= ctrl_dt_ms) {
      t_prev_ctrl = tnow;
      float rpmL_cmd, rpmR_cmd;
      diff_kinematics_to_wheels(cmd_v, cmd_w, rpmL_cmd, rpmR_cmd);
      motorL.target_rpm = rpmL_cmd;
      motorR.target_rpm = rpmR_cmd;
      motorL.stepPI(1.0f / CTRL_HZ);
      motorR.stepPI(1.0f / CTRL_HZ);
    }
    if ((tnow - t_prev_odom) >= odom_dt_ms) {
      float v_meas, w_meas;
      wheels_to_body(motorL.meas_rpm, motorR.meas_rpm, v_meas, w_meas);
      float dt = (tnow - t_prev_odom) / 1000.0f;
      t_prev_odom = tnow;
      odom_th += w_meas * dt;
      float cosT = cosf(odom_th);
      float sinT = sinf(odom_th);
      odom_x += v_meas * cosT * dt;
      odom_y += v_meas * sinT * dt;
      float half_yaw = 0.5f * odom_th;
      float qz = sinf(half_yaw);
      float qw = cosf(half_yaw);
      odom_msg.header.stamp = nh.now();
      odom_msg.header.frame_id = (char*)"odom";
      odom_msg.child_frame_id  = (char*)"base_link";
      odom_msg.pose.pose.position.x = odom_x;
      odom_msg.pose.pose.position.y = odom_y;
      odom_msg.pose.pose.position.z = 0.0f;
      odom_msg.pose.pose.orientation.x = 0.0f;
      odom_msg.pose.pose.orientation.y = 0.0f;
      odom_msg.pose.pose.orientation.z = qz;
      odom_msg.pose.pose.orientation.w = qw;
      odom_msg.twist.twist.linear.x  = v_meas;
      odom_msg.twist.twist.linear.y  = 0.0f;
      odom_msg.twist.twist.angular.z = w_meas;
      pub_odom.publish(&odom_msg);
    }
    nh.spinOnce();
  }
}

extern "C" void SystemClock_Config(void) { }
extern "C" void MX_GPIO_Init(void)       { }
extern "C" void MX_USART3_UART_Init(void){ }
extern "C" void MX_TIM3_Init(void)       { }
extern "C" void MX_TIM4_Init(void)       { }
