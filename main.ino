#include <Usb.h>
#include <cdcacm.h>
#include <usbhub.h>
#include <PS2X_lib.h>

// ====================== USB Host / 馬達控制 ======================
USB Usb;

class ACMAsyncOper : public CDCAsyncOper {
public:
  uint8_t OnInit(ACM *pacm) override { return 0; }
};
ACMAsyncOper AsyncOper;
ACM Acm(&Usb, &AsyncOper);
bool deviceReady = false;

// 馬達 CAN SlaveID
const uint16_t MOTOR_SLAVE_ID1 = 0x01;
const uint16_t MOTOR_SLAVE_ID2 = 0x02;
const uint16_t MOTOR_SLAVE_ID3 = 0x03;
const uint16_t MOTOR_SLAVE_ID4 = 0x04;
// 速度模式 ID = 0x200 + SlaveID
const uint16_t MOTOR_ID_VEL_BASE = 0x200;
// 串口 921600, 8N1
const uint32_t UART_BAUD = 921600;
// RPM 上限（防呆）
const float RPM_MAX = 395.0f;

// send_data_frame 模板
uint8_t send_data_frame_template[30] = {
  0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

float rpm_to_rad(float rpm) {
  return rpm * 0.1047197551f;
}

void f32_to_bytes_le(float f, uint8_t out[4]) {
  union { float f; uint32_t u; } v;
  v.f = f;
  out[0] = (uint8_t)(v.u & 0xFF);
  out[1] = (uint8_t)((v.u >> 8) & 0xFF);
  out[2] = (uint8_t)((v.u >> 16) & 0xFF);
  out[3] = (uint8_t)((v.u >> 24) & 0xFF);
}

void usbSerialWrite(const uint8_t *buf, uint16_t len) {
  if (!deviceReady) return;
  Acm.SndData(len, const_cast<uint8_t*>(buf));
}

void sendFrameToMotor(uint16_t motor_id, const uint8_t data[8]) {
  if (!deviceReady) return;

  uint8_t frame[30];
  for (uint8_t i = 0; i < 30; i++) {
    frame[i] = send_data_frame_template[i];
  }

  // motor_id
  frame[13] = (uint8_t)(motor_id & 0xFF);
  frame[14] = (uint8_t)((motor_id >> 8) & 0xFF);

  // data[0..7]
  for (uint8_t i = 0; i < 8; i++) {
    frame[21 + i] = data[i];
  }

  usbSerialWrite(frame, 30);
}

void buildVelData(float rpm, uint8_t data[8]) {
  if (rpm >  RPM_MAX)  rpm =  RPM_MAX;
  if (rpm < -RPM_MAX)  rpm = -RPM_MAX;

  float rad_s = rpm_to_rad(rpm);
  uint8_t v[4];
  f32_to_bytes_le(rad_s, v);

  data[0] = v[0];
  data[1] = v[1];
  data[2] = v[2];
  data[3] = v[3];
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
}

void sendVelCommand(uint16_t slaveId, float rpm) {
  Usb.Task();               // 維持 USB Host 狀態
  if (!deviceReady) return;

  uint16_t motor_id = MOTOR_ID_VEL_BASE + slaveId;   // 0x200 + SlaveID
  uint8_t data[8];
  buildVelData(rpm, data);
  sendFrameToMotor(motor_id, data);
}

void send_Enable(uint16_t slaveId) {
  Usb.Task();
  if (!deviceReady) return;
  uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
  sendFrameToMotor(slaveId, data);
}

void send_disable(uint16_t slaveId) {
  Usb.Task();
  if (!deviceReady) return;
  uint8_t data[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
  sendFrameToMotor(slaveId, data);
}

void initMotor() {
  if (Usb.Init() == -1) {
    while (1) {}   // 失敗卡死
  }

  while (!Acm.isReady()) {
    Usb.Task();
  }
  deviceReady = true;

  LINE_CODING lc;
  lc.dwDTERate = UART_BAUD;
  lc.bCharFormat = 0;
  lc.bParityType = 0;
  lc.bDataBits   = 8;
  Acm.SetLineCoding(&lc);

  delay(100);
}

// ====================== PS2 控制器 ======================
#define PS2_DAT  4   // 任一數位腳，避開 10~13
#define PS2_CMD  5
#define PS2_SEL  6
#define PS2_CLK  7

#define pressures false
#define rumble    false

PS2X ps2x;
int  error = 0;
byte type  = 0;
byte vibrate = 0;

void initps2(){
  // 2. 初始化 PS2 控制器
  delay(300);
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  type  = ps2x.readType();

  if (error != 0) {
    Serial.println("PS2 controller init failed");
  } else {
    Serial.println("PS2 controller ready");
  }
}

void ws() {
  if (error != 0) {
    // 沒有偵測到手把就不要動馬達
    sendVelCommand(MOTOR_SLAVE_ID1, 0.0f);
    sendVelCommand(MOTOR_SLAVE_ID2, 0.0f);
    delay(100);
    return;
  }

  // 注意：read_gamepad 已經在 loop() 裡呼叫過了，這裡不要再呼叫

  // 左搖桿
  int ly = ps2x.Analog(PSS_LY);  // 左：前進/後退
  int lx = ps2x.Analog(PSS_LX);  // 左：左右旋轉
  // 右搖桿
  int ry = ps2x.Analog(PSS_RY);  // 右：前進/後退
  int rx = ps2x.Analog(PSS_RX);  // 右：左右旋轉

  const int center   = 128;
  const int deadzone = 5;

  // ------- 左搖桿的 fwdL / spinL -------
  int vyL = center - ly;      // 往上推 = 正值（前進）
  int vxL = lx - center;      // 往右推 = 正值（右轉）
  if (abs(vyL) < deadzone) vyL = 0;
  if (abs(vxL) < deadzone) vxL = 0;

  // ------- 右搖桿的 fwdR / spinR -------
  int vyR = center - ry;      // 往上推 = 正值（前進）
  int vxR = rx - center;      // 往右推 = 正值（右轉）
  if (abs(vyR) < deadzone) vyR = 0;
  if (abs(vxR) < deadzone) vxR = 0;

  // ------- 兩支搖桿數值疊加 -------
  int vy = vyL + vyR;         // 前進/後退總量
  int vx = vxL + vxR;         // 自轉總量

  // 限制在單搖桿範圍內，避免超過 127
  if (vy >  127) vy =  127;
  if (vy < -127) vy = -127;
  if (vx >  127) vx =  127;
  if (vx < -127) vx = -127;

  // ------- 映射到 -100 ~ +100 RPM -------
  float fwd  = (float)vy / 127.0f * 390.0f;  // 前進/後退分量
  float spin = (float)vx / 127.0f * 390.0f;  // 自轉分量

  // 差速混合
  float rpm1 = fwd + spin;
  float rpm2 = -fwd + spin;

  // 安全限制在 -100 ~ +100 RPM
  if (rpm1 >  100.0f) rpm1 =  390.0f;
  if (rpm1 < -100.0f) rpm1 = -390.0f;
  if (rpm2 >  100.0f) rpm2 =  390.0f;
  if (rpm2 < -100.0f) rpm2 = -390.0f;

  // 送給馬達
  sendVelCommand(MOTOR_SLAVE_ID1, rpm1);
  sendVelCommand(MOTOR_SLAVE_ID2, rpm2);

  // 除錯輸出
  Serial.print("LY="); Serial.print(ly);
  Serial.print(" LX="); Serial.print(lx);
  Serial.print("  RY="); Serial.print(ry);
  Serial.print(" RX="); Serial.print(rx);
  Serial.print("  rpm1="); Serial.print(rpm1);
  Serial.print(" rpm2="); Serial.println(rpm2);

  delay(20);  // 更新頻率約 50 Hz
}
// ====================== setup / loop ======================
void setup() {
  Serial.begin(115200);

  // 1. 初始化馬達 (USB-CAN)
  initMotor();
  send_Enable(MOTOR_SLAVE_ID1);   // 上電後使能一次即可
  send_Enable(MOTOR_SLAVE_ID2);

  // 2. 初始化 PS2 控制器
  initps2();
}

void loop() {

  // 讀取 PS2 狀態 (不啟動震動)
  ps2x.read_gamepad(false, 0);

  ws();

    if(ps2x.ButtonPressed(PSB_CIRCLE)){
      Serial.println("Circle just pressed");
      send_Enable(MOTOR_SLAVE_ID3);
      send_Enable(MOTOR_SLAVE_ID4);
      sendVelCommand(MOTOR_SLAVE_ID3, -100.0f);
      sendVelCommand(MOTOR_SLAVE_ID4, 100.0f);
    }
    if(ps2x.ButtonPressed(PSB_TRIANGLE)){
      Serial.println("Triangle pressed");
      send_disable(MOTOR_SLAVE_ID3);
      send_disable(MOTOR_SLAVE_ID4);
      sendVelCommand(MOTOR_SLAVE_ID3, 0.0f);
      sendVelCommand(MOTOR_SLAVE_ID4, 0.0f);
    }
}
