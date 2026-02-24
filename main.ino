/*

  請先安裝PS2X函式庫，網址：
  https://github.com/madsci1016/Arduino-PS2X
  
*/


#include "PS2X_lib.h"

PS2X ps2x; 
int error;



void initPS2(int Clock,int Command,int Attention,int Data){
  //配對接收器 
  do { 
    //GamePad(clock, command, attention, data, Pressures?, Rumble?)
    error = ps2x.config_gamepad(Clock, Command, Attention, Data, true, true);   //這行要和接線對應正確
    if (error == 0) { Serial.print("Gamepad found!");break; } 
    else { delay(100); } 
  } while (1); 
}


#include <Usb.h>
#include <cdcacm.h>
#include <usbhub.h>

// ============ USB Host / CDC 初始化 ============
USB Usb;
class ACMAsyncOper : public CDCAsyncOper {
public:
  uint8_t OnInit(ACM *pacm) override { return 0; }
};
ACMAsyncOper AsyncOper;
ACM Acm(&Usb, &AsyncOper);
bool deviceReady = false;

// ============ 馬達 / 協議參數 ============
// 馬達的 CAN SlaveID
const uint16_t MOTOR_SLAVE_ID1 = 0x01;
const uint16_t MOTOR_SLAVE_ID2 = 0x02;
const uint16_t MOTOR_SLAVE_ID3 = 0x03;
const uint16_t MOTOR_SLAVE_ID4 = 0x04;
// 速度模式下的 motor_id = 0x200 + SlaveID（對應 control_Vel）
const uint16_t MOTOR_ID_VEL_BASE = 0x200;
// 串口 921600, 8N1
const uint32_t UART_BAUD = 921600;
// RPM 上限（防呆）
const float RPM_MAX = 395.0f;

// ============ send_data_frame 模板（照 DM_CAN.MotorControl）===========
uint8_t send_data_frame_template[30] = {
  0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ============ 工具：RPM→rad/s、float→4bytes 小端 ============
float rpm_to_rad(float rpm) {
  return rpm * 0.1047197551f;   // m3519_run.py 同樣係數
}

void f32_to_bytes_le(float f, uint8_t out[4]) {
  union { float f; uint32_t u; } v;
  v.f = f;
  out[0] = (uint8_t)(v.u & 0xFF);
  out[1] = (uint8_t)((v.u >> 8) & 0xFF);
  out[2] = (uint8_t)((v.u >> 16) & 0xFF);
  out[3] = (uint8_t)((v.u >> 24) & 0xFF);
}

// ============ USB 串口 I/O 封裝 ============
void usbSerialWrite(const uint8_t *buf, uint16_t len) {
  if (!deviceReady) return;
  Acm.SndData(len, const_cast<uint8_t*>(buf));
}

// ============ 等價 Python 的 __send_data(motor_id, data) ============
void sendFrameToMotor(uint16_t motor_id, const uint8_t data[8]) {
  if (!deviceReady) return;

  uint8_t frame[30];
  for (uint8_t i = 0; i < 30; i++) {
    frame[i] = send_data_frame_template[i];
  }

  // send_data_frame[13/14] = motor_id
  frame[13] = (uint8_t)(motor_id & 0xFF);
  frame[14] = (uint8_t)((motor_id >> 8) & 0xFF);

  // send_data_frame[21:29] = data
  for (uint8_t i = 0; i < 8; i++) {
    frame[21 + i] = data[i];
  }

  usbSerialWrite(frame, 30);
}

// ============ 速度命令：等價 control_Vel + motor_run ============
void buildVelData(float rpm, uint8_t data[8]) {
  if (rpm >  RPM_MAX) rpm  =  RPM_MAX;
  if (rpm < -RPM_MAX) rpm  = -RPM_MAX;

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

// 傳入「電機 SlaveID」和「RPM」
void sendVelCommand(uint16_t slaveId, float rpm) {
  Usb.Task();               // 維持 USB Host 狀態
  if (!deviceReady) return;

  uint16_t motor_id = MOTOR_ID_VEL_BASE + slaveId;   // 0x200 + SlaveID

  uint8_t data[8];
  buildVelData(rpm, data);
  sendFrameToMotor(motor_id, data);
}

// ============ 使能與失能（等價 enable / disable） ============
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

// ============ 馬達與 USB 的初始化打包成一個函數 ============
void initMotor(uint16_t slaveId) {
  // 初始化 USB Host
  if (Usb.Init() == -1) {
    while (1) {}   // 失敗就卡死
  }

  // 等 USB‑CAN 枚舉完成（CDC ready）
  while (!Acm.isReady()) {
    Usb.Task();
  }
  deviceReady = true;

  // 設定虛擬串口 921600, 8N1
  LINE_CODING lc;
  lc.dwDTERate = UART_BAUD;
  lc.bCharFormat = 0;
  lc.bParityType = 0;
  lc.bDataBits   = 8;
  Acm.SetLineCoding(&lc);
s
  delay(100);                // 給 USB‑CAN / 馬達一點時間穩定
  send_Enable(slaveId);      // 上電後一次性使能
}



void setup() {
  Serial.begin(115200);
  //配對接收器 
  initPS2(13,11,10,12);
  initMotor(MOTOR_SLAVE_ID1);   // 所有馬達 setup 都包在這一行
  initMotor(MOTOR_SLAVE_ID2);
  initMotor(MOTOR_SLAVE_ID3);
  initMotor(MOTOR_SLAVE_ID4);
}
void loop(){
  ps2x.read_gamepad(false, 0);  //讀取手把狀態 

  int vel_motor_ID1 = map((ps2x.Analog(PSS_LY) + ps2x.Analog(PSS_LX)) + (ps2x.Analog(PSS_RY) + ps2x.Analog(PSS_RX)), 0, 255, -100, 100);
  int vel_motor_ID2 = map((ps2x.Analog(PSS_LY) - ps2x.Analog(PSS_LX)) + (ps2x.Analog(PSS_RY) - ps2x.Analog(PSS_RX)), 0, 255, -100, 100);

  sendVelCommand(MOTOR_SLAVE_ID1,vel_motor_ID1);
  sendVelCommand(MOTOR_SLAVE_ID2,vel_motor_ID1 * -1);

  
  //測試每一個按鈕和搖桿
    if(ps2x.Button(PSB_START))         //Start鍵
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))       //Select鍵
      Serial.println("Select is being held");      

    if(ps2x.Button(PSB_PAD_UP)) {      //十字方向，上
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){   //十字方向，右
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
    }
    if(ps2x.Button(PSB_PAD_LEFT)){    //十字方向，左
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
    }
    if(ps2x.Button(PSB_PAD_DOWN)){    //十字方向，下
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
    }
    if(ps2x.NewButtonState(PSB_L3))   //L3鍵，NewButtonState按下不管多久只會觸發兩次(按下和放開)
      Serial.println("L3 pressed");
    if(ps2x.NewButtonState(PSB_R3))   //R3鍵
      Serial.println("R3 pressed");
    if(ps2x.NewButtonState(PSB_L2))   //L2鍵
      Serial.println("L2 pressed");
    if(ps2x.NewButtonState(PSB_R2))   //R2鍵
      Serial.println("R2 pressed");
    if(ps2x.NewButtonState(PSB_TRIANGLE))  //三角按鍵
      Serial.println("Triangle pressed");        


    if(ps2x.NewButtonState(PSB_CIRCLE)) { //圓型按鍵
      Serial.println("Circle pressed");
      sendVelCommand(MOTOR_SLAVE_ID3,100.0f);
      sendVelCommand(MOTOR_SLAVE_ID4,-100.0f);
    } else{
      sendVelCommand(MOTOR_SLAVE_ID3,00.0f);
      sendVelCommand(MOTOR_SLAVE_ID4,00.0f);
    }


    if(ps2x.NewButtonState(PSB_CROSS))    //X按鍵
      Serial.println("X pressed");
    if(ps2x.NewButtonState(PSB_SQUARE))   //方型按鍵
      Serial.println("Square pressed");     

    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //按下L1或L2鍵，顯示兩個香菇頭的數值
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC);   //左，上下
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC);   //左，左右
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC);   //右，上下
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC); //右，左右
    }  


    
  delay(10); 
}
