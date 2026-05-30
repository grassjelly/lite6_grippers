#include <SCServo.h>
#include "M5Atom.h"
#include <ModbusRTU.h>

#define TX_UART 33
#define RX_UART 23

// modbus-rtu setup
#define RX_MODBUS 32
#define TX_MODBUS 26
#define SLAVE_ID         1
#define REG_GRIPPER_POS  128
#define REG_READ_GRIPPER_POS 257

#define LITE6_OUTPUT0 21
#define LITE6_OUTPUT1 22

#define GRIPPER_OPEN  0
#define GRIPPER_CLOSE 512
#define SPEED         768

uint16_t gripper_pos = 0;

SCSCL scscl;
ModbusRTU mb;

void setGripperPos(uint16_t pos) {
  scscl.WritePos(1, pos, 0, SPEED);
  uint8_t r = map(constrain(pos, GRIPPER_OPEN, GRIPPER_CLOSE), GRIPPER_OPEN, GRIPPER_CLOSE, 0, 255);
  M5.dis.drawpix(0, CRGB(r, 0, 0));
  Serial.printf("pos: %d\n", pos);
}

void setup() {
  pinMode(LITE6_OUTPUT0, INPUT_PULLUP);
  pinMode(LITE6_OUTPUT1, INPUT_PULLUP);

  M5.begin(true, false, true);
  Serial.begin(115200);

  Serial1.begin(1000000, SERIAL_8N1, RX_UART, TX_UART);
  scscl.pSerial = &Serial1;

  Serial2.begin(115200, SERIAL_8N1, RX_MODBUS, TX_MODBUS);
  mb.begin(&Serial2);
  mb.slave(SLAVE_ID);
  mb.addHreg(REG_GRIPPER_POS);
  mb.Hreg(REG_GRIPPER_POS, gripper_pos);
  mb.addIreg(REG_READ_GRIPPER_POS, gripper_pos);

  delay(1000);
  setGripperPos(gripper_pos);
}

void loop() {
  int lite6_0 = digitalRead(LITE6_OUTPUT0);
  int lite6_1 = digitalRead(LITE6_OUTPUT1);
  static int last_0 = -1;
  static int last_1 = -1;

  bool state_changed = (lite6_0 != last_0 || lite6_1 != last_1);
  last_0 = lite6_0;
  last_1 = lite6_1;

  if (state_changed) {
    if (lite6_0 == LOW && lite6_1 == HIGH) {
      setGripperPos(GRIPPER_OPEN);
    } else if (lite6_0 == HIGH && lite6_1 == LOW) {
      setGripperPos(GRIPPER_CLOSE);
    }
  }

  if (lite6_0 == HIGH && lite6_1 == HIGH) {
    uint16_t new_pos = mb.Hreg(REG_GRIPPER_POS);
    if (new_pos != gripper_pos) {
      gripper_pos = new_pos;
      setGripperPos(gripper_pos);
    }
  }

  mb.task();
  yield();
}
