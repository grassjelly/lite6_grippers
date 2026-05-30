#include <SCServo.h>
#include "M5Atom.h"

#define TX_UART 33
#define RX_UART 23

#define SERVO_ID      1
#define GRIPPER_OPEN  0
#define GRIPPER_CLOSE 512
#define SPEED         500

SCSCL scscl;

void setup() {
  M5.begin(true, false, true);
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, RX_UART, TX_UART);
  scscl.pSerial = &Serial1;
  delay(1000);
}

void loop() {
  scscl.WritePos(SERVO_ID, GRIPPER_OPEN, 0, SPEED);
  Serial.println("open");
  delay(3000);

  scscl.WritePos(SERVO_ID, GRIPPER_CLOSE, 0, SPEED);
  Serial.println("close");
  delay(3000);
}
