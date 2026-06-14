# Gripper Firmware

Firmware for an M5Stack Atom Lite controlling a Feetech SCS15 servo gripper over UART, with Modbus RTU and digital I/O control modes.

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | M5Stack Atom Lite (ESP32-PICO-D4) |
| Servo | Feetech SCS15 (SCS-series, TTL UART) |
| Protocol | Half-duplex UART @ 1 Mbps |

### Wiring Diagram

[![Wiring Diagram](https://github.com/hygradme/OpenParallelGripper/blob/main/SCS3045M_version/electronics/wiring_summary.jpg)](https://github.com/hygradme/OpenParallelGripper/blob/main/SCS3045M_version/electronics/wiring_summary.jpg)

### Pin Map

| Pin | Function |
|---|---|
| 33 | Servo TX (Serial1) |
| 23 | Servo RX (Serial1) |
| 26 | Modbus TX (Serial2) |
| 32 | Modbus RX (Serial2) |
| 21 | Digital input 0 (LITE6_OUTPUT0) |
| 22 | Digital input 1 (LITE6_OUTPUT1) |

### Digital I/O Control

| OUTPUT0 | OUTPUT1 | Action |
|---|---|---|
| LOW | HIGH | Open gripper |
| HIGH | LOW | Close gripper |
| HIGH | HIGH | Modbus RTU control |

---

## Setup

### 1. Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 2. Install PlatformIO

```bash
uv tool install platformio --with pip
```

Verify:

```bash
pio --version
```

### 3. USB port permissions

```bash
sudo usermod -aG dialout $USER
```

Log out and back in, or apply immediately:

```bash
newgrp dialout
```

If you need a one-time fix without re-logging:

```bash
sudo chmod a+rw /dev/ttyUSB0
```

---

## Build & Upload

```bash
cd gripper_firmware
pio run --target upload
```

### Monitor serial output

```bash
pio device monitor
```

Output example:
```
pos: 0
pos: 512
```

---

## Modbus RTU

| Register | Type | Address | Description |
|---|---|---|---|
| `REG_GRIPPER_POS` | Holding (RW) | 128 | Target position (0–512) |
| `REG_READ_GRIPPER_POS` | Input (R) | 257 | Last written position |

- Baud rate: `115200`
- Mode: RTU, slave ID `1`
- Active when both digital inputs are HIGH (idle/floating)

---

## Libraries

| Library | Purpose |
|---|---|
| `ftservo/FTServo` | Feetech SCS/STS servo control |
| `m5stack/M5Atom` | M5Atom Lite BSP + LED |
| `fastled/FastLED` | RGB LED (dependency of M5Atom) |
| `emelianov/modbus-esp8266` | Modbus RTU slave |

---

## Python Control

`control.py` drives the gripper from a host PC via the xArm Python SDK over Ethernet.

### Install

```bash
pip install xarm-python-sdk
```

### Usage

```bash
python3 control.py <robot_ip> [actions]
```

`actions` is a string of characters executed in order, then repeated:

| Character | Action |
|---|---|
| `o` | Open gripper |
| `c` | Close gripper |
| `s` | Sleep 1 s |
| `0`–`9` | Move to that tenth of full range |

Examples:

```bash
# Open then close, repeat
python3 control.py 192.168.1.151 oc

# Open, wait, close
python3 control.py 192.168.1.151 oso
```

Position is reported in meters (`0.0` = closed, `0.025` = fully open).

---

## Credits

Firmware and wiring diagram adapted from [hygradme/OpenParallelGripper](https://github.com/hygradme/OpenParallelGripper/blob/main/SCS3045M_version/software/arduino_sketch/ModbusRTU_SCSServo.ino).

### Changes from original

- **SCS-series support**: migrated from `SMS_STS` (STS-series) to `SCSCL` and `WritePos` to correctly target the SCS15 servo.
- **UC hang fix**: the original firmware called `setGripperPos` on every loop iteration while a digital input was held, flooding the servo with commands and causing the microcontroller to hang. Fixed by only sending a command on pin state change.
- **Speed cap**: values above `768` have been observed to make the gripper non-reactive. This is likely due to the inrush current at higher speeds exceeding what the robot arm's power output can supply. Do not increase `SPEED` beyond `768`.
