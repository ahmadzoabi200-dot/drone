# quad_drone

Open-loop ESP32 firmware that reads an Xbox controller over BLE and drives four
brushless-motor ESCs with X-quadcopter motor mixing (throttle + roll/pitch/yaw).
It is a controller → ESC → motor signal-chain and motor-mixing project for
learning and bench use. **It is not flight firmware.**

---

> ## ⚠️ SAFETY — READ FIRST
> - **Props OFF for all bench testing.** Always. Spinning motors with props
>   attached while testing this code can injure you.
> - **Verify motor positions and spin directions before attaching props.**
>   Confirm each motor sits where the mix expects (FL/FR/RL/RR) and spins the
>   correct way; flip the mixing signs in the code to match your frame first.
> - This is open-loop firmware with no stabilization — see
>   ["What it does NOT do"](#what-it-does-not-do).

---

## What it does

On boot the firmware:

1. Opens serial at **115200** baud.
2. Allocates 4 ESP32 PWM timers and attaches 4 ESC outputs at **50 Hz**, each
   configured for a **1000–2000 µs** pulse range.
3. Arms the ESCs by holding all four motors at minimum throttle (1000 µs) for
   **3 seconds**.
4. Starts BLE and waits for an Xbox controller in pairing mode.

Once a controller is connected:

- **A** arms the motors; **B** disarms (emergency stop — B always wins over A).
- While **armed**, every loop (~50 Hz) it reads the inputs, applies a 0.08
  deadzone to roll/pitch/yaw, computes per-motor outputs with **X-quad mixing**,
  constrains each to 1000–2000 µs, writes them to the four ESCs, and prints the
  values over serial.
- While **disarmed**, all motors are held at idle (1000 µs).

The X-quad mix (per motor, before clamping):

```
base = 1000 + throttle * 900            // 1000..1900 µs

FL = base + pitch + roll - yaw
FR = base + pitch - roll + yaw
RL = base - pitch + roll + yaw
RR = base - pitch - roll - yaw
```

Gains at full stick: roll 150 µs, pitch 150 µs, yaw 100 µs.

### Failsafe

If the controller disconnects, the firmware forces all motors to idle (1000 µs)
and clears the armed state, so a dropped link cannot leave the motors running.

## What it does NOT do

This is stated plainly on purpose — knowing the limits is part of using it safely:

- **No IMU** — there is no gyro and no accelerometer. No sensor feedback of any kind.
- **No PID, no stabilization** — it is fully open-loop. Stick input maps straight
  to motor output; nothing corrects drift, tilt, or imbalance.
- **It will not hover, self-level, or fly stably on its own.** A real quad needs
  an IMU + a stabilization loop (e.g. MPU6050 + PID). This firmware has neither.
- **Not autonomous** — no altitude hold, no GPS, no position hold, no return-to-home.

## Hardware required

- ESP32 dev board (with BLE).
- 4 × brushless motors.
- 4 × ESCs that accept a standard 1000–2000 µs / 50 Hz servo-style signal.
- A power source for the motors/ESCs, plus power for the ESP32.
- An Xbox Wireless Controller that supports **Bluetooth (BLE)**.
- An X-configuration quad frame to mount the motors (my FreeCAD models live in
  [`hardware/`](hardware/)).

Wire each ESC's signal line to its GPIO below and share a common ground between
the ESP32 and the ESCs.

## Wiring / pinout

X-configuration, viewed from the top (front at the top):

| Motor | Position    | ESP32 GPIO |
|-------|-------------|------------|
| FL    | Front-left  | GPIO 5     |
| FR    | Front-right | GPIO 18    |
| RL    | Rear-left   | GPIO 19    |
| RR    | Rear-right  | GPIO 23    |

## Controls

| Input                | Function                              |
|----------------------|---------------------------------------|
| Left stick X         | Roll                                  |
| Left stick Y         | Pitch                                 |
| Right stick X        | Yaw                                   |
| Right trigger (RT)   | Throttle                              |
| A button             | Arm                                   |
| B button             | Disarm / emergency stop (overrides A) |

Roll, pitch and yaw use a 0.08 deadzone around center; throttle does not.

## Dependencies

Install both libraries through the Arduino Library Manager — **do not** copy their
source into this repo. Neither is mine.

- **ESP32Servo** — generates the 50 Hz ESC signal on all four pins using the
  ESP32's hardware PWM (LEDC) peripheral. Available in the Arduino Library
  Manager ([source](https://github.com/madhephaestus/ESP32Servo)).
- **BLE-Gamepad-Client** (header `BLEGamepadClient.h`) by Tomasz Bekas — the BLE
  Xbox controller client. Third-party, **not mine**, required separately.
  Source: <https://github.com/tbekas/BLE-Gamepad-Client> (Apache-2.0). It depends
  on **NimBLE-Arduino**, which the Library Manager will offer to install with it.

You also need the **ESP32 board package** (Espressif) installed in the Arduino IDE.

## Build & flash

1. Install the **Arduino IDE** and add the **ESP32 board package** via Boards
   Manager (search "esp32", by Espressif Systems).
2. In the Library Manager, install **ESP32Servo** and **BLE-Gamepad-Client**
   (accept the NimBLE-Arduino dependency when prompted).
3. Open `quad_drone/quad_drone.ino` in the Arduino IDE. The sketch lives in a
   `quad_drone/` folder so the IDE opens it directly.
4. Select your ESP32 board and serial port, then flash.
5. Open the Serial Monitor at **115200** baud.
6. **Props off.** Power the ESCs and wait for the 3-second arming. Put the Xbox
   controller into pairing mode; once it connects, press **A** to arm and use the
   right trigger for throttle.

## Known limitations

These are understood trade-offs, and the natural next steps for the project:

- **(a) Open-loop, no stabilization.** There is no IMU or PID loop, so the
  firmware cannot hold attitude or correct drift. Adding an IMU + PID stabilization
  loop is the main next step.
- **(b) Mixing saturates near full throttle.** Base throttle can reach 1900 µs,
  but the roll/pitch/yaw terms can add up to ~400 µs more. Once a motor output
  hits the 1000/2000 µs clamp, control authority on that axis is reduced at the
  extremes.
- **(c) No throttle-at-idle interlock before arming.** Pressing A arms regardless
  of the trigger position, so a non-zero throttle takes effect immediately on arm.
  A "throttle must be near zero to arm" check would make this safer.
- **(d) Blocking ~50 Hz loop.** The loop uses `delay()` and a blocking 3-second
  arming wait rather than non-blocking timing. Fine for bench learning, not for a
  real flight control loop.

