/*
 * quad_drone.ino  —  Xbox BLE controller -> 4 brushless motors (ESP32)
 *
 * Reads an Xbox controller over BLE and drives four brushless-motor ESCs
 * with a 1000-2000us / 50Hz signal using X-quadcopter motor mixing.
 *
 * Controls:
 *   Left stick      -> direction   (X = roll left/right, Y = pitch fwd/back)
 *   Right trigger   -> throttle    (up / down)
 *   Right stick X   -> yaw          (rotate, optional bonus)
 *   A button        -> ARM   (motors enabled)
 *   B button        -> DISARM / emergency stop (motors forced to idle)
 *
 * ================== READ THIS BEFORE YOU FLY ==================
 * 1) NO PROPS for first tests. Bench-test motor response with props OFF.
 * 2) This is an OPEN-LOOP controller. There is NO gyro/accelerometer and
 *    NO PID stabilization, so it will NOT hover or fly stably on its own —
 *    a real quad needs an IMU + PID loop (e.g. MPU6050). This sketch is for
 *    learning the controller->ESC->motor signal chain and motor mixing.
 * 3) Verify motor POSITIONS and SPIN DIRECTIONS, and flip the mixing signs
 *    below to match your frame before ever attaching props.
 * =============================================================
 *
 * Library needed (install via Arduino Library Manager):
 *   - ESP32Servo      (generates clean 50Hz ESC signals on all 4 pins)
 *   - BLEGamepadClient (third-party Xbox BLE controller client; see README)
 */

#include <Arduino.h>
#include <BLEGamepadClient.h>   // XboxController / XboxControlsEvent
#include <ESP32Servo.h>         // ESC PWM generation

// ---------- ESC signal pins (one per motor) ----------
// X-configuration, viewed from the top:
//
//        FRONT
//     FL       FR
//        \   /
//         \ /
//         / \
//        /   \
//     RL       RR
//         REAR
const int PIN_FL = 5;    // front-left
const int PIN_FR = 18;   // front-right
const int PIN_RL = 19;   // rear-left
const int PIN_RR = 23;   // rear-right

Servo escFL, escFR, escRL, escRR;

// ---------- ESC pulse range (microseconds) ----------
const int PULSE_MIN = 1000;   // motor stopped / idle
const int PULSE_MAX = 2000;   // full throttle

// ---------- Control authority (us) at full stick deflection ----------
// How much roll/pitch/yaw can move a single motor away from base throttle.
// Keep these modest — bigger = more aggressive (and twitchier).
const float ROLL_GAIN  = 150.0;
const float PITCH_GAIN = 150.0;
const float YAW_GAIN   = 100.0;

// Ignore tiny stick noise around center.
const float DEADZONE = 0.08;

XboxController controller;
bool wasConnected = false;
bool armed = false;

// ----- helpers -----
float applyDeadzone(float v) {
  if (fabs(v) < DEADZONE) return 0.0f;
  return v;
}

void writeMotor(Servo &esc, float us) {
  int p = (int)constrain(us, (float)PULSE_MIN, (float)PULSE_MAX);
  esc.writeMicroseconds(p);
}

void allMotorsIdle() {
  escFL.writeMicroseconds(PULSE_MIN);
  escFR.writeMicroseconds(PULSE_MIN);
  escRL.writeMicroseconds(PULSE_MIN);
  escRR.writeMicroseconds(PULSE_MIN);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Quad drone controller starting — PROPS OFF!");

  // ESP32Servo needs timers allocated before attaching.
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  escFL.setPeriodHertz(50);  escFL.attach(PIN_FL, PULSE_MIN, PULSE_MAX);
  escFR.setPeriodHertz(50);  escFR.attach(PIN_FR, PULSE_MIN, PULSE_MAX);
  escRL.setPeriodHertz(50);  escRL.attach(PIN_RL, PULSE_MIN, PULSE_MAX);
  escRR.setPeriodHertz(50);  escRR.attach(PIN_RR, PULSE_MIN, PULSE_MAX);

  // Arm the ESCs: hold minimum throttle for 3 seconds.
  Serial.println("Arming ESCs (min throttle for 3s)...");
  unsigned long start = millis();
  while (millis() - start < 3000) {
    allMotorsIdle();
    delay(20);
  }
  Serial.println("ESCs armed. Connect Xbox controller (pairing mode).");

  controller.begin();
}

void loop() {
  if (!controller.isConnected()) {
    // FAILSAFE: no controller -> motors off.
    if (wasConnected) {
      Serial.println("Controller disconnected — motors idle.");
      wasConnected = false;
      armed = false;
    }
    allMotorsIdle();
    delay(20);
    return;
  }

  if (!wasConnected) {
    Serial.println("Controller connected. Press A to ARM, B to DISARM.");
    wasConnected = true;
  }

  XboxControlsEvent e;
  controller.read(&e);

  // ----- arm / disarm -----
  if (e.buttonA) armed = true;
  if (e.buttonB) armed = false;   // emergency stop wins

  if (!armed) {
    allMotorsIdle();
    delay(20);
    return;
  }

  // ----- read inputs -----
  float throttle = constrain(e.rightTrigger, 0.0f, 1.0f);  // RT: up/down
  float roll  = applyDeadzone(e.leftStickX);               // left stick X
  float pitch = applyDeadzone(e.leftStickY);               // left stick Y
  float yaw   = applyDeadzone(e.rightStickX);              // right stick X

  // Base throttle: 1000us (off) .. 1900us, leaving headroom for mixing.
  float base = PULSE_MIN + throttle * 900.0f;

  // ----- X-quad motor mixing -----
  // If a control moves the wrong way, flip the sign of its term below.
  // Yaw: diagonal motor pairs spin opposite directions
  //   pair A = FL + RR,  pair B = FR + RL
  float fl = base + pitch * PITCH_GAIN + roll * ROLL_GAIN - yaw * YAW_GAIN;
  float fr = base + pitch * PITCH_GAIN - roll * ROLL_GAIN + yaw * YAW_GAIN;
  float rl = base - pitch * PITCH_GAIN + roll * ROLL_GAIN + yaw * YAW_GAIN;
  float rr = base - pitch * PITCH_GAIN - roll * ROLL_GAIN - yaw * YAW_GAIN;

  writeMotor(escFL, fl);
  writeMotor(escFR, fr);
  writeMotor(escRL, rl);
  writeMotor(escRR, rr);

  // ----- debug -----
  Serial.print("ARMED  T:"); Serial.print(throttle, 2);
  Serial.print(" R:");  Serial.print(roll, 2);
  Serial.print(" P:");  Serial.print(pitch, 2);
  Serial.print(" Y:");  Serial.print(yaw, 2);
  Serial.print("  us[FL FR RL RR]: ");
  Serial.print((int)fl); Serial.print(' ');
  Serial.print((int)fr); Serial.print(' ');
  Serial.print((int)rl); Serial.print(' ');
  Serial.println((int)rr);

  delay(20);   // ~50 Hz update
}
