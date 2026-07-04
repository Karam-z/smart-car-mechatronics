/*
 * ESP32 Vehicle Control & Telemetry Firmware — Code Excerpts
 * =========================================================
 * Smart Car & Driver Monitoring platform — Mechatronics subsystem
 *
 * Target : ESP32 DOIT DevKit V1 (ESP32-D0WD-V3), 240 MHz dual-core, 520 KB SRAM
 * Toolkit: C++ / Arduino
 * Libs   : XboxSeriesXControllerESP32_asukiaaa (BLE gamepad)
 *          MPU6050_tockn (IMU + complementary filter)
 *          ESP32 LEDC peripheral (50 Hz, 16-bit PWM on ESC + servo)
 *
 * NOTE: This file collects the firmware code exactly as documented in the
 *       project report (Sections 3.2, 4.2 and 4.4). It is a faithful set of
 *       excerpts — the per-module functions the report presents — rather than
 *       a single continuously compilable sketch. Global declarations, pin
 *       #defines and the setup()/loop() skeleton are summarised in the
 *       "Pin map & constants" and "Main loop structure" comment blocks so the
 *       excerpts read in context.
 */

/* =========================================================================
 * PIN MAP  (ESP32 DOIT DevKit V1 — fixed, no conflict with BLE/I2C/LEDC)
 * -------------------------------------------------------------------------
 *   SERVO_PIN  15   MG995 steering servo   (PWM, 50 Hz LEDC)
 *   ESC_PIN    12   60 A ESC signal        (PWM, 50 Hz LEDC)
 *   VPIN       34   Battery voltage sense  (ADC input, 12-bit, input-only)
 *   SDA_PIN    21   I2C data  — MPU6050
 *   SCL_PIN    22   I2C clock — MPU6050
 *   HALL_PIN   23   US1881 Hall-effect interrupt (INPUT_PULLUP, CHANGE mode)
 *
 * PWM: 50 Hz => 20,000 us frame. MAX_DUTY = 2^16 - 1 = 65535 (sub-us res).
 * ========================================================================= */


/* -------------------------------------------------------------------------
 * PWM OUTPUT — duty-cycle conversion (servo + ESC share the same formula)
 * ------------------------------------------------------------------------- */
// 16-bit duty-cycle from pulse-width in microseconds
uint32_t usToDuty(int us) {
    return (uint32_t)((float)us / 20000.0f * MAX_DUTY);
}
void writeServoUs(int us) {
    lastServoUs = us;
    ledcWrite(SERVO_PIN, usToDuty(us));
}
void writeEscUs(int us) {
    ledcWrite(ESC_PIN, usToDuty(us));
}


/* -------------------------------------------------------------------------
 * ESC ARMING — hold NEUTRAL for 3 s to satisfy power-on neutral detection
 * ------------------------------------------------------------------------- */
void armESC() {
  if (DEBUG_HUMAN) Serial.println("Arming ESC at NEUTRAL...");
  writeEscUs(ESC_NEUTRAL_US);
  delay(3000);
  if (DEBUG_HUMAN) Serial.println("ESC armed.");
}


/* -------------------------------------------------------------------------
 * THROTTLE / BRAKE / REVERSE decision (single-click forward/reverse ESC)
 *   CASE 1 (LT): proportional reverse/brake, applied INSTANTLY (bypass ramp)
 *   CASE 2 (RT): forward, smoothly ramped
 *   CASE 3     : coast to neutral
 * ------------------------------------------------------------------------- */
int decideThrottle(int rt, int lt, int *stepOut) {
    // CASE 1 — REVERSE / BRAKE (LT): proportional, applied instantly.
    // Brake depth scales with trigger pressure; no dependency on
    // currentThrottle so it bites even when the command has already
    // coasted to neutral while the car is still rolling forward.
    if (lt > TRIG_DEAD_ZONE) {
        int rev = map(lt, TRIG_DEAD_ZONE, TRIG_MAX,
                      ESC_REV_SPIN_US, ESC_REV_MAX_US);
        currentThrottle = rev;   // snap — bypass the ramp
        *stepOut = REV_ACCEL_STEP;
        return rev;
    }
    // CASE 2 — FORWARD (RT): smoothly ramped.
    if (rt > TRIG_DEAD_ZONE) {
        int target = map(rt, TRIG_DEAD_ZONE, TRIG_MAX,
                         ESC_FWD_SPIN_US, ESC_FWD_MAX_US);
        if (currentThrottle < ESC_NEUTRAL_US) {
            *stepOut = BRAKE_STEP;   // still rolling backward -> brake up
        } else {
            *stepOut = ACCEL_STEP;
            if (currentThrottle < ESC_FWD_SPIN_US)   // kick past dead zone
                currentThrottle = ESC_FWD_SPIN_US;
        }
        return target;
    }
    // CASE 3 — COAST: release to neutral at COAST_STEP.
    *stepOut = COAST_STEP;
    return ESC_NEUTRAL_US;
}


/* -------------------------------------------------------------------------
 * STEERING — direct joystick mode
 *   Left stick horizontal (0..65535) -> [SERVO_MIN_US, SERVO_MAX_US]
 *   STEER_REVERSED corrects the physical servo-horn mounting orientation.
 * ------------------------------------------------------------------------- */
void controlSteering(int rt, int lt) {
    // Direct joystick mode
    uint16_t raw = xboxController.xboxNotif.joyLHori;  // 0..65535
    int us = STEER_REVERSED
           ? map(raw, 0, 65535, SERVO_MAX_US, SERVO_MIN_US)
           : map(raw, 0, 65535, SERVO_MIN_US, SERVO_MAX_US);
    writeServoUs(us);
}


/* -------------------------------------------------------------------------
 * STEERING — closed-loop heading assist (proportional controller)
 *   Toggled by LB during forward driving. Holds a target heading from IMU
 *   yaw, updated by the joystick at a commanded turn rate.
 *   Kp = 12.0, wmax = 90 deg/s, PW_center = 1472 us, delta_u_max = 350.
 *   NOTE: prototyped and validated, later removed when LB was repurposed
 *         for the left turn-signal LED.
 * ------------------------------------------------------------------------- */
// Heading-assist P-controller (toggled by LB button)
void controlSteering(int rt, int lt) {
    if (millis() - lastCtrl < CTRL_INTERVAL) return;
    lastCtrl = millis();
    const float dt = CTRL_INTERVAL / 1000.0f;
    bool fwdDriving = (rt > TRIG_DEAD_ZONE) && (lt <= TRIG_DEAD_ZONE);
    bool useAssist  = assistOn && readyToDrive && fwdDriving;
    if (useAssist) {
        if (!headingActive) { targetHeading = carYaw; headingActive = true; }
        float stickNorm = ((int)xboxController.xboxNotif.joyLHori - 32768) / 32768.0f;
        if (fabs(stickNorm) < STICK_DEADBAND) stickNorm = 0;
        targetHeading += TURN_CMD_SIGN * stickNorm * MAX_TURN_RATE * dt;
        float err   = targetHeading - carYaw;
        float delta = KP_HEADING * err;
        delta = constrain(delta, -(float)STEER_MAX_DELTA, (float)STEER_MAX_DELTA);
        int us = STEER_CENTER_US + (int)(STEER_SIGN * delta);
        writeServoUs(clampi(us, SERVO_MIN_US, SERVO_MAX_US));
    } else {
        headingActive = false;
        uint16_t raw = xboxController.xboxNotif.joyLHori;
        int us = STEER_REVERSED
               ? map(raw, 0, 65535, SERVO_MAX_US, SERVO_MIN_US)
               : map(raw, 0, 65535, SERVO_MIN_US, SERVO_MAX_US);
        writeServoUs(us);
    }
}


/* -------------------------------------------------------------------------
 * IMU ACQUISITION — MPU6050 over I2C (addr 0x68, AD0 -> GND)
 *   Complementary filter (library) => drift-free roll & pitch.
 *   Yaw is gyro-only and drifts without a magnetometer.
 *   calcGyroOffsets(true) run at startup with the car held still.
 * ------------------------------------------------------------------------- */
#define ACCEL_SIGN  (-1)   // flip to +1 if forward acceleration reads negative
void updateIMU() {
    mpu6050.update();
    carPitch     = -mpu6050.getAngleY();                    // nose-up positive
    carRoll      =  mpu6050.getAngleX();
    carYaw       =  mpu6050.getAngleZ();                    // gyro-only, drifts
    accelFwdMps2 =  ACCEL_SIGN * mpu6050.getAccX() * 9.81f; // resting offset
}
// Resting-offset capture (run once at startup, car held STILL)
// Called inside setup() after calcGyroOffsets():
//   float accXrest = 0;
//   for (int i=0; i<200; i++) { mpu6050.update(); accXrest += mpu6050.getAccX(); delay(5); }
//   accXrest /= 200.0f;   // stored as ACCEL_X_REST_OFFSET


/* -------------------------------------------------------------------------
 * WHEEL-SPEED SENSING — US1881 Hall-effect, interrupt-driven
 *   4 magnets N-S-N-S on a 60 mm circle; wheel OD 130 mm.
 *   CHANGE-mode interrupt captures all 4 transitions per revolution.
 *   500 ms windowed calculation smooths transient noise.
 *   ISR in IRAM; pulseCount volatile, read with interrupts disabled.
 *   Limitation: cannot distinguish forward/reverse (reverse reads positive).
 * ------------------------------------------------------------------------- */
volatile unsigned long pulseCount = 0;
void IRAM_ATTR onHall() { pulseCount++; }   // ISR — runs on every magnet edge
void updateSpeed() {
    unsigned long now = millis();
    if (now - lastSpeedCalc < SPEED_INTERVAL) return;   // 500 ms window
    noInterrupts();
    unsigned long count = pulseCount;
    interrupts();
    unsigned long pulses = count - lastSpeedCount;
    float dt             = (now - lastSpeedCalc) / 1000.0f;
    lastSpeedCount       = count;
    lastSpeedCalc        = now;
    float revs   = (float)pulses / PULSES_PER_REV;        // 4 pulses/rev
    float meters = revs * WHEEL_CIRCUMFERENCE_M;          // pi * 0.130 m
    float mps    = meters / dt;
    speedKmh     = mps * 3.6f;
}


/* -------------------------------------------------------------------------
 * JETSON TELEMETRY — one JSON object per line, USB serial, 115200 baud, 20 Hz
 *   steeringAngleDeg is BACK-CALCULATED from the last servo pulse:
 *     theta = (PW_last - PW_center) / STEER_MAX_DELTA * STEER_FULL_DEG
 *     PW_center = 1472 us, STEER_MAX_DELTA = 350, STEER_FULL_DEG = 79.5 deg
 *   stationary := speedKmh <= 0.5 km/h.
 *   When DEBUG_HUMAN is false the port carries ONLY clean JSON.
 * ------------------------------------------------------------------------- */
void sendTelemetry() {
    unsigned long now = millis();
    if (now - lastTelem < TELEM_INTERVAL) return;   // 50 ms = 20 Hz
    lastTelem = now;
    float steerDeg = (float)(lastServoUs - STEER_CENTER_US)
                   / (float)STEER_MAX_DELTA * STEER_FULL_DEG;
    bool stationary = (fabs(speedKmh) <= 0.5f);
    Serial.print("{\"speedKmh\":");
    Serial.print(speedKmh, 2);
    Serial.print(",\"steeringAngleDeg\":");
    Serial.print(steerDeg, 1);
    Serial.print(",\"accelerationMps2\":");
    Serial.print(accelFwdMps2, 2);
    Serial.print(",\"obstacleDistanceMeters\":");
    Serial.print(NO_OBSTACLE_M, 1);
    Serial.print(",\"turnSignal\":false");
    Serial.print(",\"ignitionOn\":true");
    Serial.print(",\"stationary\":");
    Serial.print(stationary ? "true" : "false");
    Serial.println("}");
}


/* -------------------------------------------------------------------------
 * SAFE-START GATE — hold NEUTRAL after BLE connect until BOTH triggers
 * are confirmed released. Re-arms on every reconnection.
 * ------------------------------------------------------------------------- */
if (!readyToDrive) {
  currentThrottle = ESC_NEUTRAL_US;
  writeEscUs(ESC_NEUTRAL_US);
  static bool hinted = false;
  if (rt <= TRIG_DEAD_ZONE && lt <= TRIG_DEAD_ZONE) {
    readyToDrive = true; hinted = false;
    if (DEBUG_HUMAN) Serial.println("Ready to drive.");
  } else if (!hinted && DEBUG_HUMAN) {
    Serial.println("Triggers held - release both to arm.");
    hinted = true;
  }
  return;
}


/* -------------------------------------------------------------------------
 * CONTROLLER-DISCONNECT FAIL-SAFE — after DISCONNECT_LIMIT = 200 loops
 * (~75 ms) of lost link, force ESC to neutral and clear readyToDrive.
 * ------------------------------------------------------------------------- */
} else {
  disconnectCount++;
  readyToDrive = false;
  if (disconnectCount > DISCONNECT_LIMIT) {
    currentThrottle = ESC_NEUTRAL_US;
    writeEscUs(ESC_NEUTRAL_US);
  }
}


/* -------------------------------------------------------------------------
 * DEBUG_HUMAN — compile-time flag gating all human-readable Serial output.
 * MUST be false before connecting the USB cable to the Jetson, because the
 * FastAPI server's json.loads() parser throws on any non-JSON line.
 * ------------------------------------------------------------------------- */
#define DEBUG_HUMAN true   // set FALSE before connecting to Jetson

// In setup():
if (DEBUG_HUMAN) Serial.println("Waiting for Xbox controller...");

// In loop():
if (DEBUG_HUMAN) {
  Serial.print("RT:"); Serial.print(rt);
  Serial.print(" Thr:"); Serial.print(currentThrottle);
  Serial.print(" Spd:"); Serial.println(speedKmh, 2);
}


/* =========================================================================
 * MAIN LOOP STRUCTURE (no blocking delay() after setup)
 * -------------------------------------------------------------------------
 * Five concurrent tasks multiplexed on independent millis()-based timers:
 *   1. xboxController.onLoop()  — every iteration (min BLE input latency)
 *   2. updateIMU()              — every iteration (library self-times)
 *   3. updateSpeed()            — 500 ms windowed pulse-count calculation
 *   4. sendTelemetry()          — 50 ms (20 Hz) JSON serial output
 *   5. Throttle ramp            — 15 ms timer; ramps currentThrottle toward
 *                                 decideThrottle() target, writes clamped
 *                                 result to the ESC.
 * Steering is updated every iteration (not timer-gated) for lowest latency.
 * clampi() bounds currentThrottle to [ESC_REV_MAX_US, ESC_FWD_MAX_US].
 * ========================================================================= */
