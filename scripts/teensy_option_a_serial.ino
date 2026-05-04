//***********************************************************************************************
// Teensy 4.1 skid-steer base firmware for ROS 2 Option A
//
// This version removes ROS 1 rosserial from the Teensy.
// The Teensy acts as a deterministic USB-serial motor/encoder controller.
//
// NUC -> Teensy command:
//   VEL,<linear_mps>,<angular_radps>\n
//
// Teensy -> NUC telemetry:
//   ODOM,<left_ticks>,<right_ticks>,<v_mps>,<w_radps>,<theta_rad>,<remote_state>\n
//
// Remote/Pixhawk manual override logic is intentionally preserved from the original code:
//   REMOTE_AUTO   allows NUC/autonomous commands
//   REMOTE_MANUAL allows remote/Pixhawk direct motor control
//   NO_REMOTE     also allows autonomous commands, matching the original behavior
//
// IMPORTANT:
//   Tune bDR, rNominalDR, eTickDR, PID gains, omega_max, and PWM maps for your robot.
//***********************************************************************************************

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <IntervalTimer.h>

IntervalTimer myTimer;

//===== Remote Control Settings
#define RC_NUM_CHANNELS  8

#define RC_CH1  0     // Left Motors
#define RC_CH3  1     // Right Motors
#define RC_CH5  2     // Control State

#define NO_REMOTE 0
#define REMOTE_AUTO 1
#define REMOTE_MANUAL 2

#define RC_CH1_INPUT  A7
#define RC_CH3_INPUT  A8
#define RC_CH5_INPUT  A9
//=====

//===== Sonar Settings
#define SONAR_TRIGGER 14
#define SONAR_CH1_INPUT  A4
#define SONAR_CH2_INPUT  A3
//=====

//===== Robot status indicators
// Built-in Teensy LED status patterns.
const int STATUS_LED_PIN = LED_BUILTIN;

const float COMMAND_ACTIVE_EPS = 0.001;

uint32_t last_status_blink_ms = 0;
bool status_led_state = false;
//=====

// To store the state/PWM values coming out of the Pixhawk pins
uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

// Initial Remote state/PWM
int rem_pwm_cmdl = 0;
int rem_pwm_cmdr = 0;
int rem_state = NO_REMOTE;
volatile int pwm_cmdr = 0;
volatile int pwm_cmdl = 0;

// Slowdown counter for Sonar
int sonarcount = 0;

//===== H-Bridge Settings
int InA1 = 3;
int InA2 = 2;
int enabA = 4;

int InB1 = 5;
int InB2 = 6;
int enabB = 7;

// Set motor polarity so a positive command means forward robot motion.
const int LEFT_MOTOR_POLARITY = 1;
const int RIGHT_MOTOR_POLARITY = 1;
//=====

//===== Encoder settings
int enco_R_A = 29;
int enco_R_B = 30;
volatile int32_t enco_R_pos = 0;
int enco_R_A_old = LOW;
int enco_R_B_old = LOW;

int enco_L_A = 35;
int enco_L_B = 36;
volatile int32_t enco_L_pos = 0;
int enco_L_A_old = LOW;
int enco_L_B_old = LOW;

// Encoder polarity is separate from motor polarity. Use this only if forward
// physical wheel motion makes the corresponding tick count decrease.
const int LEFT_ENCODER_POLARITY = 1;
const int RIGHT_ENCODER_POLARITY = 1;
//=====

//===== Vehicle parameters copied from original code
float bDR = 1.6 / 3.28;      // [m] effective platform width
float rNominalDR = 0.054;    // [m] nominal wheel radius
float eTickDR = 16000;       // [ticks/m] ticks per 1 m vehicle translation
//=====

//===== Controller parameters copied from original structure
float KP1 = 5.0;             // velocity controller
float KI1 = 0.1;
float KD1 = 0.01;

float KP2 = 10.0;            // yaw-rate controller; was heading-angle controller in original
float KI2 = 0.1;
float KD2 = 1.0;

float omega_max = 15.0;      // max wheel angular velocity command

// Feedforward trims compensate for left/right drive mismatch. Initial values
// come from a stand test where straight command produced positive yaw drift.
float leftMotorFFTrim = 0.93;
float rightMotorFFTrim = 1.08;
const int AUTONOMOUS_MIN_PWM = 85;
const int AUTONOMOUS_MAX_PWM = 220;
//=====

//===== Command state
volatile float velcom = 0.0;       // commanded linear velocity [m/s]
volatile float yawratecom = 0.0;   // commanded angular velocity [rad/s]
volatile bool new_command_seen = false;
volatile uint32_t last_cmd_ms = 0;

const uint32_t COMMAND_TIMEOUT_MS = 500;
const float COMMAND_STOP_EPS = 0.001;
const float OMEGA_COMMAND_DEADBAND = 0.05;
//=====

//===== Odometry/control state
volatile float omegaL = 0.0;
volatile float omegaR = 0.0;

float vEst_nav = 0.0;
float thetaDot = 0.0;
float thetaEst_nav = 0.0;
float xdot = 0.0;
float ydot = 0.0;
float xEst = 0.0;
float yEst = 0.0;

int32_t enco_L_pos_prev = 0;
int32_t enco_R_pos_prev = 0;

float prev_time_navcontrol = 0.0;
float dt_millis = 30.0;
float dt = dt_millis / 1000.0;
float TsampleEncoderDR = 0.0;

// PID state
float velerror = 0.0;
float integral_vel = 0.0;
float velprevError = 0.0;

float yawrateerror = 0.0;
float integral_yawrate = 0.0;
float yawrateprevError = 0.0;

float previous_time_v = 0.0;
float previous_time_w = 0.0;

float u_vel = 0.0;
float u_yaw = 0.0;
float omegaR_u = 0.0;
float omegaL_u = 0.0;
//=====

//===== Serial parsing
char serial_line[96];
size_t serial_index = 0;
//=====

//===== Telemetry timing
uint32_t last_odom_tx_ms = 0;
const uint32_t ODOM_TX_PERIOD_MS = 50;  // 20 Hz
//=====


//===== Remote control/Pixhawk functions
void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }

void write_left_motor_pwm(int cmd) {
  cmd *= LEFT_MOTOR_POLARITY;

  if (cmd > 0) {
    analogWrite(InA1, 0);
    analogWrite(InA2, abs(cmd));
    digitalWrite(enabA, HIGH);
  } else if (cmd < 0) {
    analogWrite(InA2, 0);
    analogWrite(InA1, abs(cmd));
    digitalWrite(enabA, HIGH);
  } else {
    analogWrite(InA1, 0);
    analogWrite(InA2, 0);
    digitalWrite(enabA, HIGH);
  }
}

void write_right_motor_pwm(int cmd) {
  cmd *= RIGHT_MOTOR_POLARITY;

  if (cmd < 0) {
    analogWrite(InB1, 0);
    analogWrite(InB2, abs(cmd));
    digitalWrite(enabB, HIGH);
  } else if (cmd > 0) {
    analogWrite(InB2, 0);
    analogWrite(InB1, abs(cmd));
    digitalWrite(enabB, HIGH);
  } else {
    analogWrite(InB1, 0);
    analogWrite(InB2, 0);
    digitalWrite(enabB, HIGH);
  }
}

void check_manual_overwrite() {
  rc_read_values();

  // Preserve original switch logic exactly.
  if (rc_values[RC_CH5] > 1000 && rc_values[RC_CH5] < 1250) {
    rem_state = REMOTE_AUTO;
  } else if (rc_values[RC_CH5] >= 1750 && rc_values[RC_CH5] < 2050) {
    rem_state = REMOTE_MANUAL;
  } else {
    rem_state = NO_REMOTE;
  }

  if (rem_state == REMOTE_MANUAL) {
    if (rc_values[RC_CH1] > 1000 && rc_values[RC_CH1] < 1460) {
      rem_pwm_cmdl = int(map(rc_values[RC_CH1], 1000, 1480, -240, -1));
    } else if (rc_values[RC_CH1] > 1540 && rc_values[RC_CH1] < 2000) {
      rem_pwm_cmdl = int(map(rc_values[RC_CH1], 1520, 2000, 1, 240));
    } else {
      rem_pwm_cmdl = 0;
    }

    if (rc_values[RC_CH3] > 1000 && rc_values[RC_CH3] < 1460) {
      rem_pwm_cmdr = int(map(rc_values[RC_CH3], 1000, 1480, -240, -1));
    } else if (rc_values[RC_CH3] > 1540 && rc_values[RC_CH3] < 2000) {
      rem_pwm_cmdr = int(map(rc_values[RC_CH3], 1520, 2000, 1, 240));
    } else {
      rem_pwm_cmdr = 0;
    }

    write_left_motor_pwm(rem_pwm_cmdl);
    write_right_motor_pwm(rem_pwm_cmdr);
  }
}
//=====

//===== Robot status indicator functions
void set_status_led(bool on) {
  status_led_state = on;
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

void blink_status_led(uint32_t now, uint32_t period_ms) {
  if ((now - last_status_blink_ms) >= period_ms) {
    last_status_blink_ms = now;
    set_status_led(!status_led_state);
  }
}

void update_status_indicators() {
  uint32_t now = millis();
  float local_velcom;
  float local_yawratecom;
  uint32_t local_last_cmd_ms;
  bool local_new_command_seen;

  noInterrupts();
  local_velcom = velcom;
  local_yawratecom = yawratecom;
  local_last_cmd_ms = last_cmd_ms;
  local_new_command_seen = new_command_seen;
  interrupts();

  bool command_recent = (now - local_last_cmd_ms) <= COMMAND_TIMEOUT_MS;
  bool command_active = command_recent &&
                        (fabs(local_velcom) > COMMAND_ACTIVE_EPS ||
                         fabs(local_yawratecom) > COMMAND_ACTIVE_EPS);
  bool command_timed_out = local_new_command_seen && !command_recent;

  if (rem_state == REMOTE_MANUAL) {
    blink_status_led(now, 100);       // Manual override: fast blink.
  } else if (command_timed_out) {
    blink_status_led(now, 75);        // Command watchdog timeout: very fast blink.
  } else if (command_active) {
    set_status_led(true);             // Autonomous command active: solid on.
  } else {
    blink_status_led(now, 500);       // Ready/idle: slow heartbeat.
  }
}
//=====

//===== Autonomous control safety helpers
bool is_stop_command(float v, float w) {
  return fabs(v) <= COMMAND_STOP_EPS && fabs(w) <= COMMAND_STOP_EPS;
}

void reset_pid_state() {
  velerror = 0.0;
  integral_vel = 0.0;
  velprevError = 0.0;
  yawrateerror = 0.0;
  integral_yawrate = 0.0;
  yawrateprevError = 0.0;
  u_vel = 0.0;
  u_yaw = 0.0;
  omegaR_u = 0.0;
  omegaL_u = 0.0;
}

void stop_autonomous_control() {
  reset_pid_state();
  pwm_cmdr = 0;
  pwm_cmdl = 0;
}
//=====


//===== Motor Command Functions
void PWM_CMD_R() {
  // Preserve original behavior: REMOTE_AUTO or NO_REMOTE allows autonomous commands.
  if ((rem_state == REMOTE_AUTO) || (rem_state == NO_REMOTE)) {
    write_right_motor_pwm(pwm_cmdr);
  }
}

void PWM_CMD_L() {
  // Preserve original behavior: REMOTE_AUTO or NO_REMOTE allows autonomous commands.
  if ((rem_state == REMOTE_AUTO) || (rem_state == NO_REMOTE)) {
    write_left_motor_pwm(pwm_cmdl);
  }
}
//=====


//===== Encoder interrupt handlers
void doEncoder_R_A() {
  if (digitalRead(enco_R_A) != enco_R_A_old) {
    if (digitalRead(enco_R_A) == HIGH) {
      if (digitalRead(enco_R_B) == LOW) {
        enco_R_pos = enco_R_pos + 1;
      } else {
        enco_R_pos = enco_R_pos - 1;
      }
    } else {
      if (digitalRead(enco_R_B) == HIGH) {
        enco_R_pos = enco_R_pos + 1;
      } else {
        enco_R_pos = enco_R_pos - 1;
      }
    }
    enco_R_A_old = digitalRead(enco_R_A);
  }
}

void doEncoder_R_B() {
  if (digitalRead(enco_R_B) != enco_R_B_old) {
    if (digitalRead(enco_R_B) == HIGH) {
      if (digitalRead(enco_R_A) == HIGH) {
        enco_R_pos = enco_R_pos + 1;
      } else {
        enco_R_pos = enco_R_pos - 1;
      }
    } else {
      if (digitalRead(enco_R_A) == LOW) {
        enco_R_pos = enco_R_pos + 1;
      } else {
        enco_R_pos = enco_R_pos - 1;
      }
    }
    enco_R_B_old = digitalRead(enco_R_B);
  }
}

void doEncoder_L_A() {
  if (digitalRead(enco_L_A) != enco_L_A_old) {
    if (digitalRead(enco_L_A) == HIGH) {
      if (digitalRead(enco_L_B) == LOW) {
        enco_L_pos = enco_L_pos + 1;
      } else {
        enco_L_pos = enco_L_pos - 1;
      }
    } else {
      if (digitalRead(enco_L_B) == HIGH) {
        enco_L_pos = enco_L_pos + 1;
      } else {
        enco_L_pos = enco_L_pos - 1;
      }
    }
    enco_L_A_old = digitalRead(enco_L_A);
  }
}

void doEncoder_L_B() {
  if (digitalRead(enco_L_B) != enco_L_B_old) {
    if (digitalRead(enco_L_B) == HIGH) {
      if (digitalRead(enco_L_A) == HIGH) {
        enco_L_pos = enco_L_pos + 1;
      } else {
        enco_L_pos = enco_L_pos - 1;
      }
    } else {
      if (digitalRead(enco_L_A) == LOW) {
        enco_L_pos = enco_L_pos + 1;
      } else {
        enco_L_pos = enco_L_pos - 1;
      }
    }
    enco_L_B_old = digitalRead(enco_L_B);
  }
}
//=====


//===== Serial command handling
void handle_serial_line(char *line) {
  // Expected: VEL,<linear_mps>,<angular_radps>
  char *cmd = strtok(line, ",");

  if (cmd == nullptr) {
    return;
  }

  if (strcmp(cmd, "VEL") == 0) {
    char *v_str = strtok(nullptr, ",");
    char *w_str = strtok(nullptr, ",");

    if (v_str == nullptr || w_str == nullptr) {
      return;
    }

    float v = atof(v_str);
    float w = atof(w_str);

    if (!isfinite(v) || !isfinite(w)) {
      return;
    }

    noInterrupts();
    velcom = v;
    yawratecom = w;
    last_cmd_ms = millis();
    new_command_seen = true;
    if (is_stop_command(v, w)) {
      stop_autonomous_control();
    }
    interrupts();
  } else if (strcmp(cmd, "RESET_ODOM") == 0) {
    noInterrupts();
    enco_R_pos = 0;
    enco_L_pos = 0;
    enco_R_pos_prev = 0;
    enco_L_pos_prev = 0;
    interrupts();

    xEst = 0.0;
    yEst = 0.0;
    thetaEst_nav = 0.0;
    thetaDot = 0.0;
    velcom = 0.0;
    yawratecom = 0.0;

    stop_autonomous_control();
  }
}

void read_serial_commands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (serial_index > 0) {
        serial_line[serial_index] = '\0';
        handle_serial_line(serial_line);
        serial_index = 0;
      }
    } else {
      if (serial_index < sizeof(serial_line) - 1) {
        serial_line[serial_index++] = c;
      } else {
        serial_index = 0;
      }
    }
  }
}
//=====


//===== Control output
void controloutput() {
  float local_velcom;
  float local_yawratecom;
  uint32_t local_last_cmd_ms;
  uint32_t now_ms = millis();

  noInterrupts();
  local_velcom = velcom;
  local_yawratecom = yawratecom;
  local_last_cmd_ms = last_cmd_ms;
  interrupts();

  // Watchdog or explicit stop: remove power command immediately and clear PID memory.
  if (((now_ms - local_last_cmd_ms) > COMMAND_TIMEOUT_MS) ||
      is_stop_command(local_velcom, local_yawratecom)) {
    stop_autonomous_control();
    return;
  }

  // ROS 2/Nav2 sends cmd_vel as linear velocity and yaw rate. The original
  // Noetic controller used velocity + heading angle PID; reusing those heading
  // gains for yaw-rate control caused startup jitter. Feedforward is the stable
  // baseline. Add per-wheel encoder PID later if tighter speed tracking is needed.
  omegaL_u = ((local_velcom - local_yawratecom * bDR * 0.5) / rNominalDR) * leftMotorFFTrim;
  omegaR_u = ((local_velcom + local_yawratecom * bDR * 0.5) / rNominalDR) * rightMotorFFTrim;

  if (omegaR_u > omega_max) omegaR_u = omega_max;
  if (omegaR_u < -omega_max) omegaR_u = -omega_max;
  if (omegaL_u > omega_max) omegaL_u = omega_max;
  if (omegaL_u < -omega_max) omegaL_u = -omega_max;

  if (fabs(omegaR_u) < OMEGA_COMMAND_DEADBAND) {
    pwm_cmdr = 0;
  } else if (omegaR_u < 0) {
    pwm_cmdr = map((long)(omegaR_u * 1000), 0, (long)(-omega_max * 1000), -AUTONOMOUS_MIN_PWM, -AUTONOMOUS_MAX_PWM);
  } else if (omegaR_u > 0) {
    pwm_cmdr = map((long)(omegaR_u * 1000), 0, (long)(omega_max * 1000), AUTONOMOUS_MIN_PWM, AUTONOMOUS_MAX_PWM);
  } else {
    pwm_cmdr = 0;
  }

  if (fabs(omegaL_u) < OMEGA_COMMAND_DEADBAND) {
    pwm_cmdl = 0;
  } else if (omegaL_u < 0) {
    pwm_cmdl = map((long)(omegaL_u * 1000), 0, (long)(-omega_max * 1000), -AUTONOMOUS_MIN_PWM, -AUTONOMOUS_MAX_PWM);
  } else if (omegaL_u > 0) {
    pwm_cmdl = map((long)(omegaL_u * 1000), 0, (long)(omega_max * 1000), AUTONOMOUS_MIN_PWM, AUTONOMOUS_MAX_PWM);
  } else {
    pwm_cmdl = 0;
  }
}
//=====


//===== Navigation / odometry update
void NavControl() {
  unsigned long current_time_navcontrol = millis();
  if ((current_time_navcontrol - prev_time_navcontrol) > 0) {
    TsampleEncoderDR = (current_time_navcontrol - prev_time_navcontrol) / 1000.0;
  } else {
    TsampleEncoderDR = dt;
  }

  int32_t local_L;
  int32_t local_R;

  noInterrupts();
  local_L = enco_L_pos * LEFT_ENCODER_POLARITY;
  local_R = enco_R_pos * RIGHT_ENCODER_POLARITY;
  interrupts();

  int32_t diff_L = local_L - enco_L_pos_prev;
  int32_t diff_R = local_R - enco_R_pos_prev;

  // Keep the original wrap handling idea.
  if (diff_L < -65000) {
    omegaL = (diff_L + 65536) / (eTickDR * rNominalDR * TsampleEncoderDR);
  } else if (diff_L > 65000) {
    omegaL = (diff_L - 65536) / (eTickDR * rNominalDR * TsampleEncoderDR);
  } else {
    omegaL = diff_L / (eTickDR * rNominalDR * TsampleEncoderDR);
  }

  if (diff_R < -65000) {
    omegaR = (diff_R + 65536) / (eTickDR * rNominalDR * TsampleEncoderDR);
  } else if (diff_R > 65000) {
    omegaR = (diff_R - 65536) / (eTickDR * rNominalDR * TsampleEncoderDR);
  } else {
    omegaR = diff_R / (eTickDR * rNominalDR * TsampleEncoderDR);
  }

  vEst_nav = (omegaL + omegaR) * rNominalDR / 2.0;
  // ROS uses positive yaw for counterclockwise rotation viewed from above.
  thetaDot = (omegaR - omegaL) * rNominalDR / bDR;

  thetaEst_nav += thetaDot * TsampleEncoderDR;

  xdot = vEst_nav * cos(thetaEst_nav);
  ydot = vEst_nav * sin(thetaEst_nav);

  xEst += xdot * TsampleEncoderDR;
  yEst += ydot * TsampleEncoderDR;

  enco_L_pos_prev = local_L;
  enco_R_pos_prev = local_R;
  prev_time_navcontrol = current_time_navcontrol;

  controloutput();
}
//=====


//===== Telemetry output
void publish_odom_serial() {
  uint32_t now = millis();
  if ((now - last_odom_tx_ms) < ODOM_TX_PERIOD_MS) {
    return;
  }
  last_odom_tx_ms = now;

  int32_t local_L;
  int32_t local_R;

  noInterrupts();
  local_L = enco_L_pos * LEFT_ENCODER_POLARITY;
  local_R = enco_R_pos * RIGHT_ENCODER_POLARITY;
  interrupts();

  Serial.print("ODOM,");
  Serial.print(local_L);
  Serial.print(",");
  Serial.print(local_R);
  Serial.print(",");
  Serial.print(vEst_nav, 6);
  Serial.print(",");
  Serial.print(thetaDot, 6);
  Serial.print(",");
  Serial.print(thetaEst_nav, 6);
  Serial.print(",");
  Serial.println(rem_state);
}
//=====


void setup() {
  Serial.begin(115200);

  pinMode(STATUS_LED_PIN, OUTPUT);
  set_status_led(false);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);

  attachInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  attachInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  attachInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);

  pinMode(enabA, OUTPUT);
  digitalWrite(enabA, LOW);
  pinMode(InA1, OUTPUT);
  analogWrite(InA1, 0);
  pinMode(InA2, OUTPUT);
  analogWrite(InA2, 0);

  pinMode(enabB, OUTPUT);
  digitalWrite(enabB, LOW);
  pinMode(InB1, OUTPUT);
  analogWrite(InB1, 0);
  pinMode(InB2, OUTPUT);
  analogWrite(InB2, 0);

  analogWriteFrequency(InA1, 20000);
  analogWriteFrequency(InA2, 20000);
  analogWriteFrequency(InB1, 20000);
  analogWriteFrequency(InB2, 20000);

  pinMode(enco_R_A, INPUT);
  pinMode(enco_R_B, INPUT);
  pinMode(enco_L_A, INPUT);
  pinMode(enco_L_B, INPUT);

  enco_R_A_old = digitalRead(enco_R_A);
  enco_R_B_old = digitalRead(enco_R_B);
  enco_L_A_old = digitalRead(enco_L_A);
  enco_L_B_old = digitalRead(enco_L_B);

  attachInterrupt(enco_R_A, doEncoder_R_A, CHANGE);
  attachInterrupt(enco_R_B, doEncoder_R_B, CHANGE);
  attachInterrupt(enco_L_A, doEncoder_L_A, CHANGE);
  attachInterrupt(enco_L_B, doEncoder_L_B, CHANGE);

  pinMode(SONAR_TRIGGER, OUTPUT);
  digitalWrite(SONAR_TRIGGER, LOW);
  pinMode(SONAR_CH1_INPUT, INPUT);
  pinMode(SONAR_CH2_INPUT, INPUT);

  last_cmd_ms = millis();

  myTimer.begin(NavControl, 30000); // 30 ms = 33.3 Hz
}

void loop() {
  read_serial_commands();

  check_manual_overwrite();

  update_status_indicators();

  PWM_CMD_L();
  PWM_CMD_R();

  // Preserve sonar timing behavior, but do not stream sonar over serial yet.
  sonarcount = (sonarcount + 1) % 15;
  switch (sonarcount) {
    case 1:
      digitalWrite(SONAR_TRIGGER, HIGH);
      break;
    case 2:
      digitalWrite(SONAR_TRIGGER, LOW);
      break;
    default:
      break;
  }

  publish_odom_serial();

  delay(10);
}
