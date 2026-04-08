#include <SimpleFOC.h>

// --- 1. SENSORS ---
Encoder motor_encoder = Encoder(PB6, PB7, 1000);
MagneticSensorPWM joint_encoder = MagneticSensorPWM(PA15, 4, 922);  

void doA() { motor_encoder.handleA(); }
void doB() { motor_encoder.handleB(); }
void doPWM() { joint_encoder.handlePWM(); }

// --- 2. HARDWARE INSTANCES ---
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
BLDCMotor motor = BLDCMotor(14); // 14 pole pairs for G60
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// --- 3. CONFIGURATION & STATE ---
const float GEAR_RATIO = 20.0f;  // 20:1 for G60
const float MOTOR_KT = 0.205f;   // Kt for G60

// --- WATCHDOG SETTINGS (Imported from Biped Firmware) ---
unsigned long last_command_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
const float SAFE_DAMPING_KD = 5.0f;

// --- 4. LOW-PASS FILTERS ---
LowPassFilter lpf_q(0.005f);  
LowPassFilter lpf_dq(0.04f);  

// 16-Byte Struct: PC -> STM32 (Maintained for Python compatibility)
struct __attribute__((packed)) RLCommand {
  float q_des;
  float kp;
  float kd;
  float tau_ff;
} cmd = { 0.0f, 0.0f, 0.0f, 0.0f };

// 16-Byte Struct: STM32 -> PC (Maintained for Python compatibility)
struct __attribute__((packed)) RLState {
  float q_curr;
  float dq_curr;
  float tau_cmd;  
  float motor_q; // Essential for encoder cross-referencing in calibration
} state = { 0.0f, 0.0f, 0.0f, 0.0f };

void setup() {
  Serial2.begin(115200);

  motor_encoder.init();
  motor_encoder.enableInterrupts(doA, doB);
  joint_encoder.init();
  joint_encoder.enableInterrupt(doPWM);

  driver.voltage_power_supply = 24;
  driver.init();
  currentSense.linkDriver(&driver);
  while (!currentSense.init()) {}
  motor.linkCurrentSense(&currentSense);

  motor.linkSensor(&motor_encoder);
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;

  motor.PID_current_q.P = 3; motor.PID_current_q.I = 300;
  motor.PID_current_d.P = 3; motor.PID_current_d.I = 300;
  motor.LPF_current_q.Tf = 0.005f; motor.LPF_current_d.Tf = 0.005f;

  motor.current_limit = 6.57; // Strict limit for G60
  motor.voltage_limit = 24.0;
  motor.voltage_sensor_align = 4.0;

  motor.init();
  motor.initFOC();

  for (int i = 0; i < 50; i++) {
    joint_encoder.update();
    delay(1);
  }

  // Initialize watchdog timer at boot
  last_command_time = millis();
}

void loop() {
  motor.loopFOC();

  // --- 1. SENSOR FUSION KINEMATICS ---
  joint_encoder.update();

  // Position from Absolute PWM Sensor (Clean 1:1 mapping)
  float raw_q = (joint_encoder.getAngle() / 2.0f);
  
  // Velocity from Incremental Motor Encoder (Clean, inverted to match kinematics)
  float raw_dq = -(motor.shaft_velocity / GEAR_RATIO);

  // --- 2. APPLY THE FILTERS ---
  state.q_curr = lpf_q(raw_q);
  state.dq_curr = lpf_dq(raw_dq);
  state.motor_q = motor_encoder.getAngle(); // Keep raw motor angle for PC script

  // --- 3. MICRO-DEADBAND ---
  // Motor encoder is clean enough to drop from 1.2f to 0.05f
  if (abs(state.dq_curr) < 0.05f) {
    state.dq_curr = 0.0f;
  }

  // --- 4. SAFETY WATCHDOG ---
  if (millis() - last_command_time > WATCHDOG_TIMEOUT_MS) {
    cmd.kp = 0.0f;
    cmd.kd = SAFE_DAMPING_KD;
    cmd.tau_ff = 0.0f;
  }

  // --- 5. IMPEDANCE CONTROL LAW ---
  float pos_error = cmd.q_des - state.q_curr;
  float vel_error = 0.0f - state.dq_curr;

  float joint_torque_nm = (cmd.kp * pos_error) + (cmd.kd * vel_error) + cmd.tau_ff;
  state.tau_cmd = joint_torque_nm;

  // --- 6. GEARING & CURRENT CONVERSION ---
  float motor_torque_nm = joint_torque_nm / GEAR_RATIO;
  float current_command = motor_torque_nm / MOTOR_KT;

  motor.move(-current_command);

  command_interface();
}

void command_interface() {
  if (Serial2.available() >= sizeof(RLCommand)) {
    Serial2.readBytes((uint8_t*)&cmd, sizeof(RLCommand));
    while (Serial2.available() > 0) Serial2.read();
    Serial2.write((uint8_t*)&state, sizeof(RLState));
    
    // VALID COMMAND RECEIVED: Reset the watchdog timer
    last_command_time = millis();
  }
}