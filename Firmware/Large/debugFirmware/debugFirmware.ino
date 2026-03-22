#include <SimpleFOC.h>

// --- 1. SENSORS ---
Encoder motor_encoder = Encoder(PB6, PB7, 1000); 
MagneticSensorPWM joint_encoder = MagneticSensorPWM(PA15, 4, 1024);

void doA() { motor_encoder.handleA(); }
void doB() { motor_encoder.handleB(); }
void doPWM() { joint_encoder.handlePWM(); }

// --- 2. HARDWARE INSTANCES ---
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
BLDCMotor motor = BLDCMotor(21);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// --- 3. CONFIGURATION & STATE ---
const float GEAR_RATIO = 16.0f; // CubeMars G80 16:1 Reduction
const float MOTOR_KT = 0.178f;  // Torque Constant (Nm/A)

float boot_wrap_correction = 0.0f; 
float python_zero_offset = 0.0f;   // Set this manually if you want a perfect 0 for bench testing

// --- 4. HARDCODED IMPEDANCE TUNING ---
// Change these values and re-upload to test different stiffness feels
float kp = 3.0f;      // Start low (1.0 to 3.0) for bench testing
float kd = 0.0f;     // Start very low (0.01 to 0.1) to prevent violent vibration
float tau_ff = 0.0f;  

float q_des = 0.0f;   // Target angle (Controlled via Serial Monitor)

// State and Telemetry
struct RLState {
  float q_curr;
  float dq_curr;
} state = {0.0f, 0.0f};

float current_pos_error = 0.0f;
float current_joint_torque = 0.0f;

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

  motor.current_limit = 12.2; // Peak safe bench limit
  motor.voltage_limit = 12.0; 
  motor.voltage_sensor_align = 4.0; 

  motor.init();
  motor.initFOC();

  for(int i = 0; i < 50; i++) {
      joint_encoder.update();
      delay(1);
  }
  
  if (joint_encoder.getAngle() > PI) {
      boot_wrap_correction = TWO_PI;
  }

  Serial2.println("\n--- STREAMLINED BENCH TEST MODE ---");
  Serial2.print("Active Tuning -> kp: "); Serial2.print(kp);
  Serial2.print(" | kd: "); Serial2.println(kd);
  Serial2.println("Type a target angle (e.g. 0.5) and hit Enter/Send.");
}

void loop() {
  motor.loopFOC(); 

  // TRUE KINEMATICS
  joint_encoder.update();
  float continuous_enc_angle = joint_encoder.getAngle() - boot_wrap_correction;
  
  state.q_curr = (continuous_enc_angle / 2.0f) - python_zero_offset; 
  state.dq_curr = joint_encoder.getVelocity() / 2.0f; 

  // IMPEDANCE CONTROL LAW
  current_pos_error = q_des - state.q_curr;
  float vel_error = 0.0f - state.dq_curr; 
  
  current_joint_torque = (kp * current_pos_error) + (kd * vel_error) + tau_ff;

  // GEARING & CURRENT CONVERSION
  float motor_torque_nm = current_joint_torque / GEAR_RATIO;
  float current_command = motor_torque_nm / MOTOR_KT;

  // Command Motor
  motor.move(-current_command);

  // Human Interface & Telemetry
  command_interface();
  print_telemetry();
}

void command_interface() {
  static char buffer[32];
  static uint8_t index = 0;

  while (Serial2.available() > 0) {
    char c = Serial2.read();
    
    // Parse on Newline or Carriage Return
    if (c == '\n' || c == '\r') {
      if (index > 0) {
        buffer[index] = '\0'; // Null-terminate
        q_des = atof(buffer); // Convert string to float
        index = 0; 
      }
    } else if (index < 31) {
      buffer[index++] = c;
    }
  }
}

void print_telemetry() {
  static unsigned long last_print = 0;
  
  // Print at ~5Hz
  if (millis() - last_print > 200) {
    Serial2.print("Des: "); 
    Serial2.print(q_des, 4);
    Serial2.print("\tAct: "); 
    Serial2.print(state.q_curr, 4);
    Serial2.print("\tErr: "); 
    Serial2.print(current_pos_error, 4);
    Serial2.print("\tTrqCmd(Nm): "); 
    Serial2.println(current_joint_torque, 4);
    
    last_print = millis();
  }
}