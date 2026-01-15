#include <SimpleFOC.h>

// 1. Hardware Setup
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0X0C, 4);
BLDCMotor motor = BLDCMotor(21);

// 2. Current Sense Setup (Specific to B-G431B-ESC1)
// Arguments: shunt_resistor, gain, phA_pin, phB_pin, phC_pin
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// 3. Commander for live torque tuning
Commander command = Commander(Serial2);
void onMotor(char* cmd) {
  command.motor(&motor, cmd);
}

void setup() {
  pinMode(PC13, OUTPUT);  // Wake up gate driver
  digitalWrite(PC13, HIGH);

  Serial2.begin(115200);
  delay(1000);

  // 4. Initialize Hardware
  sensor.init();
  driver.voltage_power_supply = 24;
  driver.init();

  // 5. Initialize Current Sensing
  currentSense.linkDriver(&driver);
  if (!currentSense.init()) {
    Serial2.println("Current Sense Init Failed!");
    return;
  }
  motor.linkCurrentSense(&currentSense);

  // 6. Motor Configuration
  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);

  // Set to FOC Current mode (True Torque Control)
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;

  // PID for the Current Loops (Internal FOC loops)
  // These are standard values for the B-G431B-ESC1
  motor.PID_current_q.P = 3;
  motor.PID_current_q.I = 300;
  motor.PID_current_d.P = 3;
  motor.PID_current_d.I = 300;
  motor.LPF_current_q.Tf = 0.005f;
  motor.LPF_current_d.Tf = 0.005f;

  // 7. Safety Limits
  motor.current_limit = 3.0;  // Absolute limit in Amps
  motor.voltage_limit = 12;   // Voltage cap for the current PID
  motor.voltage_sensor_align = 4;

  motor.useMonitoring(Serial2);
  command.add('M', onMotor, "motor");

  motor.init();
  motor.initFOC();
  Serial2.println("True Torque Control Ready. Target is in AMPS.");
}

void loop() {
  // Fast FOC execution
  motor.loopFOC();

  // --- IMPEDANCE CONTROL ---
  float q_des = target_pos_deg * deg_to_rad;

  float pos_error = q_des - q_curr;
  float vel_error = 0 - dq_curr;

  float torque_nm = (STIFFNESS * pos_error) + (DAMPING * vel_error);
  float current_cmd = torque_nm / MOTOR_KT;

  // Safety Clamp
  if (current_cmd > 8.0f) current_cmd = 8.0f;
  if (current_cmd < -8.0f) current_cmd = -8.0f;


  // Command 0.5 Amps of torque
  // With Kt = 0.356, this is ~0.178 Nm of torque
  motor.move(0.);

  command.run();
}