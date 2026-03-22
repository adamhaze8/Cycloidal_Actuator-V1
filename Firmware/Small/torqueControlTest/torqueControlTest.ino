#include <SimpleFOC.h>

// 1. Hardware Setup
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// --- CHANGED: Updated to AS5047P (ABI Interface) ---
// PB6/PB7 = A/B, 1024 PPR
Encoder motor_sensor = Encoder(PB6, PB7, 1000);

// Interrupt routines for high-speed commutation
void doA() { motor_sensor.handleA(); }
void doB() { motor_sensor.handleB(); }
// ----------------------------------------------------

BLDCMotor motor = BLDCMotor(14);

// 2. Current Sense Setup (Specific to B-G431B-ESC1)
// Arguments: shunt_resistor, gain, phA_pin, phB_pin, phC_pin
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f / 7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// 3. Commander for live torque tuning
Commander command = Commander(Serial2);
void onMotor(char* cmd) {
  command.motor(&motor, cmd);
}

void setup() {

  Serial2.begin(115200);
  delay(1000);

  // 4. Initialize Hardware
  motor_sensor.init();
  
  // --- ADDED: Enable Encoder Interrupts ---
  motor_sensor.enableInterrupts(doA, doB);
  // ----------------------------------------

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
  motor.linkSensor(&motor_sensor);
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
  motor.voltage_sensor_align = 4; // Voltage used for sensor alignment

  motor.useMonitoring(Serial2);
  command.add('M', onMotor, "motor");

  motor.init();
  
  // NOTE: Because this is an ABI encoder without Index (Z),
  // initFOC will rotate the motor significantly to find electrical zero.
  // KEEP HANDS CLEAR.
  motor.initFOC();
  
  Serial2.println("True Torque Control Ready. Target is in AMPS.");
}

void loop() {
  // Fast FOC execution
  motor.loopFOC();

  // Command 0.5 Amps of torque
  motor.move(0.5);

  command.run();

  static long lp = 0;
  if (millis() - lp > 500) { // Speed up print to 200ms to catch the spring effect
    lp = millis();
    
    // Read raw currents
    PhaseCurrent_s currents = currentSense.getPhaseCurrents();
    
    // --- ADDED DEBUG PRINTS ---
    Serial2.print("Deg:"); 
    Serial2.println(motor.shaft_angle * 57.29578); // Convert Radians to Degrees
  }
}