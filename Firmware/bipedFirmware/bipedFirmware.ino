#include <SimpleFOC.h>
#include <Wire.h>

// --- 1. SENSORS ---
// (Keep your MT6701 Class exactly as is)
class MT6701Sensor : public Sensor {
  public:
    MT6701Sensor() { cpr = 16384.0f; }
    void init() { Wire.begin(); Wire.setClock(400000); this->Sensor::init(); }
    float getSensorAngle() override {
      Wire.beginTransmission(0x06); Wire.write(0x03); Wire.endTransmission(false);
      Wire.requestFrom(0x06, 2);
      if (Wire.available() >= 2) {
        uint8_t msb = Wire.read(); uint8_t lsb = Wire.read();
        uint16_t raw_14bit = ((uint16_t)msb << 6) | (lsb >> 2);
        return (raw_14bit / cpr) * _2PI;
      }
      return 0;
    }
  private: float cpr;
};

// --- 2. HARDWARE INSTANCES ---
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
MagneticSensorI2C motor_sensor = MagneticSensorI2C(0x36, 12, 0X0C, 4); // AS5600 for commutation
MT6701Sensor joint_sensor; // MT6701 for joint angle
BLDCMotor motor = BLDCMotor(21); 
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// --- 3. CONFIGURATION ---
const float BELT_RATIO = 87.0f / 26.0f;
// Torque Constant (Nm/A) - CRITICAL for converting Sim Torque to Real Amps
// You must measure this or find it in motor datasheet. Example: 0.1 Nm/A
const float MOTOR_KT = 0.1; 

// Robot State
float q_curr = 0; // Current Joint Position (rad)
float dq_curr = 0; // Current Joint Velocity (rad/s)

// RL Commands (These would come from Serial/CAN)
float q_des = 0; // Target Angle
float dq_des = 0; // Target Velocity
float kp = 2.0;   // Stiffness
float kd = 0.1;   // Damping
float tau_ff = 0; // Feed-forward torque

void setup() {
  Serial2.begin(115200);
  Wire.begin();

  // A. Hardware Init
  motor_sensor.init();
  joint_sensor.init();
  driver.voltage_power_supply = 24;
  driver.init();

  // B. Current Sense Init (From Script 1)
  currentSense.linkDriver(&driver);
  currentSense.init();
  motor.linkCurrentSense(&currentSense);

  // C. Motor Linkage
  motor.linkSensor(&motor_sensor);
  motor.linkDriver(&driver);

  // D. FOC Mode: TORQUE CONTROL
  // We use FOC to control torque, and we calculate the torque magnitude manually in loop()
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;

  // E. PID for Current (Tune these!)
  motor.PID_current_q.P = 3; 
  motor.PID_current_q.I = 300;
  motor.LPF_current_q.Tf = 0.005f; 

  motor.current_limit = 4.0; // Safety
  motor.init();
  motor.initFOC();
}

void loop() {
  // 1. Run FOC (Commutation)
  motor.loopFOC();

  // 2. Read Sensors
  motor_sensor.update();
  joint_sensor.update();
  
  // Calculate Joint State
  // Note: We use the Joint Sensor for Position, but often Motor Sensor for Velocity (cleaner signal)
  q_curr = joint_sensor.getAngle(); 
  dq_curr = motor.shaft_velocity / BELT_RATIO; 

  // 3. IMPEDANCE CONTROL LAW (The "RL Bridge")
  // Torque = Kp*(Pos_Err) + Kd*(Vel_Err) + FF_Torque
  float pos_error = q_des - q_curr;
  float vel_error = dq_des - dq_curr;
  
  float torque_command_nm = (kp * pos_error) + (kd * vel_error) + tau_ff;

  // 4. Convert to Amps
  float current_command = torque_command_nm / MOTOR_KT;

  // 5. Command Motor
  motor.move(current_command);

  // 6. Communication (Receive new q_des, kp, kd from PC here)
  command_interface();
}

void command_interface() {
    // Implement Serial/CAN parsing here to update q_des, kp, kd
}