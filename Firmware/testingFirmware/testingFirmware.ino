#include <SimpleFOC.h>
#include <Wire.h>

/* --- Configuration: Mechanical & Physics --- */
const float BELT_RATIO = 26.0 / 87.0;    // Joint Output -> MT6701 Sensor
const float GEARBOX_RATIO = 0.067;  // Motor -> Joint Output
const float MOTOR_KT = 0.356;

/* --- Configuration: Impedance Control --- */
float STIFFNESS = 5.0; // Kp
float DAMPING = 0.15;   // Kd

/* --- Hardware Drivers & Sensors --- */
// Custom implementation for MT6701 via I2C
class MT6701Sensor : public Sensor {
public:
  MT6701Sensor() { cpr = 16384.0f; }
  void init() {
    Wire.begin();
    Wire.setClock(400000);
    this->Sensor::init();
  }
  float getSensorAngle() override {
    Wire.beginTransmission(0x06);
    Wire.write(0x03);
    Wire.endTransmission(false);
    Wire.requestFrom(0x06, 2);
    if (Wire.available() >= 2) {
      uint8_t msb = Wire.read();
      uint8_t lsb = Wire.read();
      uint16_t raw_14bit = ((uint16_t)msb << 6) | (lsb >> 2);
      return (raw_14bit / cpr) * _2PI;
    }
    return 0;
  }
private:
  float cpr;
};

// Hardware Instantiation
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
MagneticSensorI2C motor_sensor = MagneticSensorI2C(0x36, 12, 0X0C, 4);
MT6701Sensor joint_sensor;
BLDCMotor motor = BLDCMotor(21);
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

// Global Variables
float target_pos_deg = 0.0f;
const float rad_to_deg = 57.29577f;
const float deg_to_rad = 0.0174533f;

void setup() {
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  Serial2.begin(38400);
  Wire.begin();
  delay(1000);
  Serial2.println("--- G80 IMPEDANCE BENCH ---");

  // Hardware Initialization
  motor_sensor.init();
  joint_sensor.init();
  driver.voltage_power_supply = 24;
  driver.init();

  currentSense.linkDriver(&driver);
  if (!currentSense.init()) while (1);
  
  motor.linkCurrentSense(&currentSense);
  motor.linkSensor(&motor_sensor);
  motor.linkDriver(&driver);

  // FOC & Controller Configuration
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::foc_current;

  motor.PID_current_q.P = 3; 
  motor.PID_current_q.I = 300;
  motor.PID_current_d.P = 3; 
  motor.PID_current_d.I = 300;
  motor.LPF_current_q.Tf = 0.005f;
  motor.LPF_current_d.Tf = 0.005f;

  motor.current_limit = 8.0;
  motor.voltage_sensor_align = 4;
  motor.voltage_limit = 12;
  motor.useMonitoring(Serial2);

  motor.init();
  motor.initFOC();

  // Set initial target to current position to prevent jumps
  joint_sensor.update();
  target_pos_deg = (joint_sensor.getAngle() * BELT_RATIO) * rad_to_deg;
  Serial2.print("Hold Position: ");
  Serial2.println(target_pos_deg);
}

void loop() {
  motor.loopFOC();
  motor_sensor.update();
  joint_sensor.update();

  /* --- State Estimation --- */
  // Position: Derived from joint sensor (Belt Ratio)
  float q_curr = joint_sensor.getAngle() * BELT_RATIO;

  // Velocity: Derived from motor sensor (Gearbox Ratio) for higher resolution
  float dq_curr = motor.shaft_velocity * 0.067;

  /* --- Impedance Control Law --- */
  float q_des = target_pos_deg * deg_to_rad;
  float pos_error = q_des - q_curr;
  float vel_error = 0 - dq_curr; // Target velocity is 0

  float torque_nm = (STIFFNESS * pos_error) + (DAMPING * -vel_error);
  float current_cmd = torque_nm / MOTOR_KT;

  // Safety Clamp (+/- 8A)
  if (current_cmd > 8.0) current_cmd = 8.0;
  if (current_cmd < -8.0) current_cmd = -8.0;

  motor.move(-current_cmd); // Inverted direction

  /* --- Serial Command Parsing --- */
  if (Serial2.available() > 0) {
    float new_target = Serial2.parseFloat();
    while (Serial2.available()) Serial2.read();
    if (new_target > -360 && new_target < 360) target_pos_deg = new_target;
  }

  /* --- Telemetry (1Hz) --- */
  static long lp = 0;
  if (millis() - lp > 1000) {
    lp = millis();
    Serial2.print(target_pos_deg);
    Serial2.print(" \t ");
    Serial2.print(q_curr * rad_to_deg);
    Serial2.print(" \t ");
    Serial2.println(current_cmd);
  }
}