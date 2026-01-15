#include <SimpleFOC.h>
#include <Wire.h>

// 1. Custom MT6701 Class (Manual 14-bit reconstruction)
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
  private:
    float cpr;
};

// 2. Hardware Setup
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
MagneticSensorI2C as5600 = MagneticSensorI2C(0x36, 12, 0X0C, 4);
MT6701Sensor mt6701;
BLDCMotor motor = BLDCMotor(21);

// 3. Variables
const float rad_to_deg = 57.29577f;
bool is_test_active = false;
float start_angle_as5 = 0;
float start_angle_mt6 = 0; // Added to fix the MT6_Accum offset
float target_velocity = 4.0f; 

void setup() {
  pinMode(PC13, OUTPUT); 
  digitalWrite(PC13, HIGH);
  Serial2.begin(115200);

  as5600.init();
  mt6701.init();

  driver.voltage_power_supply = 24;
  driver.dead_zone = 0.05f;
  driver.init();

  motor.linkSensor(&as5600);
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::velocity;

  motor.phase_resistance = 1.8; 
  motor.current_limit = 3.0;
  motor.PID_velocity.P = 0.2f; 
  motor.PID_velocity.I = 10.0f;
  motor.LPF_velocity.Tf = 0.02f; 
  motor.voltage_limit = 12;      
  motor.velocity_limit = 70;

  motor.init();
  motor.initFOC();

  Serial2.println("FOC Ready. Send 'S' to start automated 15-turn test.");
}

void loop() {
  motor.loopFOC();

  if (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == 'S' && !is_test_active) {
      // Capture both start angles to zero the Accum columns
      as5600.update();
      mt6701.update();
      start_angle_as5 = as5600.getAngle();
      start_angle_mt6 = mt6701.getAngle();
      
      is_test_active = true;
      Serial2.println("Motor_Raw,MT6_Raw,Motor_Accum,MT6_Accum");
    }
  }

  if (is_test_active) {
    motor.move(target_velocity);
    
    as5600.update();
    mt6701.update();

    // Calculate relative accumulated movement
    float motor_accum = (as5600.getAngle() - start_angle_as5) * rad_to_deg;
    float mt6_accum = (mt6701.getAngle() - start_angle_mt6) * rad_to_deg;
    
    // Raw angles (0-360) remain absolute to the sensor disk
    float motor_raw = fmod(as5600.getAngle() * rad_to_deg, 360.0);
    if (motor_raw < 0) motor_raw += 360.0;
    
    float mt6_raw = fmod(mt6701.getAngle() * rad_to_deg, 360.0);
    if (mt6_raw < 0) mt6_raw += 360.0;

    // Stream CSV data
    Serial2.print(motor_raw, 3);    Serial2.print(",");
    Serial2.print(mt6_raw, 3);       Serial2.print(",");
    Serial2.print(motor_accum, 3);  Serial2.print(",");
    Serial2.println(mt6_accum, 3);

    // Stop after 15 motor revolutions (5400 degrees)
    if (abs(motor_accum) >= 5400.0f) {
      is_test_active = false;
      motor.move(0);
      Serial2.println("STOP: Test Complete.");
    }
  } else {
    motor.move(0);
  }
}