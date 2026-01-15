#include <SimpleFOC.h>
#include <Wire.h>

// --- 1. Custom MT6701 Class ---
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

// --- 2. Hardware Instances ---
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
MagneticSensorI2C as5600 = MagneticSensorI2C(0x36, 12, 0X0C, 4);
MT6701Sensor mt6701;
BLDCMotor motor = BLDCMotor(21);

// --- 3. Configuration ---
// Ratio: Output turns slower than the MT6701
const float BELT_RATIO = 87.0f / 26.0f;     
const float rad_to_deg = 57.29577f;

float target_output_deg = 90.0f; 

// PID Tuning (Position Loop)
PIDController pid_pos{1.4, 0.01, 0.15, 100, 50}; 

void setup() {
  Serial2.begin(115200);
  Wire.begin();
  
  // 1. Initialize Sensors
  as5600.init();
  mt6701.init();

  // 2. Initialize Driver
  driver.voltage_power_supply = 24;
  driver.init();
  
  // 3. Link Motor (AS5600 handles commutation)
  motor.linkSensor(&as5600);
  motor.linkDriver(&driver);
  
  // 4. Controller Settings
  motor.controller = MotionControlType::velocity; // Position loop runs outside
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 10;
  motor.voltage_limit = 12;
  motor.velocity_limit = 50; 
  
  // 5. Initialize FOC
  motor.init();
  motor.initFOC();

  Serial2.println("--- SYSTEM READY (SIMPLE MODE) ---");
  Serial2.println("Actuator Zero aligned to MT6701 Absolute Zero.");
  delay(500);
}

void loop() {
  // 1. FOC Routine (Commutation)
  motor.loopFOC(); 

  // 2. Sensor Tracking (CRITICAL)
  // You MUST call update() to track rotations beyond 360 degrees
  as5600.update();
  mt6701.update();

  // 3. Calculate Absolute Output Position
  // getAngle() returns total accumulated radians (absolute start + full turns)
  // We divide by BELT_RATIO to get the final actuator angle.
  // Kept the (-) sign from your previous code to maintain direction.
  float current_output_deg = (mt6701.getAngle() * rad_to_deg) / BELT_RATIO;
  
  // 4. Position Control Loop
  float error = target_output_deg - current_output_deg;
  float vel_cmd = pid_pos(error);
  motor.move(-vel_cmd);

  // 5. Serial Monitoring
  static long lp = 0;
  if (millis() - lp > 50) {
    // Parse new targets
    if(Serial2.available() > 0) {
      target_output_deg = Serial2.parseFloat();
      while(Serial2.available() > 0) Serial2.read(); // Flush buffer
    }
    
    Serial2.print("T:"); Serial2.print(target_output_deg, 2);
    Serial2.print(" | O:"); Serial2.print(current_output_deg, 2);
    Serial2.print(" | V:"); Serial2.println(vel_cmd, 2);
    lp = millis();
  }
}