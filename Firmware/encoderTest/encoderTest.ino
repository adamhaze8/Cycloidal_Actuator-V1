#include <SimpleFOC.h>
#include <Wire.h>

// 1. Define Custom Sensor Class
class MT6701Sensor : public Sensor {
  public:
    MT6701Sensor() { cpr = 16384.0f; }

    void init() {
      // Moved Wire.begin to setup() for safety, as discussed
      this->Sensor::init();
    }

    // Manual reconstruction logic
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

// 2. Setup Sensors
MT6701Sensor mt6701; 
MagneticSensorI2C as5600 = MagneticSensorI2C(0x36, 12, 0X0C, 4); 

// 3. Mechanical Constants
const float belt_ratio = 87.0f / 26.0f; 
const float rad_to_deg = 57.29577f;

void setup() {
  Serial2.begin(115200);
  while (!Serial2);

  // Initialize I2C once globally
  Wire.begin();
  Wire.setClock(400000); 

  mt6701.init();
  as5600.init();

  Serial2.println("Raw Sensor Value Test (0-360)");
  Serial2.println("Motor_Raw\tOutput_Calc_Raw");
}

void loop() {
  // We still run update() to keep internal state valid, 
  // though strictly not required for raw reads
  mt6701.update();
  as5600.update();

  // 4. Calculate raw 0-360 positions
  
  // getMechanicalAngle() returns 0 to 2PI (0 to 6.28)
  float motor_in_deg = as5600.getMechanicalAngle() * rad_to_deg;
  
  // getSensorAngle() returns 0 to 2PI based on your class
  // Note: Because this is raw, the "Output" value will jump 
  // from ~107 (360/3.34) back to 0 every time the encoder completes a rotation.
  float output_deg = (mt6701.getSensorAngle() * rad_to_deg) / belt_ratio;

  // 5. Display Results
  static long last_print = 0;
  if (millis() - last_print > 50) {
    Serial2.print(motor_in_deg, 2);
    Serial2.print("\t\t");
    Serial2.println(output_deg, 2);
    last_print = millis();
  }
}