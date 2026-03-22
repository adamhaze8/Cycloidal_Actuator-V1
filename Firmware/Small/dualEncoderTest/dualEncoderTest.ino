#include <SimpleFOC.h>

/**
 * PROJECT BIBLE V2.2 - FINAL SENSOR VALIDATION
 * Motor: AS5047P (AB) -> PB6, PB7 (Fast Loop)
 * Joint: AS5048A (PWM) -> PA15 (Slow Loop)
 */

// 1. Motor Encoder: AS5047P
// Resolution: 1024 PPR = 4096 counts per revolution.
Encoder motor_encoder = Encoder(PB6, PB7, 1000); 

// 2. Joint Encoder: AS5048A
// Tuned for ~1kHz frequency (1000us max pulse).
MagneticSensorPWM joint_encoder = MagneticSensorPWM(PA15, 4, 1024);

const float rad_to_deg = 57.29577f;

// --- INTERRUPT HANDLERS ---
void doA() { motor_encoder.handleA(); }
void doB() { motor_encoder.handleB(); }
void doPWM() { joint_encoder.handlePWM(); }

void setup() {
  Serial2.begin(115200);
  while (!Serial2);

  // Initialize Motor Encoder
  motor_encoder.init();
  motor_encoder.enableInterrupts(doA, doB);

  // Initialize Joint Encoder
  joint_encoder.init();
  joint_encoder.enableInterrupt(doPWM);

  Serial2.println("PHD KILLER V2.2: DUAL SENSOR READ (RAW)");
  Serial2.println("Motor_Deg\tJoint_Deg");
}

void loop() {
  // Update both sensors continuously
  motor_encoder.update();
  joint_encoder.update();

  static long last_print = 0;
  if (millis() - last_print > 50) {
    float motor_pos = motor_encoder.getMechanicalAngle() * rad_to_deg;
    float raw_joint = joint_encoder.getAngle() * rad_to_deg;

    Serial2.print(motor_pos, 2);
    Serial2.print("\t\t");
    Serial2.println(raw_joint, 2); // Now prints the raw angle directly
    
    last_print = millis();
  }
}