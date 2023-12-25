#include <Servo.h>

byte intensity_pin = 9;
byte angle_pin = 8;
Servo servo_intensity;
Servo servo_angle;

uint8_t steps = 4;

uint16_t target_angle_pwm = 1500;

void setup() {
  
 Serial.begin(115200);
 servo_intensity.attach(intensity_pin);
 servo_angle.attach(angle_pin);

 servo_intensity.writeMicroseconds(1910);
 servo_angle.writeMicroseconds(target_angle_pwm);
 
 Serial.println("Send RC-PWM between 1100us and 1900us using a and d keys.");
}

void loop() {
  
  while (Serial.available() == 0);
  
  uint8_t val = Serial.read(); 

  if (val == 97) {
    target_angle_pwm -= steps; 
  }

  else if (val == 100) {
    target_angle_pwm += steps; 
  }
  
  else {
    ;
  }

  servo_angle.writeMicroseconds(target_angle_pwm);
  Serial.println(target_angle_pwm);
}
