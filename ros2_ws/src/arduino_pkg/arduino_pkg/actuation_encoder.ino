//including servo library for steering servo
#include <Servo.h>
#include <String>
#include <Arduino_LSM6DSOX.h>

Servo steering;

#define steeringPin 11

// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_in2 7  // Dir DC Motor
#define L298N_in1 8 // Dir DC Motor

#define right_encoder_phaseA 14  // Interrupt 
#define right_encoder_phaseB 15  

// ---- WHEEL CONSTANTS ----
const float wheelDiameterMM = 64.0;
const float wheelCircumferenceMM = 3.14159 * wheelDiameterMM;  // ≈ 201.06 mm
const int encoderPPR = 44;  // your encoder pulses per revolution

// ---- Distance ----
double distance_mm = 0;   // total distance
double instantaneous_distance_mm = 0;

volatile long right_encoder_counter = 0;

float Ax, Ay, Az;
float Gx, Gy, Gz;




void setup() {

  // Set driver pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);

  //Attach servo and write 95 (mid position)
  steering.attach(steeringPin);
  steering.write(95);
  
  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, LOW);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);
  while(!Serial);
  IMU.begin()

  print("Accelerometer sample rate = ");

  print(IMU.accelerationSampleRate());

  print("Hz\n");

  print("Gyroscope sample rate = ");  

  print(IMU.gyroscopeSampleRate());

  println("Hz\n");


  pinMode(right_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
}

/*
For the main loop it is like a communication method established between arduino and rpi
So the arduino expects message or command like this
for servo -> "Servo:<Val>"
for dc -> "DC:<Direction>:<Val>" and for direction its whether BACKWARD, FORWARD, or STOP
*/

void loop() {

  int motor_speed = 0;
  int servo_angle = 95;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);
  }


  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(Gx, Gy, Gz);
  }

  // Accumulate total
  distance_mm += instantaneous_distance_mm;
  instantaneous_distance_mm = 0.0;

  if(Serial.available() > 0){
    String command = Serial.readStringUntil('\n');

    int commaIndex = command.indexOf(',');
    if (commaIndex != -1){
      String part1 = command.substring(0, commaIndex);   
      String part2 = command.substring(commaIndex + 1);  
    
      int colonIndex1 = part1.indexOf(':');
      if (colonIndex1 != -1) {
        motor_speed = part1.substring(colonIndex1 + 1).toInt();
      }

      int colonIndex2 = part2.indexOf(':');
      if (colonIndex2 != -1) {
        servo_angle = part2.substring(colonIndex2 + 1).toInt();
      }
    }
    
    servo_angle = constrain(servo_angle, 0, 180);
    steering.write(servo_angle);

    int speed_analog = map(abs(motor_speed), 0, 10, 0, 255);
    speed_analog = constrain(speed_analog, 0, 255);

    analogWrite(L298N_enA, speed_analog); 
    if(motor_speed > 0){ 
      digitalWrite(L298N_in1, HIGH);
      digitalWrite(L298N_in2, LOW);
    }
    else if(motor_speed < 0){ 
      digitalWrite(L298N_in1, LOW);
      digitalWrite(L298N_in2, HIGH);
    }
    else{
      digitalWrite(L298N_in1, LOW);
      digitalWrite(L298N_in2, LOW);
    }
  }

  String command_to_be_send_to_rpi = "Distance_mm:" + String(distance_mm) + ",instantaneous_distance_mm:" + String(instantaneous_distance_mm) +"\n";
  Serial.print(command_to_be_send_to_rpi);
  delay(10);
}

void rightEncoderCallback()
{
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_encoder_counter++;
  }
  else
  {
    right_encoder_counter--;
  }
    // Convert pulses → distance (mm)
  instantaneous_distance_mm = ( (int)right_encoder_counter / encoderPPR ) * wheelCircumferenceMM;
}