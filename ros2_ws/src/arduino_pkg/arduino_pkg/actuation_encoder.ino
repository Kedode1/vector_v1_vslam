//including servo library for steering servo
#include <Servo.h>

Servo steering;

//Identifying pins connected to the arduino board
const int steeringPin = 11;
const int dcEnablePin = 9;
const int dcIn1 = 8;
const int dcIn2 = 7;

#define steeringPin 11

// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_in2 7  // Dir DC Motor
#define L298N_in1 8 // Dir DC Motor

#define right_encoder_phaseA 3  // Interrupt 
#define right_encoder_phaseB 5  

// ---- WHEEL CONSTANTS ----
const float wheelDiameterMM = 64.0;
const float wheelCircumferenceMM = 3.14159 * wheelDiameterMM;  // ≈ 201.06 mm
const int encoderPPR = 385;  // your encoder pulses per revolution

// ---- Distance ----
double distance_mm = 0;   // total distance
double instantaneous_distance_mm = 0;

volatile long right_encoder_counter = 0;

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

  // Convert pulses → distance (mm)
  instantaneous_distance_mm = ( (int)right_encoder_counter / encoderPPR ) * wheelCircumferenceMM;

  // Accumulate total
  distance_mm += instantaneous_distance_mm;


  //-------------------------------------------------------------------------
  //-------------------------ACTUATOR MOVEMENT LOGIC-------------------------
  //-------------------------------------------------------------------------
 
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
}