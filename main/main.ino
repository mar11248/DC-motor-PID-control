//-------PID Speed and Position Control of a DC motor-------
// Autor: Adrián Martínez Girón
// for Universidad del Valle de Guatemala

#include <PID_v1.h>

const byte encoder_A = 2; // to interrupt 0
const byte encoder_B = 3; // to digital pin 3

int M1 = 5; //to enable pin A1 of motor driver
int M2 = 4; //to enable pin A2 of motor driver

double pulse = 0;
double abs_pulse = 0; //saves how many pulses did the encoder send

double PM =0;
double RPM = 0;
double old_time=0;
double new_time=0;
double new_position=0;
double old_position=0;

boolean Read_Dir; //Saves the direction of the motor
int Set_Dir; //0=stop motor, 1=right motor, 2=left motor

double pwm; //pulse sent to the motor
double setpoint; //desired number for the PID control

double kp = 1, ki = 1, kd = 0; //constants for the PID control
PID motor(&abs_pulse, &pwm, &setpoint, kp, ki, kd, DIRECT);

void read_encoder1() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder_A) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder_B) == LOW) {
      pulse++;         // CW
    }
    else {
      pulse--;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder_B) == HIGH) {
      pulse++;          // CW
    }
    else {
      pulse--;          // CCW
    }
  }
  
}


void read_encoder2() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder_B) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder_A) == HIGH) {
      pulse++;         // CW
    }
    else {
      pulse--;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel B
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder_A) == LOW) {
      pulse++;          // CW
    }
    else {
      pulse--;          // CCW
    }
  }
  //Serial.println (pulse, DEC);
}
void setup() {
  Serial.begin(250000); //init the serial port
  setpoint = 20; //set the power
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  pinMode(encoder_A, INPUT);
  pinMode(encoder_B, INPUT);

  attachInterrupt(0, read_encoder1, CHANGE);

 attachInterrupt(1, read_encoder2, CHANGE);


}

void loop() {
  digitalWrite(M2, LOW);
  analogWrite(M1, 70);
  new_position=abs(pulse);
  new_time = micros();
  PM=((new_position-old_position)/((new_time-old_time)*0.000001))*60;
  RPM=(PM/420)/30;
  old_position=new_position;
  old_time = new_time;
  
  Serial.println(RPM);
  delay(100);


}
