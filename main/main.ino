//------- PID Speed and Position Control of a DC motor -------
// Autor: Adrián Martínez Girón
// for Universidad del Valle de Guatemala

//#define DEBUG
#include <PID_v1.h>

#define SAMPLE_DELAY (5)   //  this gets 1 reading per second.
//  adjust the delay to fit your needs
#define PULSES_PER_TURN (420)  //  32 state changes per turn on 1 line, 

unsigned int lastTime; //Use for speed

boolean mode = true; //change to false for speed control
int cont = 0;
const byte encoder_A = 2; // to interrupt 0
const byte encoder_B = 3; // to digital pin 3

int M1 = 5; //to enable pin A1 of motor driver
int M2 = 4; //to enable pin A2 of motor driver

double pulse = 0;
double abs_pulse = 0; // saves how many pulses did the encoder send

double PM = 0;        // pulses per minute
double RPM = 0;       // revolutions per minute

double old_time = 0;         // get time differential
double new_time = 0;
double new_position = 0;  // get position differential
double old_position = 0;

String inString = "";
int incomingByte = 0; //get byte from serial
int input = 0;  // serial number saved

double pwm = 0; //pulse sent to the motor
double setpoint; //desired number for the PID control
double angle = 0; // motor position in DEG

double kp1 = 0.16617, ki1 = 1.3085, kd1 = 0; //constants for the PID Speed control
double kp = 0.3, ki = 0.4, kd = 0.1; //constants for the PID Position control

PID motor(&pulse, &pwm, &setpoint, kp, ki, kd, DIRECT);   // PID for the position
PID motor1(&RPM, &pwm, &setpoint, kp1, ki1, kd1, DIRECT); // PID for the speed

void setup() {
  if (mode) {
    motor.SetOutputLimits(20, 250);
  }
  if (!mode) {
    motor1.SetOutputLimits(0, 255);
  }
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise
  Serial.begin(250000); //init the serial port

  pinMode(M1, OUTPUT); // set output pins for motor control
  pinMode(M2, OUTPUT);

  pinMode(encoder_A, INPUT);  // set encoder inputs
  pinMode(encoder_B, INPUT);

  motor.SetMode(AUTOMATIC);
  motor1.SetMode(AUTOMATIC);
  motor.SetSampleTime(100);
  motor1.SetSampleTime(100);

  attachInterrupt(0, read_encoder1, CHANGE);   // one interrupt enabled, if we use both we can
  //attachInterrupt(1, read_encoder2, CHANGE); // get the full encoder resolution, instead of
  // half the resolution.
}

void loop() {
  // Read the input from serial
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      input = inString.toInt();
      // clear the string for new input:
      inString = "";
    }
  }
  // ----------- Position control -----------
  if (mode) {
    setpoint = mapfloat(input, 0, 360, 0, 420);
    if (pulse < (setpoint - 1)) {
      motor.Compute();
      motor.SetControllerDirection(DIRECT);
      advance((int)pwm);
    }
    else if (pulse > (setpoint + 1)) {
      motor.Compute();
      int pwm1 = abs(pwm);
      motor.SetControllerDirection(REVERSE);
      back((int)pwm1);
    } else {
      Stop();
    }
    noInterrupts(); // disable interrutps between reads to avoid reading while the value change
    new_position = abs(pulse);
    interrupts(); // enable interrupts and continue
    angle = map (new_position, 0, 420, 0, 360);
  }
  // ----------- Speed Control -----------
  if (!mode) {
    setpoint = input;
    motor1.Compute();
    advance(pwm);
    
    if ((unsigned int)millis() - lastTime >= SAMPLE_DELAY)
    {
      RPM = (abs(pulse) * (60000.f / ((unsigned int)millis() - lastTime))) / PULSES_PER_TURN;
      pulse = 0;
      lastTime = (unsigned int)millis();
    }
// debug routine for tunning PID control in speed
#ifdef DEBUG
    double P = analogRead(A2);
    double I = analogRead(A1);
    double D = analogRead(A0);
    kp1 = mapfloat(P, 0, 1024, 0.01, 20);
    ki1 = mapfloat(I, 0, 1024, 0, 20);
    kd1 = mapfloat(D, 0, 1024, 0, 1);
    motor1.SetTunings(kp1, ki1, kd1);

    Serial.print(kp1, DEC);
    Serial.print(",");
    Serial.print(ki1, DEC);
    Serial.print(",");
    Serial.print(kd1, DEC);
    Serial.print(",");
    Serial.println(RPM);
#endif
  }
}

// Motor direction routines
void advance(int pwme)//Motor Forward
{
  digitalWrite(M2, LOW);
  analogWrite(M1, pwme);
}
void back(int pwme)//Motor reverse
{
  digitalWrite(M2, HIGH);
  analogWrite(M1, pwme);
}

void Stop()//Motor stops
{
  digitalWrite(M1, LOW);
  digitalWrite(M2, LOW);
}

//our map function for float
double mapfloat(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (double)(x - in_min) * (out_max - out_min) / (double)(in_max - in_min) + out_min;
}

// encoder reading routine
void read_encoder1() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder_A) == digitalRead(encoder_B)) {
    pulse++;
  } else {
    pulse--;
  }
}
