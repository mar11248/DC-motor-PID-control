//------- PID Speed and Position Control of a DC motor -------
// Autor: Adrián Martínez Girón
// for Universidad del Valle de Guatemala

#include <PID_v1.h>
#include "nunchuk.h"


#define SAMPLE_DELAY (20)   //  this gets 1 reading per X milliseconds.
//  adjust the delay to fit your needs
#define PULSES_PER_TURN (1311)  //  1311 state changes per turn on 1 line, 

int x, y;
int b1, b2;

double angle=0;

unsigned int lastTime; //Use for speed

boolean mode = true; //change to false for speed control
const byte encoder_A = 2; // to interrupt 0
const byte encoder_B = 3; // to digital pin 3

uint8_t M1 = 5; //to enable pin A1 of motor driver
uint8_t M2 = 4; //to enable pin A2 of motor driver

uint16_t input = 0;

volatile double pulse = 0;
volatile uint16_t abs_pulse = 0; // saves how many pulses did the encoder send

double RPM = 0;       // revolutions per minute

float new_position = 0;  // get position differential

double pwm = 0; //pulse sent to the motor
double setpoint; //desired number for the PID control

double kp1 = 0.16617, ki1 = 1.3085, kd1 = 0; //constants for the PID Speed control
double kp = 0.1, ki = 0.7 , kd = 0; //constants for the PID Position control
PID motor(&pulse, &pwm, &setpoint, kp, ki, kd, DIRECT);   // PID for the position
PID motor1(&RPM, &pwm, &setpoint, kp1, ki1, kd1, DIRECT); // PID for the speed

void setup() {
  Wire.begin();
  nunchuk_init();
  if (mode) {
    motor.SetOutputLimits(0, 200);
  }
  if (!mode) {
    motor1.SetOutputLimits(0, 255);
  }
  pinMode(A0, INPUT); // Control potentiometer input

  pinMode(10, INPUT_PULLUP); // Mode change input

  TCCR1B = TCCR1B & 0b11111000 | 1;  // set 31KHz PWM to prevent motor noise


  pinMode(M1, OUTPUT); // set output pins for motor control
  pinMode(M2, OUTPUT);

  pinMode(encoder_A, INPUT);  // set encoder inputs
  pinMode(encoder_B, INPUT);

  motor.SetMode(AUTOMATIC);  // Settings for the PID controllers
  motor1.SetMode(AUTOMATIC);
  motor.SetSampleTime(100);
  motor1.SetSampleTime(100);

  attachInterrupt(0, doEncoderA, CHANGE);   // one interrupt enabled, if we use both we can
  attachInterrupt(1, doEncoderB, CHANGE); // get the full encoder resolution, instead of
  // half the resolution.
}

void loop() {
  if (nunchuk_read()) {
    // Work with nunchuk_data
    x = nunchuk_joystickX();
    y = nunchuk_joystickY();
    b1 = nunchuk_buttonZ();
    b2 = nunchuk_buttonC();

  }
  if (b1 == 1) {
    mode = !mode;
    delay(250);
  }


  // ----------- Position control -----------
  if (mode) {

    // setpoint = mapfloat(input, 0, 1024, 0, PULSES_PER_TURN);
    angle=atan(x/y); 
    if(((x>-40)&&(x<40))||((y>-40)&&(y<40))){
      setpoint=setpoint;
    }
    else{
          setpoint = mapfloat(angle, 0, 6.28319, 0, PULSES_PER_TURN);

    }
    
    motor.Compute();

    if (pulse < (setpoint - 1)) {
      motor.SetControllerDirection(DIRECT);
      back((int)pwm);
    }
    else if (pulse > (setpoint + 1)) {
      int pwm1 = abs(pwm);
      motor.SetControllerDirection(REVERSE);
      advance((int)pwm1);
    } else {
      Stop();
    }
    noInterrupts(); // disable interrutps between reads to avoid reading
    // while the value change
    new_position = abs(pulse);
    interrupts(); // enable interrupts and continue


  }
  // ----------- Speed Control -----------
  if (!mode) {
    input = analogRead(A0);
    // setpoint = map(input, 0, 1024, 0, 500);
    setpoint = map(y, 10, 90, 0, 500);
    motor1.Compute();
    advance(pwm);

    if (((unsigned int)millis() - lastTime) >= SAMPLE_DELAY)
    {
      noInterrupts(); // disable interrutps between reads to avoid reading
      // while the value change
      int abs_pulse = abs(pulse);
      interrupts(); // enable interrupts and continue
      RPM = (abs_pulse * (60000.f / ((unsigned int)millis() - lastTime))) / PULSES_PER_TURN;
      pulse = 0;
      lastTime = (unsigned int)millis();
    }
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
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder_A) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder_B) == LOW) {
      pulse = pulse + 1;         // CW
    }
    else {
      pulse = pulse - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder_B) == HIGH) {
      pulse = pulse + 1;          // CW
    }
    else {
      pulse = pulse - 1;          // CCW
    }
  }

  // use for debugging - remember to comment out
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder_B) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder_A) == HIGH) {
      pulse = pulse + 1;         // CW
    }
    else {
      pulse = pulse - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder_A) == LOW) {
      pulse = pulse + 1;          // CW
    }
    else {
      pulse = pulse - 1;          // CCW
    }
  }
}
