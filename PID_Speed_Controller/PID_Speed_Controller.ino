/************************************************************************
MIT License
Copyright © 2020 Raj Shinde
Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the “Software”), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
The above copyright notice and this permission notice shall be included 
in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*************************************************************************/

#include<Servo.h>
 
const int encA1 = 2;         // M1 encoder pins
const int encB1 = 3;         // M1 encoder pins
const int encA2 = 19;        // M2 encoder pins
const int encB2 = 18;        // M2 encoder pins 
const double ppr = 7;        // Pulse per rotation for encoder
const double reduction = 71; // Gearbox Reduction,change as per requirement
int total1 = 0;
int total2 = 0;
double kp = 2;               // Proportional Constant
double ki = 1;               // Integral Constant
double kd = 2;               // Derivative Constant 
double errorSum1 = 0;
double errorSum2 = 0;
double prevError1 = 0;
double prevError2 = 0;
double targetRpm1 = 0;
double targetRpm2 = 0;
double tick1 = 0;           // Encoder1 ticks in 0.1 seconds
double tick2 = 0;           // Encoder2 ticks in 0.1 seconds
int dir1 = 0;               // Motor1 Direction
int dir2 = 0;               // Motor2 Direction
Servo sb1;
Servo sb2;

void setup() {
  sb1.attach(5);
  sb2.attach(6);
  sb1.writeMicroseconds(1500);
  sb2.writeMicroseconds(1500);
  pinMode(encA1,INPUT_PULLUP);
  pinMode(encB1,INPUT_PULLUP);
  pinMode(encA2,INPUT_PULLUP);
  pinMode(encB2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encA1), encoderCallback1, RISING);
  attachInterrupt(digitalPinToInterrupt(encA2), encoderCallback2, RISING);
  Serial.begin(115200);
  // Setting ISR to be called after every 0.1 Seconds
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 59286;            // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
}

void loop() {
  // Testing with Target Speeds
  targetRpm1 = 70;
  targetRpm2 = 70;
  Serial.println("set both Motors to 70 RPM");
  delay(5000);
  targetRpm1 = 50;
  targetRpm2 = 50;
  Serial.println("set both Motors to 50 RPM");
  delay(5000);
  targetRpm1 = 30;
  targetRpm2 = 30;
  Serial.println("set both Motors to 30 RPM");
  delay(5000);
  targetRpm1 = -30;
  targetRpm2 = -30;
  Serial.println("set both Motors to -30 RPM");
  delay(5000);
  targetRpm1 = -50;
  targetRpm2 = -50;
  Serial.println("set both Motors to -50 RPM");
  delay(5000);
  targetRpm1 = -70;
  targetRpm2 = -70;
  Serial.println("set both Motors to -70 RPM");
  delay(5000);
}

void encoderCallback1() {
  tick1 += 1;   // Increment encoder1 count on interrupt
  total1 += 1;  // total encoder counts, will reset after limit exceeds
  dir1 =  digitalRead(encB1);
  // Set Direction as 1 or -1
  if (dir1 == 0) 
    dir1 = -1;
}

void encoderCallback2() {
  tick2 += 1;   // Increment encoder2 count on interrupt
  total2 += 1;  // total encoder counts, will reset after limit exceeds
  dir2 =  digitalRead(encB2);
  // Set Direction as 1 or -1
  if (dir2 == 0) 
    dir2 = 1;
  else 
    dir2=-1;
}

int computePid1(const double &currentRpm, const double &setpoint) {
  double error = setpoint - currentRpm;
  int processValue = kp*error + ki*errorSum1 + kd*(error-prevError1);
  errorSum1 += error;
  prevError1 = error;
  // Clamp output to 1000us-2000us (Sabertooth input PWM range)
  if (errorSum1 > 2000) errorSum1 = 2000;
  if (errorSum1 < 1000) errorSum1 = 1000;
  return processValue;
}

int computePid2(const double &currentRpm, const double &setpoint) {
  double error = setpoint - currentRpm;
  int processValue = kp*error + ki*errorSum2 + kd*(error-prevError2);
  errorSum2 += error;
  prevError2 = error;
  // Clamp output to 1000us-2000us (Sabertooth input PWM range)
  if (errorSum2 >2000) errorSum2 = 2000;
  if (errorSum2 <1000) errorSum2 = 1000;
  return processValue;
}

ISR(TIMER1_OVF_vect)        // Service Routine to run after every 0.1 second
{
   TCNT1 = 59286;

  // Calculate current revs and RPM
  double revolution1 = tick1/ppr;
  double currentRpm1 = 600*revolution1*dir1;
  double revolution2 = tick2/ppr;
  double currentRpm2 = 600*revolution2*dir2;
  
  Serial.print("currentRpm 1: ");
  Serial.print(currentRpm1/71);
  Serial.print(" ");
  Serial.print("targetRpm 1: ");
  Serial.println(targetRpm1);

  Serial.print("currentRpm 2: ");
  Serial.print(currentRpm2/71);
  Serial.print(" ");
  Serial.print("targetRpm 2: ");
  Serial.println(targetRpm2);
  
  tick1 = 0;                // reset encoder counts
  tick2 = 0;                // reset encoder counts
  // Run PID
  int pwm1=computePid1((currentRpm1/71), targetRpm1);
  int pwm2=computePid2((currentRpm2/71), targetRpm2);
  
  Serial.print("PWM 1: ");
  Serial.println(pwm1);
  
  Serial.print("PWM 2: ");
  Serial.println(pwm2);

  // Write RPM to motor1
  if (targetRpm1 == 0) {
    sb1.writeMicroseconds(1500);
  }
  else if (pwm1 < 2000 && pwm1 >1000) {
    sb1.writeMicroseconds(pwm1);
  }
  else{
    if (pwm1>2000) {
      sb1.writeMicroseconds(2000);
    }
    else if (pwm1<1000) {
      sb1.writeMicroseconds(1000);
    }
  }

  // Write RPM to motor2
  if (targetRpm2 == 0) {
    sb2.writeMicroseconds(1500);
  }
  else if (pwm2 < 2000 && pwm2 >1000) {
    sb2.writeMicroseconds(pwm2);
  }
  else{
    if (pwm2>2000) {
      sb2.writeMicroseconds(2000);
    }
    else if (pwm2<1000) {
      sb2.writeMicroseconds(1000);
    }
  }
}


