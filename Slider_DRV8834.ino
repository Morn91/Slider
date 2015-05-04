#include <AccelStepper.h>
#include <TimerOne.h>

#define STEP 5
#define DIR 6
#define M0 7
#define M1 8
#define BATTERY 3
#define MOTOR_MAX 750
#define DRIVER_MAX 10000

AccelStepper stepper(AccelStepper::DRIVER, STEP, DIR);

float path, vRatio, motorSpeed = 0, elapsed = 0;
int accelType = 0, dir = 0, micRes[2] = {0, 1};
unsigned long time = 0, start = 0;

void setup() {
  path = 900 / 2 / 14 * 200;
  //path (steps) = path (mm) / timing belt pitch (mm) / teeths on pulley * steps per turnover;
  vRatio = 1;
  Serial.begin(9600);
  stepper.setMaxSpeed(DRIVER_MAX);
  stepper.setSpeed(0);
  Timer1.initialize(20);
  Timer1.attachInterrupt(runByTimer);
}

void loop() {
  if (start)
    work();
  else
    tuning();
}

void runByTimer() {
  stepper.runSpeed();
}

void tuning() {
  if(Serial.available() > 0) {
    char val = Serial.read();
    switch(val) {
    case '*':
      report();
      break;
    case '+':
      delay(100);
      char buffer[20];
      int i = 0;
      while(Serial.available() && i < 19)
        buffer[i++] = Serial.read();
      buffer[i++] = '\0';
      sscanf(buffer, "%d %d %d", &dir, &accelType, &time);
      if((dir == 0 || dir == 1) && ((accelType == 0 && ceil(path / time) <= MOTOR_MAX) || (accelType >= 1 && accelType <= 4 && ceil(path / time * 2) <= MOTOR_MAX))) {
        digitalWrite(13, HIGH);
        work();
      }
      break;
    }
  }
}

void work() {
  if(Serial.available() > 0) {
    char val = Serial.read();
    if(val == '-') {
      stop();
      return;
    }
    if(val == '*')
      report();
  }
  if(start == 0) {
    start = millis();
  }
  elapsed = (millis() - start) / 1000.0;
  if(elapsed < time) {
    switch(accelType) {
    case 0:
      motorSpeed = path / time;
      break;
    case 1:
      motorSpeed = 2.0 * path * elapsed / time / time;
      break;
    case 2:
      motorSpeed = 2.0 * path * (time - elapsed) / time / time;
      break;
    case 3:
      if(elapsed * 2.0 < time)
        motorSpeed = 4.0 * path * elapsed / time / time;
      else
        motorSpeed = 4.0 * path * (time - elapsed) / time / time;
      break;
    case 4:
      if(elapsed * 2.0 < time)
        motorSpeed = 4.0 * path * (time / 2.0 - elapsed) / time / time;
      else
        motorSpeed = 4.0 * path * (elapsed - time / 2.0) / time / time;
      break;
    default:
      stop();
      return;
    }
    while(motorSpeed * micRes[1] * 2 <= DRIVER_MAX && micRes[1] < 32)
      micRes[1] *= 2;
    while(motorSpeed * micRes[1] > DRIVER_MAX && micRes[1] > 1)
      micRes[1] *= 0.5;
    if(micRes[0] != micRes[1])
      microstep();
    if(dir)
      stepper.setSpeed(-motorSpeed * micRes[1]);
    else
      stepper.setSpeed(motorSpeed * micRes[1]);
  }
  else
    stop();
}

void microstep() {
  switch(micRes[1]) {
  case 1:
    pinMode(M0, OUTPUT);
    digitalWrite(M0, LOW);
    digitalWrite(M1, LOW);
    break;
  case 2:
    pinMode(M0, OUTPUT);
    digitalWrite(M0, HIGH);
    digitalWrite(M1, LOW);
    break;
  case 4:
    pinMode(M0, INPUT);
    digitalWrite(M1, LOW);
    break;
  case 8:
    pinMode(M0, OUTPUT);
    digitalWrite(M0, LOW);
    digitalWrite(M1, HIGH);
    break;
  case 16:
    pinMode(M0, OUTPUT);
    digitalWrite(M0, HIGH);
    digitalWrite(M1, HIGH);
    break;
  case 32:
    pinMode(M0, INPUT);
    digitalWrite(M1, HIGH);
    break;
  }
  micRes[0] = micRes[1];
}

void stop() {
  digitalWrite(13, LOW);
  stepper.setSpeed(0);
  start = 0;
  time = 0;
  elapsed = 0;
  while(Serial.available() > 0)
    Serial.read();
}

void report() {
  int i, voltage = 0;
  for(i = 0; i < 10; i++)
    voltage += analogRead(BATTERY);
  Serial.print(dir);
  Serial.print(' ');
  Serial.print(accelType);
  Serial.print(' ');
  Serial.print(time);
  Serial.print(' ');
  Serial.print(time - elapsed, 0);
  Serial.print(' ');
  Serial.println(voltage * vRatio / i, 0);
}
