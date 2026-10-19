/* Sweep
  by BARRAGAN <http://barraganstudio.com>
  This example code is in the public domain.

  modified 28 May 2015
  by Michael C. Miller
  modified 8 Nov 2013
  by Scott Fitzgerald

  http://arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#define MotorINT1 20
#define MotorINT2 21
#define MotorPWM 22

Servo s1;  // create servo object to control a servo
Servo s2;
Servo s3;
Servo s4;

// twelve servo objects can be created on most boards
unsigned long currentTime = 0;
unsigned long lastTime = 0;

int currentS1 = 77;
int currentS2 = 85;
int currentS3 = 0;
int currentS4 = 0;
void setup() {
  s1.attach(8);  // attaches the servo on GIO2 to the servo object NORMALLY 8
  s2.attach(9);
  s3.attach(10);  // NORMALLY 10
  s4.attach(11);
  int pos;

  for (pos = 0; pos <= 180; pos += 1) {  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    s1.write(pos);  // tell servo to go to position in variable 'pos'
    delay(30);      // waits 15ms for the servo to reach the position
    s2.write(pos);
    delay(30);
    s3.write(pos);
    delay(15);
    s4.write(pos);
    delay(15);
    Serial.print("POS: ");
    Serial.println(pos);
  }
  for (pos = 180; pos >= 0; pos -= 1) {  // goes from 180 degrees to 0 degrees
    if (pos >= currentS1) {
      s1.write(pos);  // tell servo to go to position in variable 'pos'
    }
    delay(30);  // waits 15ms for the servo to reach the position
    if (pos >= currentS2) {
      s2.write(pos);
    }
    delay(30);
    if (pos >= currentS3) {
      s3.write(pos);
    }
    delay(15);
    if (pos >= currentS4) {
      s4.write(pos);
    }
    delay(15);
    Serial.print("POS: ");
    Serial.println(pos);
  }
}

void loop() {
  currentTime = millis();

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    if (data[0] == 'M') {
      motorControl(data);
    } else if (data[0] == 'A') {
      absMotorControl(data);
    }
  }
}

void motorControl(String data) {

  //  M,0,0,5,-5,10,-10


  data.trim();
  data = data.substring(2);

  int values[6];
  int start = 0;

  for (int i = 0; i < 6; i++) {
    int comma = data.indexOf(',', start);

    if (comma == -1) {
      values[i] = data.substring(start).toInt();
      break;
    }

    values[i] = data.substring(start, comma).toInt();
    start = comma + 1;
  }

  int motorSpeed = values[0];
  int motorDirection = values[1];

  // Offsets
  int offsetS1 = values[2];
  int offsetS2 = values[3];
  int offsetS3 = values[4];
  int offsetS4 = values[5];

  // Save starting positions
  int startS1 = currentS1;
  int startS2 = currentS2;
  int startS3 = currentS3;
  int startS4 = currentS4;

  // Calculate new targets
  currentS1 = constrain(currentS1 + offsetS1, 0, 180);
  currentS2 = constrain(currentS2 + offsetS2, 0, 180);
  currentS3 = constrain(currentS3 + offsetS3, 0, 180);
  currentS4 = constrain(currentS4 + offsetS4, 0, 180);


  // Increment based on the difference from current to target 
  while (startS1 != currentS1 || startS2 != currentS2 || startS3 != currentS3 || startS4 != currentS4) {

    if (startS1 < currentS1) {
      startS1++;
    } else if (startS1 > currentS1) {
      startS1--;
    }

    if (startS2 < currentS2) {
      startS2++;
    } else if (startS2 > currentS2) {
      startS2--;
    }

    if (startS3 < currentS3) {
      startS3++;
    } else if (startS3 > currentS3) {
      startS3--;
    }

    if (startS4 < currentS4) {
      startS4++;
    } else if (startS4 > currentS4) {
      startS4--;
    }


    s1.write(startS1);
    s2.write(startS2);
    s3.write(startS3);
    s4.write(startS4);
  }


  Serial.print("S1: ");
  Serial.println(currentS1);

  Serial.print("S2: ");
  Serial.println(currentS2);

  Serial.print("S3: ");
  Serial.println(currentS3);

  Serial.print("S4: ");
  Serial.println(currentS4);
}
void absMotorControl(String data) {

  //Data should come in like A,255,1,90,90,270,180\n
  data.trim();               // Remove '\n' and whitespace
  data = data.substring(2);  // Removes A,

  int values[6];
  int start = 0;

  for (int i = 0; i < 6; i++) {
    int dataValue = data.indexOf(',', start);

    if (dataValue == -1) {
      values[i] = data.substring(start).toInt();
      break;
    }

    values[i] = data.substring(start, dataValue).toInt();
    start = dataValue + 1;
  }
  int motorSpeed = constrain(values[0], 0,255);      //0 -255
  int motorDirection = values[1];  //1 is left 0 is right 2 is Force STOP
  int S1 = constrain(values[2], 0,180);              
  int S2 = constrain(values[3], 0,180);              
  int S3 = constrain(values[4], 0,180);              
  int S4 = constrain(values[5], 0,180);             

  if (motorDirection == 1) {
    digitalWrite(MotorINT1, HIGH);
    digitalWrite(MotorINT2, LOW);
  } else if (motorDirection == 0) {
    digitalWrite(MotorINT1, LOW);
    digitalWrite(MotorINT2, HIGH);
  }

  if (motorDirection == 2) {
    digitalWrite(MotorINT1, HIGH);
    digitalWrite(MotorINT2, HIGH);
    analogWrite(MotorPWM, 0);
  } else {
    analogWrite(MotorPWM, motorSpeed);
  }

  //Servos Write
  s1.write(S1);
  s2.write(S2);
  s3.write(S3);
  s4.write(S4);
}
