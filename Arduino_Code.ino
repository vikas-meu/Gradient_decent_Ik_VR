#include <Servo.h>

// Define servo objects for the 4 servos
Servo servo0;  // Joint servo 1
Servo servo1;  // Joint servo 2
Servo servo2;  // Joint servo 3
Servo triggerServo;  // Trigger servo

// Define pin numbers (adjust these based on your hardware setup)
const int servo0Pin = 9;
const int servo1Pin = 10;
const int servo2Pin = 11;
const int triggerServoPin = 12;

// Smoothing parameters
const int stepSize = 1;  // Degrees per step
const unsigned long moveInterval = 15;  // Milliseconds between steps

// Target and current positions
int target0 = 90;
int target1 = 90;
int target2 = 90;
int targetTrigger = 90;

int current0 = 90;
int current1 = 90;
int current2 = 90;
int currentTrigger = 90;

// Timestamps for last moves
unsigned long lastMove0 = 0;
unsigned long lastMove1 = 0;
unsigned long lastMove2 = 0;
unsigned long lastMoveTrigger = 0;

void setup() {
  // Attach servos to their pins
  servo0.attach(servo0Pin);
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  triggerServo.attach(triggerServoPin);

  // Initialize serial communication at the same baud rate as Unity
  Serial.begin(9600);

  // Set initial positions
  servo0.write(current0);
  servo1.write(current1);
  servo2.write(current2);
  triggerServo.write(currentTrigger);
}

void loop() {
  // Check if data is available on serial
  if (Serial.available() > 0) {
    // Read the incoming string until newline
    String data = Serial.readStringUntil('\n');

    // Parse the comma-separated values
    int comma1 = data.indexOf(',');
    int comma2 = data.indexOf(',', comma1 + 1);
    int comma3 = data.indexOf(',', comma2 + 1);

    if (comma1 != -1 && comma2 != -1 && comma3 != -1) {
      // Extract each servo value as string
      String val0Str = data.substring(0, comma1);
      String val1Str = data.substring(comma1 + 1, comma2);
      String val2Str = data.substring(comma2 + 1, comma3);
      String triggerStr = data.substring(comma3 + 1);

      // Convert to integers
      int val0 = val0Str.toInt();
      int val1 = val1Str.toInt();
      int val2 = val2Str.toInt();
      int triggerVal = triggerStr.toInt();

      // Constrain values to 0-180 to prevent servo damage
      target0 = constrain(val0, 0, 180);
      target1 = constrain(val1, 0, 180);
      target2 = constrain(val2, 0, 180);
      targetTrigger = constrain(triggerVal, 0, 180);
    }
  }

  // Smoothly move each servo towards its target
  unsigned long currentTime = millis();

  // Servo 0
  if (currentTime - lastMove0 >= moveInterval) {
    if (current0 < target0) {
      current0 += stepSize;
      if (current0 > target0) current0 = target0;
      servo0.write(current0);
      lastMove0 = currentTime;
    } else if (current0 > target0) {
      current0 -= stepSize;
      if (current0 < target0) current0 = target0;
      servo0.write(current0);
      lastMove0 = currentTime;
    }
  }

  // Servo 1
  if (currentTime - lastMove1 >= moveInterval) {
    if (current1 < target1) {
      current1 += stepSize;
      if (current1 > target1) current1 = target1;
      servo1.write(current1);
      lastMove1 = currentTime;
    } else if (current1 > target1) {
      current1 -= stepSize;
      if (current1 < target1) current1 = target1;
      servo1.write(current1);
      lastMove1 = currentTime;
    }
  }

  // Servo 2
  if (currentTime - lastMove2 >= moveInterval) {
    if (current2 < target2) {
      current2 += stepSize;
      if (current2 > target2) current2 = target2;
      servo2.write(current2);
      lastMove2 = currentTime;
    } else if (current2 > target2) {
      current2 -= stepSize;
      if (current2 < target2) current2 = target2;
      servo2.write(current2);
      lastMove2 = currentTime;
    }
  }

  // Trigger Servo
  if (currentTime - lastMoveTrigger >= moveInterval) {
    if (currentTrigger < targetTrigger) {
      currentTrigger += stepSize;
      if (currentTrigger > targetTrigger) currentTrigger = targetTrigger;
      triggerServo.write(currentTrigger);
      lastMoveTrigger = currentTime;
    } else if (currentTrigger > targetTrigger) {
      currentTrigger -= stepSize;
      if (currentTrigger < targetTrigger) currentTrigger = targetTrigger;
      triggerServo.write(currentTrigger);
      lastMoveTrigger = currentTime;
    }
  }
}
