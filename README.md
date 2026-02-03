# IK Servo Arm Controller

## Overview

This project integrates Unity with Arduino to control a robotic arm using Inverse Kinematics (IK). The Unity scripts handle IK calculations to position the arm's end effector towards a target, map joint rotations to servo angles, and send commands over serial communication to an Arduino board. The Arduino smoothly actuates up to 4 servos (3 for joints and 1 for a trigger mechanism) based on the received data.

This setup is ideal for VR/AR applications, robotic simulations, or physical prototypes where precise and smooth servo control is needed.

## Features

- **Inverse Kinematics (IK)**: Computes joint rotations to reach a target position using gradient descent.
- **Servo Mapping**: Maps Unity joint angles to servo positions (0-180 degrees).
- **Serial Communication**: Unity sends servo commands to Arduino via a serial port (e.g., COM6).
- **Smooth Servo Movement**: Arduino interpolates positions for gradual, jitter-free motion.
- **Trigger Control**: Supports an additional servo controlled by input (e.g., VR trigger).
- **Configurable**: Easily adjust joint limits, servo ranges, ports, and smoothing parameters.

## Requirements

### Hardware
- Arduino board (e.g., Uno or Mega) with at least 4 PWM pins.
- 4 Servo motors (e.g., SG90 or similar).
- USB cable for serial connection between computer and Arduino.
- Optional: VR headset or input device for trigger control.

### Software
- Unity 2020+ (with Input System package for trigger input).
- Arduino IDE 1.8+.
- .NET Framework (for Unity's SerialPort in Windows).

## Setup

1. **Arduino Setup**:
   - Connect servos to Arduino pins (default: 9, 10, 11, 12).
   - Upload the Arduino sketch (`ServoController.ino`) to your board.

2. **Unity Setup**:
   - Create a new Unity project or import into an existing one.
   - Add the `Ikmanager.cs` and `MultiServoController.cs` scripts to your scene.
   - Set up your armature hierarchy:
     - Each joint should be a GameObject with a `Joint` script (assuming a custom `Joint` class for hierarchy and rotation).
   - Assign the root joint, end effector, and target in the `Ikmanager` inspector.
   - For `MultiServoController`, assign joint transforms, axes, angle limits, and the trigger input action.
   - Ensure the serial port (e.g., COM6) matches your Arduino connection.

3. **Serial Configuration**:
   - Update `portName` in `MultiServoController.cs` to match your Arduino's COM port.
   - Baud rate is set to 9600 (adjust if needed in both Unity and Arduino).

## Usage

1. **Run in Unity**:
   - Play the scene. The IK solver will adjust the armature to reach the target.
   - Joint angles are mapped and sent to Arduino in real-time.
   - Use the assigned input (e.g., VR trigger) to control the fourth servo.

2. **Arduino Behavior**:
   - Receives comma-separated servo values (e.g., "90,120,45,0\n").
   - Smoothly moves servos towards targets with configurable step size and interval.

3. **Testing**:
   - Monitor Unity console for distance thresholds.
   - Use Arduino Serial Monitor to debug incoming data.

## Code Structure

### Unity Scripts

#### Ikmanager.cs
This script performs IK using gradient descent to minimize the distance between the end effector and target.

```csharp
using UnityEngine;

public class Ikmanager : MonoBehaviour
{
    // Root of the armature
    public Joint m_root;
    public Joint m_end;
    public GameObject m_target;
    public float m_threshold = 0.05f;
    public float m_rate = 5.0f;

    float CalculateSlope(Joint _joint)
    {
        float deltaTheta = 0.01f;
        float distance1 = GetDistance(m_end.transform.position, m_target.transform.position);
        _joint.Rotate(deltaTheta);
        float distance2 = GetDistance(m_end.transform.position, m_target.transform.position);
        _joint.Rotate(-deltaTheta);
        return (distance2 - distance1) / deltaTheta;
    }

    void Update()
    {
        if (GetDistance(m_end.transform.position, m_target.transform.position) > m_threshold)
        {
            Joint current = m_root;
            while (current != null)
            {
                float slope = CalculateSlope(current);
                current.Rotate(-slope * m_rate);
                current = current.GetChild();
            }
        }
    }

    float GetDistance(Vector3 _point1, Vector3 _point2)
    {
        return Vector3.Distance(_point1, _point2);
    }
}
```

#### MultiServoController.cs
This script maps joint rotations to servo commands and sends them to Arduino, including trigger input.

```csharp
using UnityEngine;
using System.IO.Ports;
using UnityEngine.InputSystem;

public class MultiServoController : MonoBehaviour
{
    [System.Serializable]
    public class ServoConfig
    {
        public Transform joint;
        public Axis rotationAxis;
        public float jointMinAngle;
        public float jointMaxAngle;
        public int servoMin = 0;
        public int servoMax = 180;
        [HideInInspector] public int servoOut;
    }

    public enum Axis { X, Y, Z }

    [Header("Joint Controlled Servos (3)")]
    public ServoConfig[] servos = new ServoConfig[3];

    [Header("Trigger Controlled Servo")]
    public InputActionProperty triggerInput;
    public int triggerServoMin = 0;
    public int triggerServoMax = 180;
    private int triggerServoOut;

    [Header("Serial Settings")]
    public string portName = "COM6";
    public int baudRate = 9600;
    SerialPort serial;

    void Start()
    {
        serial = new SerialPort(portName, baudRate);
        serial.Open();
        serial.ReadTimeout = 50;
        triggerInput.action.Enable();
    }

    void Update()
    {
        // Joint based servos
        for (int i = 0; i < servos.Length; i++)
        {
            float jointAngle = GetJointRotation(servos[i]);
            jointAngle = NormalizeAngle(jointAngle);
            servos[i].servoOut = Mathf.RoundToInt(
                Mathf.Lerp(
                    servos[i].servoMin,
                    servos[i].servoMax,
                    Mathf.InverseLerp(
                        servos[i].jointMinAngle,
                        servos[i].jointMaxAngle,
                        jointAngle
                    )
                )
            );
        }

        // Trigger based servo
        float triggerValue = triggerInput.action.ReadValue<float>(); // 0 → 1
        triggerServoOut = Mathf.RoundToInt(
            Mathf.Lerp(triggerServoMin, triggerServoMax, triggerValue)
        );

        SendToArduino();
    }

    float GetJointRotation(ServoConfig servo)
    {
        Vector3 rot = servo.joint.localEulerAngles;
        switch (servo.rotationAxis)
        {
            case Axis.X: return rot.x;
            case Axis.Y: return rot.y;
            case Axis.Z: return rot.z;
        }
        return 0;
    }

    float NormalizeAngle(float angle)
    {
        if (angle > 180) angle -= 360;
        return angle;
    }

    void SendToArduino()
    {
        if (!serial.IsOpen) return;
        string data =
            servos[0].servoOut + "," +
            servos[1].servoOut + "," +
            servos[2].servoOut + "," +
            triggerServoOut + "\n";
        serial.Write(data);
    }

    void OnApplicationQuit()
    {
        if (serial != null && serial.IsOpen)
            serial.Close();
    }
}
```

### Arduino Sketch (ServoController.ino)
This handles serial input and smooth servo actuation.

```cpp
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
```

## Notes
- Assumes a custom `Joint` class for armature hierarchy (e.g., with `Rotate()` and `GetChild()` methods). Implement if missing.
- Handle serial port permissions on your OS (e.g., add user to dialout group on Linux).
- For production, add error handling for serial disconnections.

## Contributing
Feel free to fork and submit pull requests. Issues and feature requests are welcome!

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
