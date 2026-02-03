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
