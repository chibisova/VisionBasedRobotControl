using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Robot Joints")]
    public ArticulationBody[] joints;

    [Header("Target Angles (degrees)")]
    public float[] targetAngles = new float[6];

    void Start()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            var drive = joints[i].xDrive;

            drive.stiffness = 1000f;
            drive.damping = 100f;
            drive.forceLimit = 1000f;

            joints[i].xDrive = drive;
        }
    }

    void Update()
    {
        if (joints == null || targetAngles == null)
            return;

        for (int i = 0; i < joints.Length; i++)
        {
            if (i >= targetAngles.Length || joints[i] == null)
                continue;

            var drive = joints[i].xDrive;
            drive.target = targetAngles[i];
            joints[i].xDrive = drive;
        }
    }
}