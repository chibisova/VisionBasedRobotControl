using UnityEngine;
using System.Collections;

public class InverseKinematics : MonoBehaviour
{
    public ArticulationBody[] joints;
    public Transform endEffector;
    public Transform target;

    public int maxIterations = 100;
    public float learningRate = 0.5f;
    public float positionThreshold = 0.005f;
    public float numericalStep = 0.001f;

    private Vector3[] linkOffsets;
    private Quaternion[] linkRotations;
    private Vector3 zeroBasePosition;
    private Quaternion zeroBaseRotation;
    private Vector3 eeOffset;

    private float[] angles;

    private Vector3[] ikJointPositions;
    private Vector3 ikEndEffectorPosition;

    void Start()
    {
        CaptureFKModel();

        angles = new float[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            angles[i] = joints[i].jointPosition[0];
        }

        Debug.Log("=== MATHEMATICAL IK ===");
        Debug.Log($"Target: {target.position}");

        SolveIK();
        ApplyIKPose(angles);
        StartCoroutine(CalculateUnityPositionErrorAfterMove());
    }

    IEnumerator CalculateUnityPositionErrorAfterMove()
    {
        for (int i = 0; i < 30; i++)
        {
            yield return new WaitForFixedUpdate();
        }

        for (int i = 0; i < joints.Length; i++)
        {
            float actualAngle = joints[i].jointPosition[0] * Mathf.Rad2Deg;
            Debug.Log($"Unity J{i + 1} Angle: {actualAngle:F2}°");
        }

        Debug.Log($"IK EE: {ikEndEffectorPosition}");
        Debug.Log($"Unity EE: {endEffector.position}");

        float modelError = Vector3.Distance(endEffector.position, ikEndEffectorPosition);
        Debug.Log($"Unity vs IK Error: {modelError:F4} m");

        float error = Vector3.Distance(endEffector.position, target.position);
        Debug.Log($"Unity Position Error: {error:F4} m");

        float[] actualAngles = new float[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            actualAngles[i] = joints[i].jointPosition[0];
        }

        Vector3 actualAngleFK = CalculateFKPosition(actualAngles);

        Debug.Log($"Actual Angle FK: {actualAngleFK}");
        Debug.Log($"Actual Angle FK Error: {Vector3.Distance(actualAngleFK, target.position):F4} m");
    }

    void CaptureFKModel()
    {
        int n = joints.Length;

        zeroBasePosition = joints[0].transform.position;
        zeroBaseRotation = joints[0].transform.rotation;

        linkOffsets = new Vector3[n - 1];
        linkRotations = new Quaternion[n - 1];

        for (int i = 0; i < n - 1; i++)
        {
            Transform parent = joints[i].transform;
            Transform child = joints[i + 1].transform;

            linkOffsets[i] = parent.InverseTransformPoint(child.position);
            linkRotations[i] = Quaternion.Inverse(parent.rotation) * child.rotation;
        }

        Transform j6 = joints[n - 1].transform;

        eeOffset = j6.InverseTransformPoint(endEffector.position);
    }

    Vector3 CalculateFKPosition(float[] testAngles)
    {
        int n = testAngles.Length;

        Vector3 position = zeroBasePosition;
        Quaternion rotation = zeroBaseRotation;

        for (int i = 0; i < n; i++)
        {
            Quaternion jointRotation = Quaternion.AngleAxis(testAngles[i] * Mathf.Rad2Deg, Vector3.down);
            rotation = rotation * jointRotation;

            if (i < n - 1)
            {
                position += rotation * linkOffsets[i];
                rotation = rotation * linkRotations[i];
            }
        }

        position += rotation * eeOffset;

        return position;
    }

    void SolveIK()
    {
        int n = joints.Length;
        float damping = 0.05f;

        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            Vector3 currentPosition = CalculateFKPosition(angles);
            Vector3 error = target.position - currentPosition;

            if (error.magnitude < positionThreshold)
            {
                CalculateIKPose(angles);
                ApplyIKToRobot();

                Debug.Log($"IK converged at iteration {iteration}");
                Debug.Log($"Final IK Error: {error.magnitude:F4} m");
                Debug.Log($"Solved Angles: {angles[0] * Mathf.Rad2Deg:F2}, {angles[1] * Mathf.Rad2Deg:F2}, {angles[2] * Mathf.Rad2Deg:F2}, {angles[3] * Mathf.Rad2Deg:F2}, {angles[4] * Mathf.Rad2Deg:F2}, {angles[5] * Mathf.Rad2Deg:F2}");
                return;
            }

            Vector3[] jacobian = new Vector3[n];

            for (int joint = 0; joint < n; joint++)
            {
                float originalAngle = angles[joint];

                angles[joint] = originalAngle + numericalStep;

                Vector3 perturbedPosition = CalculateFKPosition(angles);

                angles[joint] = originalAngle;

                jacobian[joint] = (perturbedPosition - currentPosition) / numericalStep;
            }

            float[,] jjt = new float[3, 3];

            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 3; col++)
                {
                    float value = 0f;

                    for (int joint = 0; joint < n; joint++)
                    {
                        value += jacobian[joint][row] * jacobian[joint][col];
                    }

                    jjt[row, col] = value;

                    if (row == col)
                    {
                        jjt[row, col] += damping * damping;
                    }
                }
            }

            float[,] inverse = Invert3x3(jjt);

            float[] errorVector = { error.x, error.y, error.z };

            float[] intermediate = new float[3];

            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 3; col++)
                {
                    intermediate[row] += inverse[row, col] * errorVector[col];
                }
            }

            float[] deltaAngles = new float[n];

            for (int joint = 0; joint < n; joint++)
            {
                float value = 0f;

                for (int row = 0; row < 3; row++)
                {
                    value += jacobian[joint][row] * intermediate[row];
                }

                deltaAngles[joint] = value * learningRate;
            }

            for (int joint = 0; joint < n; joint++)
            {
                angles[joint] += deltaAngles[joint];
            }
        }

        Vector3 finalPosition = CalculateFKPosition(angles);
        float finalError = Vector3.Distance(finalPosition, target.position);

        CalculateIKPose(angles);
        ApplyIKToRobot();

        Debug.Log("IK did not fully converge.");
        Debug.Log($"Final IK Error: {finalError:F4} m");
        Debug.Log($"Solved Angles: {angles[0] * Mathf.Rad2Deg:F2}, {angles[1] * Mathf.Rad2Deg:F2}, {angles[2] * Mathf.Rad2Deg:F2}, {angles[3] * Mathf.Rad2Deg:F2}, {angles[4] * Mathf.Rad2Deg:F2}, {angles[5] * Mathf.Rad2Deg:F2}");
    }

    float[,] Invert3x3(float[,] matrix)
    {
        float a = matrix[0, 0];
        float b = matrix[0, 1];
        float c = matrix[0, 2];
        float d = matrix[1, 0];
        float e = matrix[1, 1];
        float f = matrix[1, 2];
        float g = matrix[2, 0];
        float h = matrix[2, 1];
        float i = matrix[2, 2];

        float determinant = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);

        if (Mathf.Abs(determinant) < 0.000001f)
        {
            determinant = 0.000001f;
        }

        float inverseDeterminant = 1f / determinant;

        float[,] inverse = new float[3, 3];

        inverse[0, 0] = (e * i - f * h) * inverseDeterminant;
        inverse[0, 1] = (c * h - b * i) * inverseDeterminant;
        inverse[0, 2] = (b * f - c * e) * inverseDeterminant;

        inverse[1, 0] = (f * g - d * i) * inverseDeterminant;
        inverse[1, 1] = (a * i - c * g) * inverseDeterminant;
        inverse[1, 2] = (c * d - a * f) * inverseDeterminant;

        inverse[2, 0] = (d * h - e * g) * inverseDeterminant;
        inverse[2, 1] = (b * g - a * h) * inverseDeterminant;
        inverse[2, 2] = (a * e - b * d) * inverseDeterminant;

        return inverse;
    }

    Vector3 CalculateIKPose(float[] testAngles)
    {
        int n = testAngles.Length;

        Vector3 position = zeroBasePosition;
        Quaternion rotation = zeroBaseRotation;

        ikJointPositions = new Vector3[n];
        ikJointPositions[0] = position;

        for (int i = 0; i < n; i++)
        {
            Quaternion jointRotation = Quaternion.AngleAxis(testAngles[i] * Mathf.Rad2Deg, Vector3.down);
            rotation = rotation * jointRotation;

            if (i < n - 1)
            {
                position += rotation * linkOffsets[i];
                rotation = rotation * linkRotations[i];
                ikJointPositions[i + 1] = position;
            }
        }

        position += rotation * eeOffset;
        ikEndEffectorPosition = position;

        return position;
    }

    void ApplyIKToRobot()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            ArticulationBody joint = joints[i];

            ArticulationDrive drive = joint.xDrive;

            drive.target = angles[i] * Mathf.Rad2Deg;

            joint.xDrive = drive;
        }
    }

    void ApplyIKPose(float[] solvedAngles)
    {
        for (int i = 0; i < joints.Length; i++)
        {
            ArticulationDrive drive = joints[i].xDrive;
            drive.target = solvedAngles[i] * Mathf.Rad2Deg;
            joints[i].xDrive = drive;
        }
    }

    void CalculateUnityPositionError()
    {
        float error = Vector3.Distance(endEffector.position, target.position);

        Debug.Log($"Unity Position Error: {error:F4} m");
    }

    private void OnDrawGizmos()
    {
        if (ikJointPositions == null || ikJointPositions.Length == 0)
            return;

        // IK joints
        Gizmos.color = Color.magenta;

        for (int i = 0; i < ikJointPositions.Length; i++)
        {
            Gizmos.DrawSphere(ikJointPositions[i], 0.022f);

            if (i > 0)
            {
                Gizmos.DrawLine(ikJointPositions[i - 1], ikJointPositions[i]);
            }
        }

        // IK end effector
        Gizmos.color = Color.yellow;

        Gizmos.DrawSphere(ikEndEffectorPosition, 0.025f);

        // J6 -> IK EE
        Gizmos.DrawLine(ikJointPositions[ikJointPositions.Length - 1], ikEndEffectorPosition);

        // Target
        if (target != null)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(target.position, 0.03f);

            // IK EE -> target
            Gizmos.color = Color.white;
            Gizmos.DrawLine(ikEndEffectorPosition, target.position);
        }
    }
}