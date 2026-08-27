using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class ForwardKinematics : MonoBehaviour
{
    public ArticulationBody[] joints;
    public Transform endEffector;

    // Zero-pose local transform from each joint to its child.
    private Vector3[] linkOffsets;
    private Quaternion[] linkRotations;

    // EE transform relative to J6 at zero pose.
    private Vector3 eeOffset;
    private Quaternion eeRotation;

    private Vector3[] zeroJointPositions;

    private Quaternion zeroBaseRotation;

    private Vector3[] fkJointPositions;
    private Vector3 fkEndEffectorPosition;

    private float fkPositionError;

    void Start()
    {
        CaptureZeroPose();
        TestJ2Transform();
        TestZeroPoseChain();
    }

    void FixedUpdate()
    {
        CalculateFK();
    }

    void TestJ2Transform()
    {
        ArticulationBody j2 = joints[1];

        float j2Angle =
            j2.jointPosition[0] * Mathf.Rad2Deg;

        Vector3 j2AxisWorld =
            j2.transform.TransformDirection(
                Vector3.down
            ).normalized;

        Debug.Log("=== J2 AXIS TEST ===");
        Debug.Log($"J2 angle: {j2Angle:F2}°");
        Debug.Log($"J2 local axis: {Vector3.down}");
        Debug.Log($"J2 world axis: {j2AxisWorld}");
    }

    void TestZeroPoseChain()
    {
        Debug.Log("=== ZERO POSE CHAIN TEST ===");

        Vector3 start = zeroJointPositions[0];

        Debug.Log($"J1: {start}");

        for (int i = 1; i < zeroJointPositions.Length; i++)
        {
            Debug.Log(
                $"J{i + 1}: {zeroJointPositions[i]}"
            );
        }

        Debug.Log($"EE: {endEffector.position}");

        Debug.Log(
            $"Direct J1 → EE: " +
            $"{endEffector.position - zeroJointPositions[0]}"
        );
    }

    void CaptureZeroPose()
    {
        int n = joints.Length;

        zeroBaseRotation = joints[0].transform.rotation;

        zeroJointPositions = new Vector3[n];

        fkJointPositions = new Vector3[n];

        linkOffsets = new Vector3[n - 1];
        linkRotations = new Quaternion[n - 1];

        for (int i = 0; i < n; i++)
        {
            zeroJointPositions[i] =
                joints[i].transform.position;
        }

        Debug.Log("=== FK ZERO-POSE MODEL ===");

        /*
         * Capture the relationship between consecutive
         * ArticulationBody transforms at zero pose.
         */
        for (int i = 0; i < n - 1; i++)
        {
            Transform parent = joints[i].transform;
            Transform child = joints[i + 1].transform;

            linkOffsets[i] =
                parent.InverseTransformPoint(child.position);

            linkRotations[i] =
                Quaternion.Inverse(parent.rotation) *
                child.rotation;

            Debug.Log(
                $"J{i + 1} -> J{i + 2} | " +
                $"Offset: {linkOffsets[i]} | " +
                $"Rotation: {linkRotations[i].eulerAngles}"
            );
        }

        // EE relative to J6.
        Transform j6 = joints[n - 1].transform;

        eeOffset =
            j6.InverseTransformPoint(endEffector.position);

        eeRotation =
            Quaternion.Inverse(j6.rotation) *
            endEffector.rotation;

        Debug.Log($"J6 -> EE Offset: {eeOffset}");
        Debug.Log(
            $"J6 -> EE Rotation: {eeRotation.eulerAngles}"
        );
    }

    void CalculateFK()
    {
        int n = joints.Length;

        float[] angles = new float[n];

        fkJointPositions = new Vector3[n];

        for (int i = 0; i < n; i++)
        {
            angles[i] =
                joints[i].jointPosition[0] * Mathf.Rad2Deg;
        }

        /*
         * Start from J1's ZERO-POSE world transform.
         */
        Vector3 position =
            zeroJointPositions[0];

        Quaternion rotation =
            zeroBaseRotation;

        fkJointPositions[0] = position;
        /*
         * Build the chain.
         *
         * Each joint rotates around its LOCAL -Y axis.
         * After applying that joint rotation, the
         * zero-pose transform to the child is applied.
         */
        for (int i = 0; i < n; i++)
        {
            /*
             * Joint axis is LOCAL -Y.
             */
            Quaternion jointRotation =
                Quaternion.AngleAxis(
                    angles[i],
                    Vector3.down
                );

            /*
             * Apply this joint's rotation in its
             * own local frame.
             */
            rotation =
                rotation * jointRotation;

            /*
             * Move from this joint to the next joint.
             *
             * linkOffsets[i] is expressed in this
             * joint's local frame.
             */
            if (i < n - 1)
            {
                position +=
                    rotation * linkOffsets[i];

                /*
                 * Restore the child's zero-pose
                 * orientation relative to this joint.
                 */
                rotation =
                    rotation * linkRotations[i];
                fkJointPositions[i + 1] = position;

            }
        }

        /*
         * J6 → End Effector.
         */
        position +=
            rotation * eeOffset;

        fkEndEffectorPosition = position;

        Debug.Log("=== FORWARD KINEMATICS ===");

        Debug.Log(
            $"Angles: " +
            $"{angles[0]:F2}, " +
            $"{angles[1]:F2}, " +
            $"{angles[2]:F2}, " +
            $"{angles[3]:F2}, " +
            $"{angles[4]:F2}, " +
            $"{angles[5]:F2}"
        );

        Debug.Log($"FK Position: {position}");
        Debug.Log($"Unity Position: {endEffector.position}");

        fkPositionError =
            Vector3.Distance(
                position,
                endEffector.position
            );

        Debug.Log($"Position Error: {fkPositionError:F4} m");
    }

    private void OnDrawGizmos()
    {
        if (joints == null || joints.Length == 0)
            return;

        if (fkJointPositions == null ||
            fkJointPositions.Length != joints.Length)
            return;

        // --------------------------------------------------
        // 1. ACTUAL UNITY JOINTS
        // --------------------------------------------------

        #if UNITY_EDITOR
        Handles.color = Color.cyan;

        for (int i = 0; i < joints.Length; i++)
        {
            Vector3 unityJointPosition =
                joints[i].transform.position;

            Handles.SphereHandleCap(
                0,
                unityJointPosition,
                Quaternion.identity,
                0.018f,
                EventType.Repaint
            );
        }
        #endif


        // --------------------------------------------------
        // 2. ACTUAL UNITY JOINT CONNECTIONS
        // --------------------------------------------------

        #if UNITY_EDITOR
        Handles.color = Color.green;

        for (int i = 0; i < joints.Length - 1; i++)
        {
            Vector3 start =
                joints[i].transform.position;

            Vector3 end =
                joints[i + 1].transform.position;

            Handles.DrawAAPolyLine(
                5f,       // thickness
                start,
                end
            );
        }
        #endif


        // --------------------------------------------------
        // 3. FK JOINT POSITIONS
        // --------------------------------------------------

        #if UNITY_EDITOR
        Handles.color = new Color(0.2f, 0.4f, 1f);

        for (int i = 0; i < fkJointPositions.Length; i++)
        {
            Handles.SphereHandleCap(
                0,
                fkJointPositions[i],
                Quaternion.identity,
                0.01f,
                EventType.Repaint
            );
        }
        #endif

        // --------------------------------------------------
        // 5. ACTUAL UNITY END EFFECTOR
        // --------------------------------------------------

        #if UNITY_EDITOR
        Handles.color = new Color(1f, 0.5f, 0f); 

        Handles.SphereHandleCap(
            0,
            endEffector.position,
            Quaternion.identity,
            0.025f,
            EventType.Repaint
        );
        #endif


        // --------------------------------------------------
        // 4. FK END EFFECTOR
        // --------------------------------------------------

        #if UNITY_EDITOR
        Handles.color = new Color(0.7f, 0.2f, 1f);

        Handles.SphereHandleCap(
            0,
            fkEndEffectorPosition,
            Quaternion.identity,
            0.013f,
            EventType.Repaint
        );
        #endif

        // --------------------------------------------------
        // 6. FK EE → UNITY EE ERROR LINE
        // --------------------------------------------------

        #if UNITY_EDITOR
        Handles.color = Color.white;

        Handles.DrawAAPolyLine(
            3f,
            fkEndEffectorPosition,
            endEffector.position
        );
        #endif


        // --------------------------------------------------
        // 7. FK J6 → FK EE
        // --------------------------------------------------

        #if UNITY_EDITOR
        Handles.color = Color.yellow;

        Handles.DrawAAPolyLine(
            5f,
            fkJointPositions[fkJointPositions.Length - 1],
            fkEndEffectorPosition
        );
        #endif

        // --------------------------------------------------
        // 8. JOINTS LABELS
        // --------------------------------------------------

        #if UNITY_EDITOR
        GUIStyle labelStyle = new GUIStyle(EditorStyles.label);
        labelStyle.fontSize = 8;
        labelStyle.normal.textColor = Color.white;

        for (int i = 0; i < joints.Length; i++)
        {
            Vector3 position = joints[i].transform.position;

            Vector3 labelOffset =
                Camera.current.transform.up * 0.035f +
                Camera.current.transform.right * 0.02f;

            Handles.Label(
                position + labelOffset,
                $"J{i + 1}",
                labelStyle
            );
        }
        #endif
    }

    private void OnGUI()
    {
        // Legend position
        float x = Screen.width - 260f;
        float y = 30f;
        float width = 250f;
        float height = 170f;

        // Background
        GUI.color = new Color(0f, 0f, 0f, 0.5f);
        GUI.Box(
            new Rect(x, y, width, height),
            ""
        );

        GUI.color = Color.white;

        // Title
        GUIStyle titleStyle = new GUIStyle(GUI.skin.label);
        titleStyle.fontSize = 18;
        titleStyle.fontStyle = FontStyle.Bold;

        GUI.Label(
            new Rect(x + 15f, y + 10f, width - 30f, 25f),
            "FK VISUALIZATION",
            titleStyle
        );

        // Legend text
        GUIStyle labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.fontSize = 14;

        float rowY = y + 45f;
        float rowSpacing = 22f;

        // Unity joints
        GUI.color = Color.cyan;
        GUI.Label(
            new Rect(x + 15f, rowY, 20f, 20f),
            "●"
        );

        GUI.color = Color.white;
        GUI.Label(
            new Rect(x + 40f, rowY, 160f, 20f),
            "Unity Joint",
            labelStyle
        );

        // FK joints
        rowY += rowSpacing;

        GUI.color = new Color(0.2f, 0.4f, 1f);
        GUI.Label(
            new Rect(x + 15f, rowY, 20f, 20f),
            "●"
        );

        GUI.color = Color.white;
        GUI.Label(
            new Rect(x + 40f, rowY, 160f, 20f),
            "FK Joint",
            labelStyle
        );

        // FK chain
        rowY += rowSpacing;

        GUI.color = Color.green;
        GUI.Label(
            new Rect(x + 15f, rowY, 20f, 20f),
            "━"
        );

        GUI.color = Color.white;
        GUI.Label(
            new Rect(x + 40f, rowY, 160f, 20f),
            "FK Chain",
            labelStyle
        );

        // FK end effector
        rowY += rowSpacing;

        GUI.color = new Color(0.7f, 0.2f, 1f);
        GUI.Label(
            new Rect(x + 15f, rowY, 20f, 20f),
            "●"
        );

        GUI.color = Color.white;
        GUI.Label(
            new Rect(x + 40f, rowY, 160f, 20f),
            "FK End Effector",
            labelStyle
        );

        // Unity end effector
        rowY += rowSpacing;

        GUI.color = new Color(1f, 0.5f, 0f);
        GUI.Label(
            new Rect(x + 15f, rowY, 20f, 20f),
            "●"
        );

        GUI.color = Color.white;
        GUI.Label(
            new Rect(x + 40f, rowY, 160f, 20f),
            "Unity End Effector",
            labelStyle
        );

        GUI.color = Color.white;

        // --------------------------------------------------
        // POSITION ERROR
        // --------------------------------------------------

        float errorWidth = 250f;
        float errorHeight = 70f;

        float errorX = Screen.width - errorWidth - 10f;
        float errorY = Screen.height - errorHeight - 15f;

        // Background
        GUI.color = new Color(0f, 0f, 0f, 0.5f);

        GUI.Box(
            new Rect(
                errorX,
                errorY,
                errorWidth,
                errorHeight
            ),
            ""
        );

        // Error title
        GUIStyle errorTitleStyle =
            new GUIStyle(GUI.skin.label);

        errorTitleStyle.fontSize = 13;
        errorTitleStyle.fontStyle = FontStyle.Bold;
        errorTitleStyle.alignment = TextAnchor.MiddleCenter;

        GUI.color = Color.white;

        GUI.Label(
            new Rect(
                errorX + 10f,
                errorY + 8f,
                errorWidth - 20f,
                20f
            ),
            "FK POSITION ERROR",
            errorTitleStyle
        );

        // Error value
        GUIStyle errorValueStyle =
            new GUIStyle(GUI.skin.label);

        errorValueStyle.fontSize = 15;
        errorValueStyle.fontStyle = FontStyle.Bold;
        errorValueStyle.alignment = TextAnchor.MiddleCenter;
        errorValueStyle.normal.textColor = Color.green;

        GUI.Label(
            new Rect(
                errorX + 10f,
                errorY + 30f,
                errorWidth - 20f,
                30f
            ),
            $"{fkPositionError * 1000f:F2} mm",
            errorValueStyle
        );

        GUI.color = Color.white;
    }
}