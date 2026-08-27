using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KinematicModel : MonoBehaviour
{
    [Header("Robot Links")]
    public ArticulationBody[] joints;

    [Header("End Effector")]
    public Transform endEffector;

    private void Start()
    {
        Debug.Log("=== KINEMATIC MODEL ===");

        for (int i = 0; i < joints.Length; i++)
        {
            ArticulationBody joint = joints[i];

            float angle = joint.jointPosition[0];

            Debug.Log(
                $"J{i + 1}: {joint.name}\n" +
                $"  Local Position: {joint.transform.localPosition}\n" +
                $"  Local Rotation: {joint.transform.localRotation.eulerAngles}\n" +
                $"  Anchor Position: {joint.anchorPosition}\n" +
                $"  Anchor Rotation: {joint.anchorRotation.eulerAngles}\n" +
                $"  Parent Anchor Position: {joint.parentAnchorPosition}\n" +
                $"  Parent Anchor Rotation: {joint.parentAnchorRotation.eulerAngles}\n" +
                $"  Joint Type: {joint.jointType}\n" +
                $"  Angle: {angle * Mathf.Rad2Deg:F2}\n"
            );
        }

        Debug.Log(
            $"EE: {endEffector.name}\n" +
            $"  Local Position: {endEffector.localPosition}\n" +
            $"  World Position: {endEffector.position}"
        );
    }
}
