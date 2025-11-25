using UnityEngine;
using System.Collections;

public class ForwardKinematics : MonoBehaviour
{
    // Reference to your InverseKinFollower script
    public InverseKinFollower inverseKinScript;

    // Links length (must match your InverseKinFollower)
    private float L1 = 0.13465f;
    private float L2 = 0.11f;
    private float L3 = 0.096f;
    private float L4 = 0.07505f;
    private float L5 = 0.0634f;
    private float L6 = 0.0452f;

    // Current end effector position and rotation
    private Vector3 currentPosition;
    private Quaternion currentRotation;

    // Debug settings
    public bool showDebugInfo = true;
    public float updateFrequency = 0.1f;

    // Coordinate system correction
    public bool applyCoordinateCorrection = true;

    void Start()
    {
        if (inverseKinScript == null)
        {
            inverseKinScript = FindObjectOfType<InverseKinFollower>();
        }

        if (inverseKinScript == null)
        {
            Debug.LogError("ForwardKinematics: No InverseKinFollower script found!");
        }

        StartCoroutine(PrintFKValuesContinuously());
    }

    void Update()
    {
        if (inverseKinScript == null) return;
        CalculateForwardKinematics();
    }

    public void CalculateForwardKinematics()
    {
        if (inverseKinScript == null) return;

        // Get current joint angles from the inverse kinematics script
        float th1 = inverseKinScript.thetha1;
        float th2 = inverseKinScript.thetha2;
        float th3 = inverseKinScript.thetha3;
        float th4 = inverseKinScript.thetha4;
        float th5 = inverseKinScript.thetha5;
        float th6 = inverseKinScript.thetha6;

        // Convert to radians for calculations
        float th1_rad = th1 * Mathf.Deg2Rad;
        float th2_rad = th2 * Mathf.Deg2Rad;
        float th3_rad = th3 * Mathf.Deg2Rad;
        float th4_rad = th4 * Mathf.Deg2Rad;
        float th5_rad = th5 * Mathf.Deg2Rad;
        float th6_rad = th6 * Mathf.Deg2Rad;

        // DH parameters (in radians)
        float alpha1 = 90f * Mathf.Deg2Rad;
        float alpha2 = 0f * Mathf.Deg2Rad;
        float alpha3 = 0f * Mathf.Deg2Rad;
        float alpha4 = 90f * Mathf.Deg2Rad;
        float alpha5 = 270f * Mathf.Deg2Rad;
        float alpha6 = 0f * Mathf.Deg2Rad;

        float r1 = 0f;
        float r2 = L2;
        float r3 = L3;
        float r4 = 0f;
        float r5 = 0f;
        float r6 = 0f;

        float d1 = L1;
        float d2 = 0f;
        float d3 = 0f;
        float d4 = L5;
        float d5 = L4;
        float d6 = L6;

        // Calculate individual transformation matrices
        Matrix4x4 T1 = CreateDHMatrix(th1_rad, alpha1, r1, d1);
        Matrix4x4 T2 = CreateDHMatrix(th2_rad, alpha2, r2, d2);
        Matrix4x4 T3 = CreateDHMatrix(th3_rad, alpha3, r3, d3);
        Matrix4x4 T4 = CreateDHMatrix(th4_rad, alpha4, r4, d4);
        Matrix4x4 T5 = CreateDHMatrix(th5_rad, alpha5, r5, d5);
        Matrix4x4 T6 = CreateDHMatrix(th6_rad, alpha6, r6, d6);

        // Multiply all transformation matrices
        Matrix4x4 TT = T1 * T2 * T3 * T4 * T5 * T6;

        // Extract position
        currentPosition = new Vector3(TT[0, 3], TT[1, 3], TT[2, 3]);

        // Apply coordinate system correction to match Unity's coordinate system
        if (applyCoordinateCorrection)
        {
            currentPosition = ConvertToUnityCoordinates(currentPosition);
        }

        // Extract rotation
        currentRotation = ExtractRotationFromMatrix(TT);
    }

    private Matrix4x4 CreateDHMatrix(float theta, float alpha, float r, float d)
    {
        Matrix4x4 matrix = Matrix4x4.identity;

        float cosTheta = Mathf.Cos(theta);
        float sinTheta = Mathf.Sin(theta);
        float cosAlpha = Mathf.Cos(alpha);
        float sinAlpha = Mathf.Sin(alpha);

        // Standard DH transformation matrix
        matrix[0, 0] = cosTheta;
        matrix[0, 1] = -sinTheta * cosAlpha;
        matrix[0, 2] = sinTheta * sinAlpha;
        matrix[0, 3] = r * cosTheta;

        matrix[1, 0] = sinTheta;
        matrix[1, 1] = cosTheta * cosAlpha;
        matrix[1, 2] = -cosTheta * sinAlpha;
        matrix[1, 3] = r * sinTheta;

        matrix[2, 0] = 0;
        matrix[2, 1] = sinAlpha;
        matrix[2, 2] = cosAlpha;
        matrix[2, 3] = d;

        return matrix;
    }

    private Vector3 ConvertToUnityCoordinates(Vector3 originalPosition)
    {
        // Adjust based on your coordinate system needs
        // Common conversions:

        // Option 1: If your robot uses different axes
        // return new Vector3(originalPosition.y, originalPosition.z, originalPosition.x);

        // Option 2: If you need to flip axes
        // return new Vector3(-originalPosition.x, originalPosition.z, originalPosition.y);

        // Option 3: Based on your IK script coordinate handling
        // Your IK script uses: ox = -z, oy = x, oz = y + 0.016f
        return new Vector3(originalPosition.y, originalPosition.z, -originalPosition.x);
    }

    private Quaternion ExtractRotationFromMatrix(Matrix4x4 matrix)
    {
        Vector3 forward = new Vector3(matrix[0, 2], matrix[1, 2], matrix[2, 2]);
        Vector3 upwards = new Vector3(matrix[0, 1], matrix[1, 1], matrix[2, 1]);

        // Handle zero vectors
        if (forward.magnitude < 0.001f || upwards.magnitude < 0.001f)
        {
            return Quaternion.identity;
        }

        return Quaternion.LookRotation(forward, upwards);
    }

    private IEnumerator PrintFKValuesContinuously()
    {
        while (true)
        {
            if (showDebugInfo && inverseKinScript != null)
            {
                PrintCurrentValues();
            }
            yield return new WaitForSeconds(updateFrequency);
        }
    }

    public void PrintCurrentValues()
    {
        Vector3 eulerAngles = currentRotation.eulerAngles;

        Debug.Log($"=== Forward Kinematics ===");
        Debug.Log($"Joint Angles: θ1={inverseKinScript.thetha1:F1}°, θ2={inverseKinScript.thetha2:F1}°, θ3={inverseKinScript.thetha3:F1}°");
        Debug.Log($"                θ4={inverseKinScript.thetha4:F1}°, θ5={inverseKinScript.thetha5:F1}°, θ6={inverseKinScript.thetha6:F1}°");
        Debug.Log($"End Effector Position: X={currentPosition.x:F4}m, Y={currentPosition.y:F4}m, Z={currentPosition.z:F4}m");
        Debug.Log($"End Effector Rotation: X={eulerAngles.x:F1}°, Y={eulerAngles.y:F1}°, Z={eulerAngles.z:F1}°");

        // Compare with IK target
        Vector3 ikTarget = new Vector3(inverseKinScript.ox, inverseKinScript.oy, inverseKinScript.oz);
        Debug.Log($"IK Target Position: X={ikTarget.x:F4}m, Y={ikTarget.y:F4}m, Z={ikTarget.z:F4}m");
        Debug.Log($"Position Difference: {Vector3.Distance(currentPosition, ikTarget):F4}m");
    }

    // Method to test with known values
    [ContextMenu("Test FK with Zero Angles")]
    public void TestWithZeroAngles()
    {
        Debug.Log("=== Testing FK with all angles at 0° ===");

        float th1 = 0f, th2 = 0f, th3 = 0f, th4 = 0f, th5 = 0f, th6 = 0f;

        // Calculate FK manually for testing
        float th1_rad = th1 * Mathf.Deg2Rad;
        float th2_rad = th2 * Mathf.Deg2Rad;
        float th3_rad = th3 * Mathf.Deg2Rad;
        float th4_rad = th4 * Mathf.Deg2Rad;
        float th5_rad = th5 * Mathf.Deg2Rad;
        float th6_rad = th6 * Mathf.Deg2Rad;

        float alpha1 = 90f * Mathf.Deg2Rad;
        float alpha2 = 0f * Mathf.Deg2Rad;
        float alpha3 = 0f * Mathf.Deg2Rad;
        float alpha4 = 90f * Mathf.Deg2Rad;
        float alpha5 = 270f * Mathf.Deg2Rad;
        float alpha6 = 0f * Mathf.Deg2Rad;

        float r1 = 0f, r2 = L2, r3 = L3, r4 = 0f, r5 = 0f, r6 = 0f;
        float d1 = L1, d2 = 0f, d3 = 0f, d4 = L5, d5 = L4, d6 = L6;

        Matrix4x4 T1 = CreateDHMatrix(th1_rad, alpha1, r1, d1);
        Matrix4x4 T2 = CreateDHMatrix(th2_rad, alpha2, r2, d2);
        Matrix4x4 T3 = CreateDHMatrix(th3_rad, alpha3, r3, d3);
        Matrix4x4 T4 = CreateDHMatrix(th4_rad, alpha4, r4, d4);
        Matrix4x4 T5 = CreateDHMatrix(th5_rad, alpha5, r5, d5);
        Matrix4x4 T6 = CreateDHMatrix(th6_rad, alpha6, r6, d6);

        Matrix4x4 TT = T1 * T2 * T3 * T4 * T5 * T6;
        Vector3 testPosition = new Vector3(TT[0, 3], TT[1, 3], TT[2, 3]);

        Debug.Log($"Test Position (all 0°): X={testPosition.x:F4}, Y={testPosition.y:F4}, Z={testPosition.z:F4}");
    }

    // Visualize in Scene view
    void OnDrawGizmos()
    {
        /*
        if (!showDebugInfo) return;

        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(currentPosition, 0.01f);

        // Draw coordinate axes
        float axisLength = 0.05f;
        Gizmos.color = Color.red;
        Gizmos.DrawLine(currentPosition, currentPosition + currentRotation * Vector3.right * axisLength);
        Gizmos.color = Color.green;
        Gizmos.DrawLine(currentPosition, currentPosition + currentRotation * Vector3.up * axisLength);
        Gizmos.color = Color.blue;
        Gizmos.DrawLine(currentPosition, currentPosition + currentRotation * Vector3.forward * axisLength);
        */
    }

    // Public getters
    public Vector3 GetEndEffectorPosition() => currentPosition;
    public Quaternion GetEndEffectorRotation() => currentRotation;
}