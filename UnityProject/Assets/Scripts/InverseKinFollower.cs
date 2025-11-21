using UnityEngine;
using static UnityEngine.GraphicsBuffer;
using UnityEngine.UI;
using UnityEditor;
using UnityEngine.UIElements;
using UnityEditorInternal;
using System.Globalization;

public class InverseKinFollower : MonoBehaviour
{
    private float[] q = new float[6]; // start joints orientation

    private float[] qm = new float[6]; // trajectory

    public float q01, q02, q03, q04, q05, q06;

    public float thetha1, thetha2, thetha3, thetha4, thetha5, thetha6;

    public GameObject robot;

    // point to calculate ik
    public float ox;
    public float oy;
    public float oz;

    public bool RobotOperate;

    private float Tm= 1; // trajectory steps
    public GameObject IK_Changer; // object to follow

    // links length
    private float L1 = 0.13465f;
    private float L2 = 0.11f; 
    private float L3 = 0.096f;
    private float L4 = 0.07505f;
    private float L5 = 0.0634f;
    private float L6 = 0.0452f;

    // to calculate inverse kin manually
    // public InputField NewX, NewY, NewZ;

    void Start()
    {
        q02 = 90;
        q04 = 90;
        q05 = 90;
        q06 = 90;
    }
    void Update()
    {
        ox = -IK_Changer.transform.localPosition.z;
        oy = IK_Changer.transform.localPosition.x;
        oz = IK_Changer.transform.localPosition.y + 0.016f;
        RobotOperate = true;
        CalculateInverseKinematics();
    }

    public void ChangeStateRobotOperate(bool state)
    {
        if (!state)
        {
            RobotOperate = true;
        }
        else
        {
            RobotOperate = false;
        }
    }

    public void CalculateInverseKinematics()
    {
        // Transformation matrix of the ik point
        /*
        Matrix4x4 T = new Matrix4x4(new Vector4(1, 0, 0, ox),
                                    new Vector4(0, 0, 1, oy),
                                    new Vector4(0, -1, 0, oz),
                                    new Vector4(0, 0, 0, 1));
        */

        Matrix4x4 T = new Matrix4x4(new Vector4(-1, 0,  0, ox),
                                    new Vector4( 0, 1,  0, oy),
                                    new Vector4( 0, 0, -1, oz),
                                    new Vector4( 0, 0,  0, 1));

        // Rotation matrix of the ik point
        Matrix4x4 R = new Matrix4x4(new Vector4(T[0, 0], T[1, 0], T[2, 0], 0),
                                    new Vector4(T[0, 1], T[1, 1], T[2, 1], 0),
                                    new Vector4(T[0, 2], T[1, 2], T[2, 2], 0),
                                    new Vector4(0, 0, 0, 1));

        Vector3 o = new Vector3(ox, oy, oz); // center point calculation
        float xc = o.x - L6 * R[2, 0];
        float yc = o.y - L6 * R[2, 1];
        float zc = o.z - L6 * R[2, 2];

        // calculate thetha1
        float r = Mathf.Sqrt(xc * xc + yc * yc);
        float fi = Mathf.Asin(L5/r) * Mathf.Rad2Deg;
        float fi1 = Mathf.Atan2(xc, yc) * Mathf.Rad2Deg;
        thetha1 = fi1 - fi;

        // calculate thetha2 and thetha3
        float s = zc - L1;
        float f = Mathf.Sqrt(r * r - L5 * L5);
        float c = f - L4;
        float e = Mathf.Sqrt(s * s + c * c);
        float alpha = Mathf.Atan2(s, c) * Mathf.Rad2Deg;
        float D1 = (L2 * L2 + e * e - L3 * L3) / (2 * L2 * e);
        float gama = Mathf.Acos(D1) * Mathf.Rad2Deg;
        float D2 = (L2 * L2 + L3 * L3 - e * e) / (2 * L2 * L3);
        float gama1 = Mathf.Acos(D2) * Mathf.Rad2Deg;
        thetha2 = alpha + gama; 
        thetha3 = -(180 - gama1);

        // calculate thetha4
        float x = f * Mathf.Tan(alpha * Mathf.Deg2Rad);
        float w = x - s;
        float gama2 = 180 - gama - gama1;
        float beta = 180 - gama2;
        float beta2 = Mathf.Atan2(w, L4) * Mathf.Rad2Deg;
        thetha4 = gama2 - beta2;
        thetha4 = thetha4 + 90;

        // calculate thetha5, thetha6
        thetha5 = 90;
        thetha6 = 90;

        // ModifyRobot(thetha1, thetha2, thetha3, thetha4, thetha5, thetha6);

        q[0] = thetha1;
        q[1] = thetha2;
        q[2] = thetha3;
        q[3] = thetha4;
        q[4] = thetha5;
        q[5] = thetha6;

        qm[0] = (q[0] - q01) / Tm;
        qm[1] = (q[1] - q02) / Tm;
        qm[2] = (q[2] - q03) / Tm;
        qm[3] = (q[3] - q04) / Tm;
        qm[4] = (q[4] - q05) / Tm;
        qm[5] = (q[5] - q06) / Tm;

        StartCoroutine(LoopWithDelay(q01, q02, q03, q04, q05, q06));
    }

    public void BackToInitPos() // function to move robot tot initial position
    {
        thetha1 = 0;
        thetha2 = 90;
        thetha3 = 0;
        thetha4 = 0;
        thetha5 = -90;
        thetha6 = 0;

        q[0] = thetha1;
        q[1] = thetha2;
        q[2] = thetha3;
        q[3] = thetha4;
        q[4] = thetha5;
        q[5] = thetha6;

        qm[0] = (q[0] - q01) / Tm;
        qm[1] = (q[1] - q02) / Tm;
        qm[2] = (q[2] - q03) / Tm;
        qm[3] = (q[3] - q04) / Tm;
        qm[4] = (q[4] - q05) / Tm;
        qm[5] = (q[5] - q06) / Tm;

        RobotOperate = true;
        StartCoroutine(LoopWithDelay(q01, q02, q03, q04, q05, q06));
    }

    // Cubes IK
    public void Point1_Position() // calculate ik for the given point
    {
        ox = -0.05615f;
        oy = 0.24523f;
        oz = 0.1394f;

        RobotOperate = true;
        CalculateInverseKinematics();
    }

    public void Point2_Position() // calculate ik for the given point
    {
        ox = -0.093f;
        oy = 0.184f;
        oz = 0.166f;

        RobotOperate = true;
        CalculateInverseKinematics();
    }

    public void RobotStop() // stop the robot
    {
        RobotOperate = false;
    }

    System.Collections.IEnumerator LoopWithDelay(float q01, float q02, float q03, float q04, float q05, float q06)
    {
        for (int i = 1; i <= Tm; i++)
        {
            if (RobotOperate)
            {
                q01 += qm[0]; q02 += qm[1]; q03 += qm[2]; q04 += qm[3]; q05 += qm[4]; q06 += qm[5];
            }

            thetha1 = q01;
            thetha2 = q02;
            thetha3 = q03;
            thetha4 = q04;
            thetha5 = q05;
            thetha6 = q06;
            ModifyRobot(q01, q02, q03, q04, q05, q06);

            yield return new WaitForSeconds(0.05f); // how fast robot moves

            if (!RobotOperate)
            {
                yield break;
            }
        }
    }

    private void ModifyRobot(float a1, float a2, float a3, float a4, float a5, float a6)
    {
        RobotTest robotIns = robot.GetComponent<RobotTest>();

        robotIns.J1Angle = a1;
        robotIns.J2Angle = a2;
        robotIns.J3Angle = a3;
        robotIns.J4Angle = a4;
        robotIns.J5Angle = a5;
        robotIns.J6Angle = a6;

        q01 = thetha1;
        q02 = thetha2;
        q03 = thetha3;
        q04 = thetha4;
        q05 = thetha5;
        q06 = thetha6;
    }
}