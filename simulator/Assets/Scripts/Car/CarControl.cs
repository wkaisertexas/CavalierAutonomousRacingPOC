using UnityEngine;

using UnityEngine.Splines;
using Unity.Mathematics;
using System.IO;

public class CarControl : MonoBehaviour
{
    [Header("Car Settings")]
    public float motorTorque = 2000;
    public float brakeTorque = 2000;
    public float maxSpeed = 20;
    public float steeringRange = 30;
    public float steeringRangeAtMaxSpeed = 10;
    public float centreOfGravityOffset = -1f;
    public bool userControlled = true;

    [Header("Spline Settings")]
    // the spline for the road path
    public SplineContainer splineContainer;
    public GameObject closestPoint;

    [Header("PID Control")]
    public float Kp = 1;
    public float Ki = 1;
    public float Kd = 1;

    WheelControl[] wheels;
    Rigidbody rigidBody;

    StreamWriter writer;

    // Start is called before the first frame update
    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();

        // Adjust center of mass vertically, to help prevent the car from rolling
        rigidBody.centerOfMass += Vector3.up * centreOfGravityOffset;

        // Find all child GameObjects that have the WheelControl script attached
        wheels = GetComponentsInChildren<WheelControl>();
    
        // Initialize the stream write
        string projectDir = Path.GetDirectoryName(Application.dataPath);
        string filePath = Path.Combine(projectDir, "experiment_data.csv");
        writer = new StreamWriter(filePath);
        writer.WriteLine("Time, HInput, VInput, Distance, Crash");
    }

    void GetPIDControl(out float vInput, out float hInput, out float dist)
    {
        // Getting the spline
        Spline path = splineContainer[0]; // gets the first spline

        // Getting the car position
        Vector3 carPosition = transform.position;

        // Getting the closest point on the spline
        SplineUtility.GetNearestPoint<Spline>(path, new float3(carPosition.x, carPosition.y, carPosition.z), out float3 nearestPoint, out float t, 64, 16);

        // Okay, so t is a decent approximation of the percentage of the spline that the car is at. However, it's not perfect.
        const float epsilon = 0.0001f;
        const int maxIterations = 500;

        // Getting the closest point on the spline
        float minDistance = Vector3.Distance(SplineUtility.EvaluatePosition(path, t), carPosition);
        float t_new = t;
        for (int i = 0; i < maxIterations; i++)
        {
            float t_test = t + epsilon * (i - maxIterations / 2);
            Vector3 point = SplineUtility.EvaluatePosition(path, t_test);
            float dist2 = Vector3.Distance(point, carPosition);
            if (dist2 < minDistance)
            {
                minDistance = dist2;
                t_new = t_test;
            }
        }

        // updating nearest point
        t = t_new;
        nearestPoint = SplineUtility.EvaluatePosition(path, t);

        // Getting the tangent of the spline at `t`
        Vector3 tangent = SplineUtility.EvaluateTangent(path, t);
        nearestPoint = SplineUtility.EvaluatePosition(path, t);

        // calculate the tangent angle in the xz plane
        float tangentAngle = Mathf.Atan2(tangent.z, tangent.x) * Mathf.Rad2Deg;

        // Getting the forward direction of the car
        Vector3 eulerAngles = transform.eulerAngles;

        // Distance
        float distance = Vector3.Distance(nearestPoint, carPosition);

        // Visualizing the closest point
        closestPoint.transform.position = new Vector3(nearestPoint.x, nearestPoint.y, nearestPoint.z);

        // Setting up PID control

        float angleToSteer = Mathf.Atan2(nearestPoint.z, nearestPoint.x);

        float angleDifference = Mathf.Sin((eulerAngles.y - (angleToSteer + tangentAngle) / 2) * Mathf.Deg2Rad);
        float sqrtDistance = Mathf.Sqrt(distance);

        hInput = angleDifference; // this should be determined by the PID controller
        vInput = 0.1f; // this should not change
        dist = distance;
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetKeyDown(KeyCode.Space))
        {
            userControlled = !userControlled;
        }

        if(Input.GetKeyDown(KeyCode.Escape))
        {
            writer.Close();
            Application.Quit();
        }

        // Getting user input
        float vInput;
        float hInput;
        float dist = 0;

        if (userControlled)
        {
            vInput = Input.GetAxis("Vertical");
            hInput = Input.GetAxis("Horizontal");
        }
        else
        {
            // Use the spline
            GetPIDControl(out vInput, out hInput, out dist);
        }
        // Calculate current speed in relation to the forward direction of the car
        // (this returns a negative number when traveling backwards)
        float forwardSpeed = Vector3.Dot(transform.forward, rigidBody.velocity);


        // Calculate how close the car is to top speed
        // as a number from zero to one
        float speedFactor = Mathf.InverseLerp(0, maxSpeed, forwardSpeed);

        // Use that to calculate how much torque is available 
        // (zero torque at top speed)
        float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);

        // …and to calculate how much to steer 
        // (the car steers more gently at top speed)
        float currentSteerRange = Mathf.Lerp(steeringRange, steeringRangeAtMaxSpeed, speedFactor);

        // Check whether the user input is in the same direction 
        // as the car's velocity
        bool isAccelerating = Mathf.Sign(vInput) == Mathf.Sign(forwardSpeed);

        foreach (var wheel in wheels)
        {
            // Apply steering to Wheel colliders that have "Steerable" enabled
            if (wheel.steerable)
            {
                wheel.WheelCollider.steerAngle = hInput * currentSteerRange;
            }

            if (isAccelerating)
            {
                // Apply torque to Wheel colliders that have "Motorized" enabled
                if (wheel.motorized)
                {
                    wheel.WheelCollider.motorTorque = vInput * currentMotorTorque;
                }
                wheel.WheelCollider.brakeTorque = 0;
            }
            else
            {
                // If the user is trying to go in the opposite direction
                // apply brakes to all wheels
                wheel.WheelCollider.brakeTorque = Mathf.Abs(vInput) * brakeTorque;
                wheel.WheelCollider.motorTorque = 0;
            }
        }

        writer.WriteLine(Time.time + ", " + hInput + ", " + vInput + ", " + dist);
    }
}
