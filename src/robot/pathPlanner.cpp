#include <Arduino.h>
#include "encoder.h"
#include "drive.h"
#include "wireless.h"
#include "PID.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <vector>
#include <cmath>
#include "spline.h"
#include <iostream>
using namespace std;

// wheel radius in meters
#define r 0.06
// distance from back wheel to center in meters
#define b 0.2

// holds the odometry data to be sent to the microcontroller
// odometry_message odom_data;

float pathDistance = 0;
// x and y position of the robot in meters
float x = 0;
float y = 0;
float theta = 0;
float dPhiFL = 0;
float dPhiBL = 0;
float dPhiFR = 0;
float dPhiBR = 0;
float old_heading = 0;
float heading = 0;
float w = 0;
float curr_k = 0;
float time_step = 0.01;
float V = 0; // sum of the target wheel velocities
float R = 1; // ratio of L/R wheel velocities

float x_i = 0.0;
float y_i = 0.0;
float x_f = 1.0;
float y_f = 1.0;
std::vector<double> X = {x_i, x_f};
std::vector<double> Y = {y_i, y_f};

double spline_a = 0;
double spline_b = 0;
double spline_c = 0;
double spline_d = 0;

float errorK = 0;
float errorV = 0;
float sumErrorV = 0;
float sumErrorK = 0;

float kpK = 5;
float kiK = 0;
float kdK = 0;

float kpV = 5;
float kiV = 0;
float kdV = 0;

// allows the intergral control to max contribution at the max drive voltage
// prevents integral windum
float maxSumError = (DRIVE_VOLTAGE / ki) / 2;

unsigned long prevLoopTimeMicros = 0; // in microseconds
// how long to wait before updating PID parameters
unsigned long loopDelayMicros = 5000; // in microseconds

unsigned long prevPrintTimeMillis = 0;
unsigned long printDelayMillis = 50;

double ACCEL_VEL_TRANSITION = (double)loopDelayMicros * 1e-6;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void getPosition();
void setDesiredVel(float vel, float k);
// void updateRobotPose(float dPhiL, float dPhiR);
void getSetPointTrajectory();
// void updateOdometry();
// void printOdometry();
void spline(std::vector<double> X, std::vector<double> Y);
double df(double x);
double ddf(double x);
double curvature(double x);

void setup()
{
    Serial.begin(115200);
    encoderSetup();
    driveSetup();
    wirelessSetup();
}

void loop()
{
    if (micros() - prevLoopTimeMicros > loopDelayMicros)
    {
        prevLoopTimeMicros = micros();
        // get new encoder readings and update the velocity
        // also updates dPhi values for the change in angle of each motor
        updateVelocity(loopDelayMicros * 1e-6);

        // dRad is the change in radians since the last reading of the encoders
        // just use the back left and back right encoders to calculate trajectory
        // updateRobotPose(dPhiBL, dPhiBR);

        // sends odometry to the remote
        // updateOdometry();
        // sendOdometry();

        // uncomment the desired method for updating the PI setpoint
        // getSetPointTrajectory();
        // getSetPointDriveTest();
        // getSetPointJoystick();

        spline(X, Y);

        getPosition();

        double k = curvature(x);

        setDesiredVel(0.2, k);

        // calculate error for each motor
        float newErrorFL = desiredVelFL - filtVelFL;
        float newErrorBL = desiredVelBL - filtVelBL;
        float newErrorFR = desiredVelFR - filtVelFR;
        float newErrorBR = desiredVelBR - filtVelBR;

        // get control signal by running PID on all for motors
        voltageFL = runPID(newErrorFL, errorFL, kp, ki, kd, sumErrorFL, maxSumError, loopDelayMicros * 1e-6);
        voltageBL = runPID(newErrorBL, errorBL, kp, ki, kd, sumErrorBL, maxSumError, loopDelayMicros * 1e-6);
        voltageFR = runPID(newErrorFR, errorFR, kp, ki, kd, sumErrorFR, maxSumError, loopDelayMicros * 1e-6);
        voltageBR = runPID(newErrorBR, errorBR, kp, ki, kd, sumErrorBR, maxSumError, loopDelayMicros * 1e-6);

        // only drive the back motors
        driveVolts(0, voltageBL, 0, voltageBR);
    }

    // put print statements here
    if (millis() - prevPrintTimeMillis > printDelayMillis)
    {
        prevPrintTimeMillis = millis();
        // printOdometry();
        // Serial.printf("Left Vel: %.2f Right Vel %.2f\n", filtVelBL, filtVelBR);
        // Serial.printf("dPhiBL: %.4f dPhiBR %.4f\n", dPhiBL, dPhiBR);
    }
}

void getPosition()
{
    sensors_event_t orientationData, linearAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    x = x + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
    y = y + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
    // V = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);
    heading = orientationData.orientation.x;
}

// sets the desired velocity based on desired velocity vel in m/s
// and k curvature in 1/m representing 1/(radius of curvature)
void setDesiredVel(float vel, float k)
{
    // TODO: update pos and heading from IMU data
    V = vel;
    w = (heading - old_heading) / time_step;
    curr_k = w / V;
    float newErrorK = k - curr_k;
    R = runPID(newErrorK, errorK, kpK, kiK, kdK, sumErrorK, maxSumError, loopDelayMicros * 1e-6);

    float left = V / (1 + R);
    float right = V / (1 + 1 / R);

    desiredVelBL = left;
    desiredVelFL = 0;
    desiredVelBR = right;
    desiredVelFR = 0;
    old_heading = heading;
}

// makes robot follow a trajectory
// void getSetPointTrajectory(float vel, float k)
// {
//     // default to not moving
//     // velocity in m/s
//     // k is 1/radius from center of rotation circle
//     // TODO Add trajectory planning by changing the value of vel and k
//     // based on odemetry conditions

//     if (pathDistance <= 1.0)
//     {
//         // STRAIGHT LINE FORWARD
//         vel = 0;
//         k = 0;
//     }
//     else if (pathDistance > 1 && pathDistance < (1 + 0.25 * PI))
//     {
//         // TURN IN SEMICIRCLE
//         vel = 0;
//         k = 0;
//     }
//     else if (pathDistance > (1 + 0.25 * PI) && pathDistance < (2 + 0.25 * PI))
//     {
//         // STRAIGHT LINE BACK
//         vel = 0;
//         k = 0;
//     }
//     else
//     {
//         // STOP
//         vel = 0;
//         k = 0;
//     }
//     setDesiredVel(vel, k);
// }

// updates the robot's path distance variable based on the latest change in angle
// void updateRobotPose(float dPhiL, float dPhiR)
// {
//     // TODO change in angle
//     float dtheta = 0;
//     // TODO update theta value
//     theta += 0;
//     // TODO use the equations from the handout to calculate the change in x and y
//     float dx = 0;
//     float dy = 0;
//     // TODO update x and y positions
//     x += 0;
//     y += 0;
//     // TODO update the pathDistance
//     pathDistance += 0;
//     // Serial.printf("x: %.2f y: %.2f\n", x, y);
// }

// stores all the the latest odometry data into the odometry struct
// void updateOdometry()
// {
//     odom_data.millis = millis();
//     odom_data.pathDistance = pathDistance;
//     odom_data.x = x;
//     odom_data.y = y;
//     odom_data.theta = theta;
//     odom_data.velL = filtVelBL;
//     odom_data.velR = filtVelBR;
// }
// prints current odometry to be read into MATLAB
// void printOdometry()
// {
//     // convert the time to seconds
//     Serial.printf("%.2f\t%.4f\t%.4f\t%.4f\t%.4f\n", odom_data.millis / 1000.0, odom_data.x, odom_data.y, odom_data.theta, odom_data.pathDistance);
// }

void spline(std::vector<double> X, std::vector<double> Y)
{
    // default cubic spline (C^2) with natural boundary conditions (f''=0)
    tk::spline s;
    s.set_boundary(tk::spline::first_deriv, 0.0,
                   tk::spline::first_deriv, 0.0);
    s.set_points(X, Y);

    // f(x) = a_i + b_i*(x-x_i) + c_i*(x-x_i)^2 + d_i*(x-x_i)^3

    // assumes y_i = a_i = 0 and x_i = 0

    spline_a = 0;
    spline_b = s.m_b[0];
    spline_c = s.m_c[0];
    spline_d = s.m_d[0];
}

double df(double x)
{
    return (3 * spline_a * x * x + 2 * spline_b * x + spline_c);
}

double ddf(double x)
{
    return (6 * spline_a * x + 2 * spline_b);
}

double curvature(double x)
{
    double radius = pow(1 + pow(df(x), 2), 3.0 / 2.0) / ddf(x);
    return 1.0 / radius;
}