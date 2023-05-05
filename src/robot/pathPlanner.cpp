#include <Arduino.h>
#include "encoder.h"
#include "drive.h"
#include "wireless.h"
#include "PID.h"
// #include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// wheel radius in meters
#define r 0.06
// distance from back wheel to center in meters
#define b 0.2

unsigned long prevLoopTimeMicros = 0; // in microseconds
// how long to wait before updating PID parameters
unsigned long loopDelayMicros = 5000; // in microseconds

double ACCEL_VEL_TRANSITION = (double)loopDelayMicros * 1e-6;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; // trig functions require radians, BNO055 outputs degrees

float desired_v = 0.2;
float desired_k = 1 / 2;

void getPosition();
void setDesiredVel(float vel, float k);
void setWheelVel();
// void updateRobotPose(float dPhiL, float dPhiR);
// void updateOdometry();
// void printOdometry();
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup()
{
    Serial.begin(115200);
    encoderSetup();
    driveSetup();
    wirelessSetup();
}
void loop()
{
    if (micros() < 100000)
    { // run for 10 seconds
        if (micros() - prevLoopTimeMicros > loopDelayMicros)
        {
            prevLoopTimeMicros = micros();

            updateVelocity(loopDelayMicros * 1e-6); // update current wheel velocities

            getPosition(); // get current x,y,heading based on IMU data

            setDesiredVel(desired_v, desired_k); // set new desired wheel velocities

            setWheelVel(); // send new desired wheel velocities
        }
    }
}

// get current poisition from IMU data
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
    w = (heading - old_heading) / (loopDelayMicros * 1e-6);
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

// run PID controller for desired velocities
void setWheelVel()
{
    // calculate error for each motor
    float newErrorFL = desiredVelFL - filtVelFL;
    float newErrorBL = desiredVelBL - filtVelBL;
    float newErrorFR = desiredVelFR - filtVelFR;
    float newErrorBR = desiredVelBR - filtVelBR;

    // get control signal by running PID on all four motors
    voltageFL = runPID(newErrorFL, errorFL, kp, ki, kd, sumErrorFL, maxSumError, loopDelayMicros * 1e-6);
    voltageBL = runPID(newErrorBL, errorBL, kp, ki, kd, sumErrorBL, maxSumError, loopDelayMicros * 1e-6);
    voltageFR = runPID(newErrorFR, errorFR, kp, ki, kd, sumErrorFR, maxSumError, loopDelayMicros * 1e-6);
    voltageBR = runPID(newErrorBR, errorBR, kp, ki, kd, sumErrorBR, maxSumError, loopDelayMicros * 1e-6);

    // only drive the back motors
    driveVolts(0, voltageBL, 0, voltageBR);
}