#include <Arduino.h>
#include "encoder.h"
#include "drive.h"
#include "PID.h"
#include <iostream>

float pathDistance = 0;
// position metrics: x,y (m) heading (degrees)
float x = 0;
float y = 0;
float heading = 0;
// values for curvature PID
float old_heading = 0;
float w = 0;
float curr_k = 0;
float V = 0; // sum of the target wheel velocities
float R = 1; // ratio of L/R wheel velocities

// // spline values
// float x_i = 0.0;
// float y_i = 0.0;
// float x_f = 1.0;
// float y_f = 1.0;
// std::vector<double> X = {x_i, x_f};
// std::vector<double> Y = {y_i, y_f};
// double spline_a = 0;
// double spline_b = 0;
// double spline_c = 0;
// double spline_d = 0;

// instantaneous velocity of each wheel in radians per second
float velFL = 0;
float velBL = 0;
float velFR = 0;
float velBR = 0;

// filtered velocity of each wheel in radians per second
float filtVelFL = 0;
float filtVelBL = 0;
float filtVelFR = 0;
float filtVelBR = 0;

// scaling factor for each new reading
// if alpha = 0, each new reading is not even considered
// if alpha = 1, each new reading is the only thing considered
// lower values of alpha smooth the filtered velocity more, but delay the signal
float alpha = 0.05;

// sum errors for integral term
float sumErrorFL = 0;
float sumErrorBL = 0;
float sumErrorFR = 0;
float sumErrorBR = 0;
float sumErrorK = 0;

// desired velocity setpoints in rad/s
float desiredVelFL = 0;
float desiredVelBL = 0;
float desiredVelFR = 0;
float desiredVelBR = 0;

// voltage to send to the motors
float voltageFL = 0;
float voltageBL = 0;
float voltageFR = 0;
float voltageBR = 0;

// error reading
float errorFL = 0;
float errorBL = 0;
float errorFR = 0;
float errorBR = 0;
float errorK = 0;

// PID Constants
float kp = 5;
float ki = 20;
float kd = 0;

float kpK = 0.05;
float kiK = 0;
float kdK = 0;

float lastRadFL = 0;
float lastRadBL = 0;
float lastRadFR = 0;
float lastRadBR = 0;

float dPhiFL = 0;
float dPhiBL = 0;
float dPhiFR = 0;
float dPhiBR = 0;

// allows the intergral control to max contribution at the max drive voltage
// prevents integral windum
float maxSumError = (DRIVE_VOLTAGE / ki) / 2;
// updates the filtered velocity values
// dt is the time in seconds since the last update
void updateVelocity(float dt)
{
    // store current positions to reference
    lastRadFL = encFLRad;
    lastRadBL = encBLRad;
    lastRadFR = encFRRad;
    lastRadBR = encBRRad;
    // get new positions
    readEncoders();
    // store the change in angle
    dPhiFL = encFLRad - lastRadFL;
    dPhiBL = encBLRad - lastRadBL;
    dPhiFR = encFRRad - lastRadFR;
    dPhiBR = encBRRad - lastRadBR;
    // get (change in angle)/time
    velFL = dPhiFL / dt;
    velBL = dPhiBL / dt;
    velFR = dPhiFR / dt;
    velBR = dPhiBR / dt;
    // use first order alpha based filter to get filtered velocities
    filtVelFL = alpha * velFL + (1 - alpha) * filtVelFR;
    filtVelBL = alpha * velBL + (1 - alpha) * filtVelBL;
    filtVelFR = alpha * velFR + (1 - alpha) * filtVelFR;
    filtVelBR = alpha * velBR + (1 - alpha) * filtVelBR;
}

// Standard PID controller
float runPID(float error, float last_error, float kp, float ki, float kd, float &sumError, float maxSumError, float loopTime)
{
    sumError += error * loopTime;
    // avoid integral windum
    sumError = constrain(sumError, -maxSumError, maxSumError);
    // standard PID configuration
    float P = kp * error;
    float I = ki * sumError;
    float D = kd * (error - last_error) / loopTime;
    return P + I + D;
}