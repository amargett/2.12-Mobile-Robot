#include <Arduino.h>
#include "encoder.h"
#include "drive.h"
#include "wireless.h"
#include "PID.h"
#include "ESP32Servo.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//wheel radius in meters
#define r 0.06
//distance from back wheel to center in meters
#define b 0.2

//holds the odometry data to be sent to the microcontroller
odometry_message odom_data;

float pathDistance = 0;
//x and y position of the robot in meters
float x = 0;
float y = 0;
float theta = 0;

float arm_target = 90;

int servoPin = 13;

float dPhiFL = 0;
float dPhiBL = 0;
float dPhiFR = 0;
float dPhiBR = 0;

Servo myservo;

float gyro_orientation_rad;

float velocity = 0;
float curvature = 0;
float servo_angle = 90;
//allows the intergral control to max contribution at the max drive voltage
//prevents integral windum
float maxSumError = (DRIVE_VOLTAGE/ki)/2;

unsigned long prevLoopTimeMicros = 0; //in microseconds
//how long to wait before updating PID parameters
unsigned long loopDelayMicros = 5000; //in microseconds

unsigned long prevPrintTimeMillis = 0;
unsigned long printDelayMillis = 50;

void setDesiredVel(float vel, float k);
void updateRobotPose(float dPhiL, float dPhiR);
void getSetPointTrajectory();
void updateOdometry();
void printOdometry();


void setup(){
    Serial.begin(115200);
    encoderSetup();
    driveSetup();
    wirelessSetup();
    pinMode(servoPin, OUTPUT);
    
    myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object

    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

}

void loop(){


     /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    
    /* Display the floating point data */
    Serial.print("Theta: ");
    gyro_orientation_rad = 0.01745*event.orientation.x;
    Serial.print(gyro_orientation_rad);
    Serial.println("\t");
    

    if (Serial.available()> 0 )  {
        // read incoming data
        String data = Serial.readStringUntil('\n');
        // split data into three float values
        int firstCommaIndex = data.indexOf(',');
        int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
        velocity = data.substring(0, firstCommaIndex).toFloat();
        curvature = data.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
        servo_angle = data.substring(secondCommaIndex + 1).toFloat();
        // print values to serial monitor
    }


    myservo.write(servo_angle);
    

    if (micros() - prevLoopTimeMicros > loopDelayMicros){
        prevLoopTimeMicros = micros();
        //get new encoder readings and update the velocity
        //also updates dPhi values for the change in angle of each motor
        updateVelocity(loopDelayMicros*1e-6);

        //dRad is the change in radians since the last reading of the encoders
        //just use the back left and back right encoders to calculate trajectory
        updateRobotPose(dPhiBL, dPhiBR);

        //sends odometry to the remote
        updateOdometry();
        sendOdometry();

        //uncomment the desired method for updating the PI setpoint 
        getSetPointTrajectory();
        //getSetPointDriveTest();
        //getSetPointJoystick();

        //calculate error for each motor
        float newErrorFL = desiredVelFL - filtVelFL;
        float newErrorBL = desiredVelBL - filtVelBL;
        float newErrorFR = desiredVelFR - filtVelFR;
        float newErrorBR = desiredVelBR - filtVelBR;

        //get control signal by running PID on all for motors
        voltageFL = runPID(newErrorFL, errorFL, kp, ki, kd, sumErrorFL, maxSumError, loopDelayMicros*1e-6);      
        voltageBL = runPID(newErrorBL, errorBL, kp, ki, kd, sumErrorBL, maxSumError, loopDelayMicros*1e-6);
        voltageFR = runPID(newErrorFR, errorFR, kp, ki, kd, sumErrorFR, maxSumError, loopDelayMicros*1e-6);            
        voltageBR = runPID(newErrorBR, errorBR, kp, ki, kd, sumErrorBR, maxSumError, loopDelayMicros*1e-6);
        
        //only drive the back motors
        driveVolts(0, voltageBL, 0, voltageBR);
    }

    //put print statements here
    if (millis() - prevPrintTimeMillis > printDelayMillis){
        prevPrintTimeMillis = millis();
        printOdometry();
        //Serial.printf("Left Vel: %.2f Right Vel %.2f\n", filtVelBL, filtVelBR);
        //Serial.printf("dPhiBL: %.4f dPhiBR %.4f\n", dPhiBL, dPhiBR);
    }

}

//sets the desired velocity based on desired velocity vel in m/s
//and k curvature in 1/m representing 1/(radius of curvature)
void setDesiredVel(float vel, float k){

    desiredVelFL = 0;

    desiredVelFR = 0;

    desiredVelBL = vel*(1-b*k)/r;
    desiredVelBR = vel*(1+b*k)/r;

}

//makes robot follow a trajectory
void getSetPointTrajectory(){
    //default to not moving
    //velocity in m/s
    //k is 1/radius from center of rotation circle

    //TODO Add trajectory planning by changing the value of vel and k
    //based on odemetry conditions
    /*
    if (pathDistance <= 1.0){
        //STRAIGHT LINE FORWARD
        vel = 0.2;
        k = 0;
        arm_target = 145;
    } else if (pathDistance > 1 && pathDistance < 1.1){
        //TURN IN SEMICIRCLE
        vel = 0.1;
        k = 0;
        arm_target = 115;
    } else if (pathDistance >= 1.1 && pathDistance < (2 + 0.25*PI)){
        //STRAIGHT LINE BACK
        vel = -0.2;
        k = 0;
    } else {
        //STOP
        vel = 0;
        k = 0;
    }
    */


    setDesiredVel(velocity, curvature);
}

//updates the robot's path distance variable based on the latest change in angle
void updateRobotPose(float dPhiL, float dPhiR){
    //TODO change in angle
    float dtheta = r/(2*b)*(dPhiR-dPhiL);
    //TODO update theta value
    theta += dtheta;
    //TODO use the equations from the handout to calculate the change in x and y
    float dx = r/2*(cos(theta)*dPhiR+cos(theta)*dPhiL);
    float dy = r/2*(sin(theta)*dPhiR+sin(theta)*dPhiL);
    //TODO update x and y positions
    x += dx;
    y += dy;
    //TODO update the pathDistance
    pathDistance += sqrt(dx*dx + dy*dy);
    //Serial.printf("x: %.2f y: %.2f\n", x, y);
}

//stores all the the latest odometry data into the odometry struct
void updateOdometry(){
    odom_data.millis = millis();
    odom_data.pathDistance = pathDistance;
    odom_data.x = x;
    odom_data.y = y;
    odom_data.theta = theta;
    odom_data.velL = filtVelBL;
    odom_data.velR = filtVelBR;
}
