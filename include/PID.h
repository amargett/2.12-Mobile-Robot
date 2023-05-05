//instantaneous velocity of each wheel in radians per second
extern float velFL;
extern float velBL;
extern float velFR;
extern float velBR;

extern float x = 0;
extern float y = 0;
extern float heading = 0;

extern float old_heading = 0;
extern float w = 0;
extern float curr_k = 0;
extern float V = 0; // sum of the target wheel velocities
extern float R = 1; // ratio of L/R wheel velocities


//filtered velocity of each wheel in radians per second
extern float filtVelFL;
extern float filtVelBL;
extern float filtVelFR;
extern float filtVelBR;

//scaling factor for each new reading
//if alpha = 0, each new reading is not even considered
//if alpha = 1, each new reading is the only thing considered
//lower values of alpha smooth the filtered velocity more, but delay the signal
extern float alpha;

//sum errors for integral term
extern float sumErrorFL;
extern float sumErrorBL;
extern float sumErrorFR;
extern float sumErrorBR;
extern float sumErrorK = 0;

//desired velocity setpoints in rad/s
extern float desiredVelFL;
extern float desiredVelBL;
extern float desiredVelFR;
extern float desiredVelBR;

//voltage to send to the motors
extern float voltageFL;
extern float voltageBL;
extern float voltageFR;
extern float voltageBR;

//error reading
extern float errorFL;
extern float errorBL;
extern float errorFR;
extern float errorBR;
extern float errorK = 0;
//PID Constants
extern float kp;
extern float ki;
extern float kd;

extern float kpK = 0.05;
extern float kiK = 0;
extern float kdK = 0;

extern float lastRadFL;
extern float lastRadBL;
extern float lastRadFR;
extern float lastRadBR;

extern float dPhiFL;
extern float dPhiBL;
extern float dPhiFR;
extern float dPhiBR;


//function prototypes
void updateVelocity(float dt);
float runPID(float error,float last_error, float kp, float ki, float kd, float &sumError, float maxSumError, float loopTime);
