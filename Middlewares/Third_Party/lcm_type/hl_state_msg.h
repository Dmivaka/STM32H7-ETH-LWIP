typedef struct act_meas
{
    float position;
    float velocity;
    float torque;
} act_meas;

typedef struct euler_orient
{
    float x;
    float y;
    float z;
} euler_orient;

typedef struct imu_meas
{
    euler_orient euler;
} imu_meas;

typedef struct measurment
{
    act_meas act[12];
    imu_meas imu;
} measurment;
