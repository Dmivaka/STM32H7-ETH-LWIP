typedef struct servo_cmd_msg
{
    float      timestamp;
    float      position[12];
    float      velocity[12];
    float      torque[12];
    float      kp[12];
    float      kd[12];
} servo_cmd_msg;