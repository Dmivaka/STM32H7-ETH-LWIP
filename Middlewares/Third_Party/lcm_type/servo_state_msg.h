typedef struct servo_state_msg
{
    float      timestamp;
    float      position[12];
    float      velocity[12];
    float      torque[12];
}servo_state_msg;