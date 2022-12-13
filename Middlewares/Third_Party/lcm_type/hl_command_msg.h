typedef struct act_command
{
  float position;
  float velocity;
  float torque;
  float kp;
  float kd;
} act_command;

typedef struct hl_command_msg
{
  act_command act[12];
} hl_command_msg;
