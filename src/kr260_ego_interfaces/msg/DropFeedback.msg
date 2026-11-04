uint8 READY=0
uint8 RUNNING=1
uint8 COMPLETE=2
uint8 FAILED=3
std_msgs/Header header
string run_id
uint32 request_id
uint8 state
# COMPLETE requires the executor to confirm the command ended AND PWM is off.
bool outputs_disabled
string detail
