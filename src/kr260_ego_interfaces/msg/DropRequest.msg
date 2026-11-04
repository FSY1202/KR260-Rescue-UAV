# At most one OPEN per (run_id, request_id). CANCEL never opens the actuator.
std_msgs/Header header
string run_id
uint32 request_id
bool cancel
uint32 pulse_us
float32 duration_s
float32 valid_for_s
