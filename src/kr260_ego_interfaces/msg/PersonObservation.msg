# Source metadata is preserved from YOLO; a repeated HTTP result is not a new frame.
std_msgs/Header header
string detector_session
string camera_session
uint64 result_seq
uint64 frame_seq
uint64 source_monotonic_ns
bool person_detected
float32 confidence
