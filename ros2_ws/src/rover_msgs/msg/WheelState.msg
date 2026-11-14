# Wheel and link state reported by the firmware at 50 Hz
float32 left_pos      # rad, accumulated
float32 right_pos     # rad, accumulated
float32 left_vel      # rad/s
float32 right_vel     # rad/s
float32 heading       # rad, complementary-filtered yaw
bool link_alive
bool motors_cut_off
uint32 uptime_ms
