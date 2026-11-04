# Per-frame camera metadata from rpicam-vid's meta.json (libcamera request metadata).
std_msgs/Header header

uint32 exposure_us
float32 analogue_gain
float32 digital_gain
float32 lux
uint32 colour_temperature        # kelvin
float32[2] colour_gains          # red, blue
float32[9] colour_correction_matrix
uint32[4] sensor_black_levels
uint32 focus_fom
uint32 ae_state
string scaler_crop               # e.g. "(0, 0)/2592x1944"
uint64 frame_wall_clock_us       # recorder wall clock at capture, 0 if absent
uint64 boot_ns                   # sensor timestamp on the boot clock
uint32 segment
uint32 frame_index               # index within the segment
