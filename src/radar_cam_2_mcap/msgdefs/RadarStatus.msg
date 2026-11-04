# LD2451 data frame status.
std_msgs/Header header

uint8 reported_count             # target count as sent by the radar (payload byte 0)
uint8 target_count               # targets actually present in the decoded frame
bool approach_alarm              # a target is approaching (payload byte 1)
uint64 boot_ns                   # frame receive time on the boot clock
