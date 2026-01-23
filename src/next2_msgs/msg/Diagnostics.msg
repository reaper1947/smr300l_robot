uint8 SYSTEM_READY_STATUS_OK = 0
uint8 SYSTEM_READY_NOT_USE = 1
uint8 SYSTEM_READY_STATUS_FAULT = 2
uint8 SYSTEM_READY_WAIT_COMEUP = 3

std_msgs/Header header
uint8 system_ready
string desciption
Devices device_state
Error error_state
