std_msgs/Header header

geometry_msgs/Point pos
geometry_msgs/Quaternion q
geometry_msgs/Vector3 vel_lin
geometry_msgs/Vector3 vel_ang
geometry_msgs/Vector3 acc_lin

# State machine of the drone (same values as sm_states_t in state.h)
uint8 INIT=0
uint8 CHECKING=1
uint8 ARMING=2
uint8 TAKING_OFF=3
uint8 HOVERING=4
uint8 EXTERNAL_CONTROL=5
uint8 LANDING=6
uint8 DISARMING=7
uint8 ERROR=8
uint8 sm_state
