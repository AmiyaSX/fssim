uint32 _can_id = 24 # message id

float64 dv_speed_target           
float64 dv_steering_angle_target  
uint8  dv_brake_hydralic_target  
float64 dv_motor_moment_target    
uint8  dv_mission_status         

# dv_mission_status enum values
uint8  dv_mission_status_Acceleration = 1
uint8  dv_mission_status_Skidpad = 2
uint8  dv_mission_status_Trackdrive = 3
uint8  dv_mission_status_Braketest = 4
uint8  dv_mission_status_Inspection = 5
uint8  dv_mission_status_Autocross = 6
uint8  dv_mission_status_Mission_Finished = 7
uint8  dv_mission_status_Emergency = 0