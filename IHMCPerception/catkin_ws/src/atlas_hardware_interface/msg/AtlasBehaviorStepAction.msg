int32 TOE_OFF_DISABLE=0
int32 TOE_OFF_ENABLE=1
int32 TOE_OFF_FORCE_ENABLE=2

float64 step_duration # duration of the single support phase (STEP_SUBSTATE_STEPPING) 
float64 sway_duration #duration of the double support phase (STEP_SUBSTATE_SWAYING)  
float64 swing_height # step apex above the lift height while swinging the leg 
float64 lift_height #height to lift vertically before moving the foot in x,y 
int32 toe_off #specify whether toe-off is allowed or not during swaying 
float64 knee_nominal #Nominal knee angle during the step. Used to change the desired amount of knee flex while walking. Useful to tweak when running into ankle limits on sloped terrain. Ignored if less than 0.3 