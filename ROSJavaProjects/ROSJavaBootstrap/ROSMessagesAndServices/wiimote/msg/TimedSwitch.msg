# TimedSwitch allows sender to:
#    o turn a switch on,
#    o turn a switch off, and
#    o repeat an on/off pattern forever or for a
#          given number of times.
# Fields (refer to definitions of constants in the definition body):
#     o switch_mode:
#         ON: turn on  (num_cycles and pulse_pattern fields are ignored)
#        OFF: turn off (num_cycles and pulse_pattern fields are ignored)
#  NO_CHANGE: leave LED in its current state
#     REPEAT: repeat an on/off pattern for as long
#             as is indicated in the num_cycles field. The
#             pattern is defined in the pulse_pattern field.
#
#     o num_cycles:
#          n>=0: run the pattern that is defined in pulse_pattern
#                n times.
#          n==FOREVER: run the pattern that is defined in pulse_pattern
#                       until a new TimedSwitch message is sent.              
#
#     o pulse_pattern:
#          A series of time durations in fractions of a second. The
#          first number is the duration for having the switch on.
#          The second number is the duration for which the switch
#          is off. The third is an 'on' period again, etc.
#          A pattern is terminated with the end of the array.
#           
#          Example: [1,1] specifies an on-off sequence of 1 second.               

int8 ON        =  1
int8 OFF       =  0
int8 NO_CHANGE = -2
int8 REPEAT    = -1
int8 FOREVER   = -1

int8 switch_mode
int32 num_cycles
float32[] pulse_pattern
