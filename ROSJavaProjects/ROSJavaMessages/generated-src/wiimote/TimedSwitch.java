package wiimote;

public interface TimedSwitch extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "wiimote/TimedSwitch";
  static final java.lang.String _DEFINITION = "# TimedSwitch allows sender to:\n#    o turn a switch on,\n#    o turn a switch off, and\n#    o repeat an on/off pattern forever or for a\n#          given number of times.\n# Fields (refer to definitions of constants in the definition body):\n#     o switch_mode:\n#         ON: turn on  (num_cycles and pulse_pattern fields are ignored)\n#        OFF: turn off (num_cycles and pulse_pattern fields are ignored)\n#  NO_CHANGE: leave LED in its current state\n#     REPEAT: repeat an on/off pattern for as long\n#             as is indicated in the num_cycles field. The\n#             pattern is defined in the pulse_pattern field.\n#\n#     o num_cycles:\n#          n>=0: run the pattern that is defined in pulse_pattern\n#                n times.\n#          n==FOREVER: run the pattern that is defined in pulse_pattern\n#                       until a new TimedSwitch message is sent.              \n#\n#     o pulse_pattern:\n#          A series of time durations in fractions of a second. The\n#          first number is the duration for having the switch on.\n#          The second number is the duration for which the switch\n#          is off. The third is an \'on\' period again, etc.\n#          A pattern is terminated with the end of the array.\n#           \n#          Example: [1,1] specifies an on-off sequence of 1 second.               \n\nint8 ON        =  1\nint8 OFF       =  0\nint8 NO_CHANGE = -2\nint8 REPEAT    = -1\nint8 FOREVER   = -1\n\nint8 switch_mode\nint32 num_cycles\nfloat32[] pulse_pattern\n";
  static final byte ON = 1;
  static final byte OFF = 0;
  static final byte NO_CHANGE = -2;
  static final byte REPEAT = -1;
  static final byte FOREVER = -1;
  byte getSwitchMode();
  void setSwitchMode(byte value);
  int getNumCycles();
  void setNumCycles(int value);
  float[] getPulsePattern();
  void setPulsePattern(float[] value);
}
