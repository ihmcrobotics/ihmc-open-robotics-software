package atlas_msgs;

public interface AtlasSimInterfaceCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasSimInterfaceCommand";
  static final java.lang.String _DEFINITION = "# For interfacing AtlasSimInterface Dynamics Behavior Library\n# This ROS message should track behavior commands in AtlasControlInput struct\n# inside AtlasSimInterfaceTypes.h.\n# With the exception of addition of k_effort to provide user a way to switch\n# to/from PID servo control in AtlasPlugin.cpp on a per joint basis.\n\nHeader header\n\n# permissible values for behavior\nint32 STAND             =  0 # stand\nint32 USER              =  1 # disable AtlasSimInterface updates, rely on\n                             # /atlas/atlas_command or /atlas/joint_commands\nint32 FREEZE            =  2 # safety mode\nint32 STAND_PREP        =  3 # stand-prep (AtlasSimInterface documentation)\nint32 WALK              =  4 # multi-step walk\nint32 STEP              =  5 # single step walk\nint32 MANIPULATE        =  6 # stand and allows manipulation.\n\nint32 behavior                # can be one of\n                              # USER, FREEZE, STAND_PREP\n                              # WALK, STEP, STAND, MANIPULATE\n\n# multi_step walking trajectory parameters\natlas_msgs/AtlasBehaviorWalkParams walk_params\n\n# parameters for single_step behavior\natlas_msgs/AtlasBehaviorStepParams step_params\n\n# parameters for standing behavior\natlas_msgs/AtlasBehaviorStandParams stand_params\n\n# parameters for stand and manipulate\natlas_msgs/AtlasBehaviorManipulateParams manipulate_params\n\n# additional vector for transitioning from servo model in AtlasPlugin\n# to BDI servo.\n\nuint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, \n                       # at run time, a double between 0 and 1 is obtained\n                       # by dividing by 255.0d.\n\n";
  static final int STAND = 0;
  static final int USER = 1;
  static final int FREEZE = 2;
  static final int STAND_PREP = 3;
  static final int WALK = 4;
  static final int STEP = 5;
  static final int MANIPULATE = 6;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getBehavior();
  void setBehavior(int value);
  atlas_msgs.AtlasBehaviorWalkParams getWalkParams();
  void setWalkParams(atlas_msgs.AtlasBehaviorWalkParams value);
  atlas_msgs.AtlasBehaviorStepParams getStepParams();
  void setStepParams(atlas_msgs.AtlasBehaviorStepParams value);
  atlas_msgs.AtlasBehaviorStandParams getStandParams();
  void setStandParams(atlas_msgs.AtlasBehaviorStandParams value);
  atlas_msgs.AtlasBehaviorManipulateParams getManipulateParams();
  void setManipulateParams(atlas_msgs.AtlasBehaviorManipulateParams value);
  org.jboss.netty.buffer.ChannelBuffer getKEffort();
  void setKEffort(org.jboss.netty.buffer.ChannelBuffer value);
}
