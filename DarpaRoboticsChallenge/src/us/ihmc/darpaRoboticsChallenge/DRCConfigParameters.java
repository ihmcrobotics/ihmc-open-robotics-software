package us.ihmc.darpaRoboticsChallenge;


public class DRCConfigParameters
{
   // Set whether or not to use GFE Robot Model
   public static final boolean USE_GFE_ROBOT_MODEL = true;

   // Convenience field
   public static final boolean USE_R2_ROBOT_MODEL = !USE_GFE_ROBOT_MODEL;

   public static final boolean STREAM_VIDEO = true;

   // Networking
   public static final String SCS_MACHINE_IP_ADDRESS = "localhost";
   public static final String OPERATOR_INTERFACE_IP_ADDRESS = "localhost";
   public static final int BG_VIDEO_SERVER_PORT_NUMBER = 2099;
   public static final int ROBOT_DATA_RECEIVER_PORT_NUMBER = 7777;
   public static final int FOOTSTEP_PLANNER_STREAMING_SERVER_PORT_NUMBER = 9876;
   public static final long FOOTSTEP_PLANNER_STREAMING_SERVER_DATA_IDENTIFIER = 9876L;
   public static final int LIDAR_DATA_PORT_NUMBER = 4697;
   
   public static final long ROBOT_JOINT_SERVER_UPDATE_MILLIS = 100;

}
