package us.ihmc.darpaRoboticsChallenge;


public class DRCConfigParameters
{
   // Set whether or not to use GFE Robot Model
   public static final boolean USE_GFE_ROBOT_MODEL = true;

   // Convenience field
   public static final boolean USE_R2_ROBOT_MODEL = !USE_GFE_ROBOT_MODEL;

   public static final boolean STREAM_VIDEO = true;

   // Networking
   public static final String SCS_MACHINE_IP_ADDRESS = "localhost"; //"10.100.0.37";
   public static final String OPERATOR_INTERFACE_IP_ADDRESS = "localhost";
   public static final int BG_VIDEO_SERVER_PORT_NUMBER = 2099;
   public static final int ROBOT_DATA_RECEIVER_PORT_NUMBER = 7777;

   public static final int FOOTSTEP_PATH_PORT_NUMBER = 3333;
   public static final long FOOTSTEP_PATH_DATA_IDENTIFIER = 3333L;

   public static final int FOOTSTEP_STATUS_PORT_NUMBER = 4444;
   public static final long FOOTSTEP_STATUS_DATA_IDENTIFIER = 4444L;

   public static final int PAUSE_COMMAND_PORT_NUMBER = 5555;
   public static final long PAUSE_COMMAND_DATA_IDENTIFIER = 5555L;

   public static final int LIDAR_DATA_PORT_NUMBER = 4697;
   
   public static final long ROBOT_JOINT_SERVER_UPDATE_MILLIS = 100;
   

}
