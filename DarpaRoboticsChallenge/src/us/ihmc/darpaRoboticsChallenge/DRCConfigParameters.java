package us.ihmc.darpaRoboticsChallenge;


// Remove all parameters from this class and move to robot-specific interfaces. Change DRCConfigParametersTest to enforce less variables.
@Deprecated
public class DRCConfigParameters
{
   public static final boolean MAKE_SLIDER_BOARD = false; 

   // Log images from the primary camera
   public static boolean LOG_PRIMARY_CAMERA_IMAGES = false;

   // UI
   public static final boolean USE_COLLISIONS_MESHS_FOR_VISUALIZATION = false;

   public static final double FOOTSTEP_FITTING_BUFFER_SIZE = -0.01;
   public static boolean CALIBRATE_ARM_MODE = false;

   
   public static final boolean SEND_ROBOT_DATA_TO_ROS = false;
   
   // Do not add anything here. I'll shout at you. Jesper
}
