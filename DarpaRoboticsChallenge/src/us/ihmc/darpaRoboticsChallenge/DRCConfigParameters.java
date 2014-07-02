package us.ihmc.darpaRoboticsChallenge;


// Remove all parameters from this class and move to robot-specific interfaces. Change DRCConfigParametersTest to enforce less variables.
@Deprecated
public class DRCConfigParameters
{
   public static boolean CALIBRATE_ARM_MODE = false;

   public static final boolean USE_HYDRA = false;
   public static final boolean USE_MINI_ATLAS = false;

   // Networking
   public static final int NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT = 4895;
   public static final int NETWORK_PROCESSOR_TO_UI_TCP_PORT = 4897;
   public static final int NETWORK_PROCESSOR_TO_UI_RAW_PROTOCOL_TCP_PORT = 4898;
   public static final int NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_TCP_PORT = 5000;
   public static final int CONTROLLER_CLOUD_DISPATCHER_BACKEND_TCP_PORT = 5002;
   public static final int PPS_PROVIDER_PORT = 5050;

   // Log images from the primary camera
   public static boolean LOG_PRIMARY_CAMERA_IMAGES = false;

   public static boolean LIDAR_ADJUSTMENT_ACTIVE = false;
   public static boolean USE_POINT_CLOUD_INSTEAD_OF_LIDAR = false;


   // UI
   public static final int UI_JOINT_CONFIGURATION_UPDATE_MILLIS = 100;
   public static final boolean USE_COLLISIONS_MESHS_FOR_VISUALIZATION = false;
   public static final boolean USE_SLIDER_FOR_POSE_PLAYBACK = false;

   // LIDAR Processor
   public static final boolean LIDAR_PROCESSOR_TIMING_REPORTING_ON = false;
   public static final double GRID_RESOLUTION = 0.025; // in meters
   public static final double OCTREE_RESOLUTION_WHEN_NOT_USING_RESOLUTION_SPHERE = 0.025;
   public static final double FOOTSTEP_FITTING_BUFFER_SIZE = -0.01;

   public static final int CHEATING_POLARIS_PORT = 1543;
   public static final String CHEATING_POLARIS_HOST = "localhost";

   public static final boolean ALLOW_MODEL_CORRUPTION = false;

   public static final boolean SEND_ROBOT_DATA_TO_ROS = true;
   
}
