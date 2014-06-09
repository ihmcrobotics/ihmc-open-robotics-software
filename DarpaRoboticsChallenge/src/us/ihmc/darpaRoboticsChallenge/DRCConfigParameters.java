package us.ihmc.darpaRoboticsChallenge;


public class DRCConfigParameters
{
   public static final boolean USE_LOW_LEVEL_POSITION_CONTROL_FOR_HANDS = true;
   
   public static final boolean USE_RED_TEAM_CONTROLLER = false;
   public static final boolean USE_INVERSE_KINEMATICS_TASKSPACE_CONTROL = false;

// When false and desired hand pose from the GUI is in world, the robot will hold the desired hand pose in world when walking. BE CAREFUL with that option! 
   public static final boolean HOLD_HANDS_IN_CHEST_FRAME_WHEN_WALKING = true;
   
   public static final boolean CORRUPT_SIMULATION_MODEL = false;

   public static final boolean ALLOW_LAG_SIMULATION = true;
   public static final boolean ENABLE_LAG_SIMULATION_ON_START = false;
   public static boolean CALIBRATE_ARM_MODE = false;
   public static boolean USE_CALIBRATED_JOINT_BIAS = true;
   public static boolean ENABLE_QOUT_ENCODER = CALIBRATE_ARM_MODE;


   static
   {
      if (ALLOW_LAG_SIMULATION)
      {
         System.err.println("Warning: Allowing simulation of lag");
      }

      if (CALIBRATE_ARM_MODE)
      {
         System.err.println("Warning: Calibrate arm mode is on, qout is disabled");
      }
   }


   public static final boolean USE_HYDRA = false;
   public static final boolean USE_MINI_ATLAS = false;

   public static final boolean LIMIT_CONTROLLER_OUTPUT_TORQUES = false;    // true;//True causes hip oscillations or jerk in simulation

   // Limit the controller to use only a certain percentage of maximum torque that the robot can provide
   public static final double MAX_TORQUE_TO_USE_IN_PERCENT = 1.2;

   public static final boolean USE_PERFECT_SENSORS = false;

   // Add more contact points on the feet
   public static boolean CREATE_LOAD_OF_CONTACT_POINTS_FEET = false;
   public static boolean USE_SIX_CONTACT_POINTS_PER_FOOT = false;

   static
   {
      if (USE_PERFECT_SENSORS)
         System.err.println("Warning! Using Perfect Sensors!");
   }

   public static final String[] JOINTS_TO_IGNORE_FOR_GAZEBO = {"hokuyo_joint"};
   public static final boolean SHOW_SCS_GUI_FOR_GAZEBO = true;

   public static final boolean SHOW_BANDWIDTH_DIALOG = false;


   

   // Networking
   public static final String CONSTELLATION_SIMULATOR_COMPUTER_VPN_IP = "10.0.0.51";
   public static final String CONSTELLATION_FIELD_COMPUTER_1_VPN_IP = "10.0.0.52";
   public static final String CONSTELLATION_FIELD_COMPUTER_2_VPN_IP = "10.0.0.53";

   public static final String ROS_MASTER_URI = "http://" + DRCLocalConfigParameters.ROS_HOST_IP_ADDRESS + ":11311";
   public static final int CONTROLLER_TO_UI_TCP_PORT = 4893;

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
   // Video Settings
   public static final boolean STREAM_VIDEO = true;

   // UI
   public static final int UI_JOINT_CONFIGURATION_UPDATE_MILLIS = 100;
   public static final boolean USE_COLLISIONS_MESHS_FOR_VISUALIZATION = false;
   public static final boolean USE_SLIDER_FOR_POSE_PLAYBACK = false;
   public static final boolean USE_SUPER_DUPER_HIGH_RESOLUTION_FOR_COMMS = false;

   // LIDAR Configuration - LIDAR filtering parameters now in LidarFilterParameters
   public static final double LIDAR_SPINDLE_VELOCITY = 5.1;
   
   // will simulate with: 0.005
   public static final boolean DEBUG_GAZEBO_LIDAR = false;

   // LIDAR Processor
   public static final boolean LIDAR_PROCESSOR_TIMING_REPORTING_ON = false;
   public static final double GRID_RESOLUTION = 0.025;    // in meters
   public static final double OCTREE_RESOLUTION_WHEN_NOT_USING_RESOLUTION_SPHERE = 0.025;
   public static final double FOOTSTEP_FITTING_BUFFER_SIZE = -0.01;

   public static final boolean DO_AUTOMATED_STANDPREP = true;

   // Hand Controller
   public static final boolean USE_PURE_POSITION_CONTROL_FOR_HANDS = false;

   public static final int CHEATING_POLARIS_PORT = 1543;
   public static final String CHEATING_POLARIS_HOST = "localhost";

   // Atlas on-board filtering parameters
   public static final boolean USE_IHMCFILTER_JOINT_ANGLES = false;
   public static final boolean USE_IHMCFILTER_JOINT_VELOCITIES = false;
   public static final boolean USE_IHMCFILTER_JOINT_TORQUES = true;
   public static final String ATLAS_CALIBRATION_FILE = "AtlasCalibration/AtlasNullOffsets.properties";

   public static final double contactTresholdForceForSCS = 5.0;
   public static final double contactTresholdForceForGazebo = 120.0;

   public static final boolean ALLOW_MODEL_CORRUPTION = false;
}
