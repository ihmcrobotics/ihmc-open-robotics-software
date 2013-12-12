package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotParameters;

public class DRCConfigParameters
{
   public static final boolean USE_LOW_LEVEL_POSITION_CONTROL_FOR_HANDS = true;
   
   public static final boolean USE_RED_TEAM_CONTROLLER = false;
   public static final boolean USE_MANIPULATION_DECOUPLED_TASKSPACE_CONTROL = false;
   public static final boolean USE_INVERSE_KINEMATICS_TASKSPACE_CONTROL = false;

// When false and desired hand pose from the GUI is in world, the robot will hold the desired hand pose in world when walking. BE CAREFUL with that option! 
   public static final boolean HOLD_HANDS_IN_CHEST_FRAME_WHEN_WALKING = true;
   
   public static final double DEFAULT_SWING_TIME;
   public static final double DEFAULT_TRANSFER_TIME;
   
   static
   {
      if (DRCLocalConfigParameters.RUNNING_ON_REAL_ROBOT)
      {
         DEFAULT_SWING_TIME = 1.5;
         DEFAULT_TRANSFER_TIME = 1.5;
      }
      else 
      {
         DEFAULT_SWING_TIME = 0.6;
         DEFAULT_TRANSFER_TIME = 0.25;
      }
   }
   

   public static final boolean CORRUPT_SIMULATION_MODEL = false;

   public static final boolean USE_DUMMY_DRIVNG = false;
   public static final boolean RESTART_FOR_FANCY_CONTROL = true;    // Enable for testing standup

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

   public static final double ATLAS_INTERFACING_DT = 0.003;
   public static final double CONTROL_DT = 0.006;
   public static final double ATLAS_ONBOARD_SAMPLINGFREQ = 1000.0;
   public static final double ATLAS_ONBOARD_DT = 1.0 / ATLAS_ONBOARD_SAMPLINGFREQ;

   // Set whether or not to use GFE Robot Model
   public static final boolean USE_GFE_ROBOT_MODEL = true;

   // Convenience field
   public static final boolean USE_R2_ROBOT_MODEL = !USE_GFE_ROBOT_MODEL;

   // Video Source IDs
   public static final int MULTISENSE_LEFT_CAMERA = 1;
   public static final int MULTISENSE_RIGHT_CAMERA = 2;
   public static final int FISHEYE_LEFT_CAMERA = 3;
   public static final int FISHEYE_RIGHT_CAMERA = 4;
   public static final int REAR_CAMERA = 5;
   public static final int VIDEO_FRAME = 6;

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


   public static final String MULTISENSE_CAMERA_STRING_BASE;

   // ROS Topics
   public static final String FISHEYE_RIGHT_CAMERA_TOPIC;
   public static final String FISHEYE_LEFT_CAMERA_TOPIC;
   public static final String MULTISENSE_LEFT_CAMERA_TOPIC;
   public static final String MULTISENSE_RIGHT_CAMERA_TOPIC;

   public static final int GAZEBO_VER = DRCLocalConfigParameters.GAZEBO_VER;

   public static boolean LIDAR_ADJUSTMENT_ACTIVE = false;

   static
   {
      if (DRCLocalConfigParameters.USING_REAL_HEAD)
      {
         MULTISENSE_CAMERA_STRING_BASE = "/multisense_sl";

         // ROS Topics
         FISHEYE_RIGHT_CAMERA_TOPIC = "/blackfly/camera/right/compressed";
         FISHEYE_LEFT_CAMERA_TOPIC = "/blackfly/camera/left/compressed";
         MULTISENSE_LEFT_CAMERA_TOPIC = MULTISENSE_CAMERA_STRING_BASE + "/left/image_rect_color/compressed";
         MULTISENSE_RIGHT_CAMERA_TOPIC = MULTISENSE_CAMERA_STRING_BASE + "/right/image_rect/compressed";
      }
      else
      {
         if (GAZEBO_VER >= 3)
         {
            MULTISENSE_CAMERA_STRING_BASE = "/multisense_sl/camera";

            // ROS Topics
            FISHEYE_RIGHT_CAMERA_TOPIC = "/l_situational_awareness_camera/image_raw/compressed";
            FISHEYE_LEFT_CAMERA_TOPIC = "/r_situational_awareness_camera/image_raw/compressed";
            MULTISENSE_LEFT_CAMERA_TOPIC = MULTISENSE_CAMERA_STRING_BASE + "/left/image_rect_color/compressed";
            MULTISENSE_RIGHT_CAMERA_TOPIC = MULTISENSE_CAMERA_STRING_BASE + "/right/image_rect_color/compressed";
         }
         else
         {
            MULTISENSE_CAMERA_STRING_BASE = "/multisense_sl/camera";
            FISHEYE_RIGHT_CAMERA_TOPIC = "/blackfly/camera/right/compressed";
            FISHEYE_LEFT_CAMERA_TOPIC = "/blackfly/camera/left/compressed";
            MULTISENSE_LEFT_CAMERA_TOPIC = MULTISENSE_CAMERA_STRING_BASE + "/left/image_rect_color/compressed";
            MULTISENSE_RIGHT_CAMERA_TOPIC = MULTISENSE_CAMERA_STRING_BASE + "/right/image_rect_color/compressed";
         }

      }
   }


   // Video Settings
   public static final boolean STREAM_VIDEO = true;

   // UI
   public static final int UI_JOINT_CONFIGURATION_UPDATE_MILLIS = 100;
   public static final boolean USE_COLLISIONS_MESHS_FOR_VISUALIZATION = false;
   public static final boolean USE_SLIDER_FOR_POSE_PLAYBACK = false;
   public static final boolean USE_SUPER_DUPER_HIGH_RESOLUTION_FOR_COMMS = false;

   // State Estimator
   public static final boolean USE_STATE_ESTIMATOR = true;
   public static final boolean INTRODUCE_FILTERED_GAUSSIAN_POSITIONING_ERROR = false;
   public static final double NOISE_FILTER_ALPHA = 1e-1;
   public static final double POSITION_NOISE_STD = 0.01;
   public static final double QUATERNION_NOISE_STD = 0.01;
   public static final boolean ASSUME_PERFECT_IMU = true;    // assume perfect orientation, angular velocity and linear acceleration output from IMU
   public static final boolean USE_SIMPLE_PELVIS_POSITION_ESTIMATOR = true;



   // LIDAR Configuration - LIDAR filtering parameters now in LidarFilterParameters
   public static final double LIDAR_SPINDLE_VELOCITY = 5.1;
   
   /** LIDAR near scan configuration - enable or disable positioning the rotation origin on the near scan point cloud */
   public static final boolean LIDAR_ENABLE_NEAR_SCAN_MOUSE_COLLISIONS = DRCLocalConfigParameters.LIDAR_ENABLE_NEAR_SCAN_MOUSE_COLLISIONS; 

   // the useful children are "Static Link Graphic" and "atlas", but you don't really need atlas. ~30% faster without atlas.
   public static final String[] SCS_LIDAR_NODES_TO_INTERSECT = new String[] {"Static Link Graphic"};

   public static final boolean STREAM_POLAR_LIDAR = true;
   public static final int LIDAR_UPDATE_RATE_OVERRIDE = 30;
   public static final int LIDAR_SWEEPS_PER_SCAN = 1;

   public static final int LIDAR_POINTS_PER_SWEEP;
   public static final float LIDAR_SWEEP_MAX_YAW;
   public static final float LIDAR_SWEEP_MIN_YAW;

   static
   {
      if (DRCLocalConfigParameters.USING_REAL_HEAD)
      {
         final float crc = -.0010908f;
         LIDAR_POINTS_PER_SWEEP = 1081;
         LIDAR_SWEEP_MIN_YAW = -2.356194f + crc;
         LIDAR_SWEEP_MAX_YAW = 2.356194f + crc;

      }
      else
      {
         LIDAR_POINTS_PER_SWEEP = 720;
         LIDAR_SWEEP_MIN_YAW = -1.570796f;
         LIDAR_SWEEP_MAX_YAW = 1.570796f;
      }
   }

   public static final boolean OVERRIDE_DRC_LIDAR_CONFIG = true;
   public static final float LIDAR_MIN_DISTANCE = 0.2f;
   public static final float LIDAR_MAX_DISTANCE = 10.0f;

   public static final float LIDAR_SCAN_MAX_ROLL = 0.0f;    // rolls the LIDAR to

   // simulate a faster update rate.
   public static final float LIDAR_SCAN_MIN_ROLL = 0.0f;
   public static final float LIDAR_ANGLE_INCREMENT = (float) Math.toRadians(0.25);
   public static final float LIDAR_TIME_INCREMENT = 0.0f;
   public static final float LIDAR_SCAN_TIME = 0.0f;
   public static final double LIDAR_NOISE_LEVEL_OVERRIDE = 0.005;    // DRCGazebo

   // will simulate with: 0.005
   public static final boolean DEBUG_GAZEBO_LIDAR = false;

   // LIDAR Processor
   public static final boolean LIDAR_PROCESSOR_TIMING_REPORTING_ON = false;
   public static final double GRID_RESOLUTION = 0.025;    // in meters
   public static final double OCTREE_RESOLUTION_WHEN_NOT_USING_RESOLUTION_SPHERE = 0.025;
   public static final double FOOTSTEP_FITTING_BUFFER_SIZE = 0.02;

   // Footstep Generator
   public static final double BOUNDING_BOX_FOR_FOOTSTEP_HEIGHT_FINDING_SIDE_LENGTH =
      (1 + 0.3) * 2
      * Math.sqrt(DRCRobotParameters.DRC_ROBOT_FOOT_FORWARD * DRCRobotParameters.DRC_ROBOT_FOOT_FORWARD
                  + 0.25 * DRCRobotParameters.DRC_ROBOT_FOOT_WIDTH * DRCRobotParameters.DRC_ROBOT_FOOT_WIDTH);

   public static final int JOINT_CONFIGURATION_RATE_IN_MS = 10;

   public static final boolean USE_TABS_IN_UI = true;
   public static final boolean DO_AUTOMATED_STANDPREP = true;

   // Path parameters
   public static final double DEFAULT_SPINNING_DIAMETER = 0.4;

   // Hand Controller
   public static final boolean USE_PURE_POSITION_CONTROL_FOR_HANDS = false;

   public static final int CHEATING_POLARIS_PORT = 1543;
   public static final String CHEATING_POLARIS_HOST = "localhost";

   // Filter Parameters
   public static final double positionSensorFrequencyHz;
   public static final double velocitySensorFrequencyHz;

   static
   {
      if (!DRCLocalConfigParameters.RUNNING_ON_REAL_ROBOT)
      {
         positionSensorFrequencyHz = Double.POSITIVE_INFINITY;
         velocitySensorFrequencyHz = Double.POSITIVE_INFINITY;
      }
      else
      {
         positionSensorFrequencyHz = 16.0;
         velocitySensorFrequencyHz = 16.0;
      }
   }

   // Atlas on-board filtering parameters
   public static final boolean USE_IHMCFILTER_JOINT_ANGLES = false;
   public static final boolean USE_IHMCFILTER_JOINT_VELOCITIES = false;
   public static final boolean USE_IHMCFILTER_JOINT_TORQUES = true;
   public static final String ATLAS_CALIBRATION_FILE = "AtlasCalibration/AtlasNullOffsets.properties";

   public static final double JOINT_VELOCITY_SLOP_TIME_FOR_BACKLASH_COMPENSATION = 0.06;    // 0.045; //0.03;

   public static final double JOINT_POSITION_FILTER_FREQ_HZ = positionSensorFrequencyHz;
   public static final double JOINT_VELOCITY_FILTER_FREQ_HZ = velocitySensorFrequencyHz;
   public static final double ORIENTATION_FILTER_FREQ_HZ = positionSensorFrequencyHz;
   public static final double ANGULAR_VELOCITY_FILTER_FREQ_HZ = velocitySensorFrequencyHz;
   public static final double LINEAR_ACCELERATION_FILTER_FREQ_HZ = velocitySensorFrequencyHz;

   // State Estimator Filter Parameters
   public static final double pointVelocityXYMeasurementStandardDeviation = 2.0;    // 8.0; //2.0;
   public static final double pointVelocityZMeasurementStandardDeviation = 2.0;    // 8.0; //2.0;

   public static final double pointPositionXYMeasurementStandardDeviation = 0.1;    // 0.4; //0.1;
   public static final double pointPositionZMeasurementStandardDeviation = 0.1;    // 0.4; //0.1;

   public static final double contactTresholdForceForSCS = 5.0;
   public static final double contactTresholdForceForGazebo = 120.0;
   public static final double contactTresholdForceForRealAtlasRobot = 120.0;
}
