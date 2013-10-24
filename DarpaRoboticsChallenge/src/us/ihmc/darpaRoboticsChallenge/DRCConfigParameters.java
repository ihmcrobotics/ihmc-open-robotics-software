package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig.LocalCloudMachines;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotParameters;

public class DRCConfigParameters
{
   public static final boolean USE_VRC_PARAMETERS = true;
   public static final boolean RUNNING_ON_REAL_ROBOT = false;
   public static final boolean INTEGRATE_ACCELERATIONS_AND_CONTROL_VELOCITIES = RUNNING_ON_REAL_ROBOT;
   public static final boolean CORRUPT_SIMULATION_MODEL = false;

   public static final boolean USE_DUMMY_DRIVNG = false;
   public static final boolean RESTART_FOR_FANCY_CONTROL = true;    // Enable for testing standup

   public static final boolean ALLOW_LAG_SIMULATION = true;
   public static final boolean ENABLE_LAG_SIMULATION_ON_START = false;

   static
   {
      if (ALLOW_LAG_SIMULATION)
      {
         System.err.println("Warning: Allowing simulation of lag");
      }
   }

   public static final boolean USE_COLLISIONS_MESHS_FOR_VISUALIZATION = false;
   public static final boolean SEND_HIGH_SPEED_CONFIGURATION_DATA = false;

   // TODO: Temporary static variable for testing getting into the car
   public static final boolean TEST_GETTING_INTO_CAR = false;    // true;

   public static final boolean USE_SLIDER_FOR_POSE_PLAYBACK = false;    // true;

   public static final boolean USE_SUPER_DUPER_HIGH_RESOLUTION_FOR_COMMS = false;

   public static final boolean USE_HYDRA = false;
   public static final boolean USE_FISHEYE = RUNNING_ON_REAL_ROBOT;

   public static final boolean LIMIT_CONTROLLER_OUTPUT_TORQUES = false;

   // Limit the controller to use only a certain percentage of maximum torque that the robot can provide
   public static final double MAX_TORQUE_TO_USE_IN_PERCENT = 0.98;

   public static final boolean USE_PERFECT_SENSORS = false;

   static
   {
      if (USE_PERFECT_SENSORS)
         System.err.println("Warning! Using Perfect Sensors!");
   }

   public static final String[] JOINTS_TO_IGNORE_FOR_GAZEBO = {"hokuyo_joint"};
   public static final boolean SHOW_SCS_GUI_FOR_GAZEBO = true;

   public static final boolean SHOW_BANDWIDTH_DIALOG = false;

   public static final double ESTIMATE_DT = 0.003;
   public static final double CONTROL_DT = 0.006;

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
   public static final String LOCALHOST = "localhost";
   public static final String CLOUD_MINION1_IP = DRCLocalCloudConfig.getIPAddress(LocalCloudMachines.CLOUDMINION_1);
   public static final String CLOUD_MINION2_IP = DRCLocalCloudConfig.getIPAddress(LocalCloudMachines.CLOUDMINION_2);
   public static final String CLOUD_MINION3_IP = DRCLocalCloudConfig.getIPAddress(LocalCloudMachines.CLOUDMINION_3);
   public static final String CLOUD_MINION4_IP = DRCLocalCloudConfig.getIPAddress(LocalCloudMachines.CLOUDMINION_4);
   public static final String CLOUD_MINION5_IP = DRCLocalCloudConfig.getIPAddress(LocalCloudMachines.CLOUDMINION_5);
   public static final String CLOUD_MINION6_IP = DRCLocalCloudConfig.getIPAddress(LocalCloudMachines.CLOUDMINION_6);
   public static final String CLOUD_MINION8_IP = DRCLocalCloudConfig.getIPAddress(LocalCloudMachines.CLOUDMINION_8);
   public static final String CLOUD_MONSTER_JR_IP = DRCLocalCloudConfig.getIPAddress(LocalCloudMachines.CLOUDMONSTER_JR);
   public static final String CLOUD_MONSTER_IP = DRCLocalCloudConfig.getIPAddress(LocalCloudMachines.CLOUDMONSTER);

   public static final String CONSTELLATION_SIMULATOR_COMPUTER_VPN_IP = "10.0.0.51";
   public static final String CONSTELLATION_FIELD_COMPUTER_1_VPN_IP = "10.0.0.52";
   public static final String CONSTELLATION_FIELD_COMPUTER_2_VPN_IP = "10.0.0.53";

   public static final Boolean USE_IROBOT_HAND = false;
   public static final String LEFT_IROBOT_HAND_IP = "192.168.40.38";
   public static final String RIGHT_IROBOT_HAND_IP = "192.168.40.32";

   public static final String LOG_HOST = "192.168.6.204";
   public static final String GAZEBO_HOST = "10.66.171.41";    // CONSTELLATION_SIMULATOR_COMPUTER_VPN_IP; //CONSTELLATION_SIMULATOR_COMPUTER_VPN_IP;

   public static final String SCS_MACHINE_IP_ADDRESS ;    
   static
   {
      if (RUNNING_ON_REAL_ROBOT)
      {
         SCS_MACHINE_IP_ADDRESS = "10.66.171.20";
      }
      else
      {
         SCS_MACHINE_IP_ADDRESS = LOCALHOST; // CONSTELLATION_FIELD_COMPUTER_2_VPN_IP; //CONSTELLATION_FIELD_COMPUTER_2_VPN_IP;    // CLOUD_MONSTER_IP;
      }
   }
   
   public static final String NET_PROC_MACHINE_IP_ADDRESS = LOCALHOST;    // CONSTELLATION_FIELD_COMPUTER_1_VPN_IP; //CONSTELLATION_FIELD_COMPUTER_1_VPN_IP;    // SCS_MACHINE_IP_ADDRESS;

   public static final String OPERATOR_INTERFACE_IP_ADDRESS = LOCALHOST;

   public static final String ROS_MASTER_URI = "http://" + GAZEBO_HOST + ":11311";
   public static final int CONTROLLER_TO_UI_TCP_PORT = 4893;

   public static final int NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT = 4895;

   public static final int NETWORK_PROCESSOR_TO_UI_TCP_PORT = 4897;

   public static final int NETWORK_PROCESSOR_TO_UI_RAW_PROTOCOL_TCP_PORT = 4898;

   public static final int NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_TCP_PORT = 5000;

   public static final int CONTROLLER_CLOUD_DISPATCHER_BACKEND_TCP_PORT = 5002;

   public static final int PPS_PROVIDER_PORT = 5050;

   public static final long ROBOT_JOINT_SERVER_UPDATE_MILLIS = 100;

   // Video Settings
   public static final boolean STREAM_VIDEO = true;

   // State Estimator
   public static final boolean USE_STATE_ESTIMATOR = true;
   public static final boolean INTRODUCE_FILTERED_GAUSSIAN_POSITIONING_ERROR = false;
   public static final double NOISE_FILTER_ALPHA = 1e-1;
   public static final double POSITION_NOISE_STD = 0.01;
   public static final double QUATERNION_NOISE_STD = 0.01;
   public static final boolean ASSUME_PERFECT_IMU = true;    // assume perfect orientation, angular velocity and linear acceleration output from IMU

   // LIDAR:
   public static final boolean USE_ROS_FOR_MULTISENSE_TRANSFORMS = RUNNING_ON_REAL_ROBOT;
   public static final boolean USING_REAL_HEAD = RUNNING_ON_REAL_ROBOT;
   public static final double LIDAR_SPINDLE_VELOCITY = 5.0;

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
      if (DRCConfigParameters.USING_REAL_HEAD)
      {
         LIDAR_POINTS_PER_SWEEP = 1081;
         LIDAR_SWEEP_MIN_YAW = -2.356194f;
         LIDAR_SWEEP_MAX_YAW = 2.356194f;

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
   public static final float LIDAR_MAX_DISTANCE = 20.0f;
   public static final float LIDAR_NEAR_SCAN_MAX_DISTANCE = 3.0f;

   public static final float LIDAR_SCAN_MAX_ROLL = 0.0f;    // rolls the LIDAR to

   // simulate a faster
   // update rate.
   public static final float LIDAR_SCAN_MIN_ROLL = 0.0f;
   public static final float LIDAR_ANGLE_INCREMENT = (float) Math.toRadians(0.25);
   public static final float LIDAR_TIME_INCREMENT = 0.0f;
   public static final float LIDAR_SCAN_TIME = 0.0f;
   public static final double LIDAR_NOISE_LEVEL_OVERRIDE = 0.005;    // DRCGazebo

   // will
   // simulate
   // with:
   // 0.005
   public static final boolean DEBUG_GAZEBO_LIDAR = false;

   // LIDAR Processor
   public static final boolean LIDAR_PROCESSOR_TIMING_REPORTING_ON = false;
   public static final double GRID_RESOLUTION = 0.025;    // in meters
   public static final double OCTREE_RESOLUTION_WHEN_NOT_USING_RESOLUTION_SPHERE = 0.05;
   public static final float QUADTREE_HEIGHT_THRESHOLD = 0.05f;
   public static final double LIDAR_BLINDNESS_CYLINDAR_SQUARED_RADIUS = 0.1;
   public static final boolean HIDE_THINGS_ABOVE_HEAD_FROM_LIDAR = true;

   // Footstep Generator
   public static final double BOUNDING_BOX_FOR_FOOTSTEP_HEIGHT_FINDING_SIDE_LENGTH =
      (1 + 0.3) * 2
      * Math.sqrt(DRCRobotParameters.DRC_ROBOT_FOOT_FORWARD * DRCRobotParameters.DRC_ROBOT_FOOT_FORWARD
                  + 0.25 * DRCRobotParameters.DRC_ROBOT_FOOT_WIDTH * DRCRobotParameters.DRC_ROBOT_FOOT_WIDTH);

   public static final int JOINT_CONFIGURATION_RATE_IN_MS = 10;

   // Resolution Sphere
   public static final boolean USE_RESOLUTION_SPHERE = true;
   public static final double LIDAR_RESOLUTION_SPHERE_OUTER_RESOLUTION = 0.10;
   public static final double LIDAR_RESOLUTION_SPHERE_OUTER_RADIUS = 16.0;
   public static final double LIDAR_RESOLUTION_SPHERE_INNER_RESOLUTION = 0.05;
   public static final double LIDAR_RESOLUTION_SPHERE_INNER_RADIUS = 8.0;
   public static final double LIDAR_RESOLUTION_SPHERE_DISTANCE_FROM_HEAD = 1.0;

   public static final boolean USE_TABS_IN_UI = true;

   // Hand Controller
   public static final boolean USE_PURE_POSITION_CONTROL_FOR_HANDS = false;

   public static final int CHEATING_POLARIS_PORT = 1543;
   public static final String CHEATING_POLARIS_HOST = LOCALHOST;
   
   // Filter Parameters
   public static final double  positionSensorFrequencyHz;
   public static final double  velocitySensorFrequencyHz;

   static
   {
      if (USE_VRC_PARAMETERS)
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
   
   public static final double JOINT_POSITION_FILTER_FREQ_HZ = positionSensorFrequencyHz;
   public static final double JOINT_VELOCITY_FILTER_FREQ_HZ = velocitySensorFrequencyHz;
   public static final double ORIENTATION_FILTER_FREQ_HZ = positionSensorFrequencyHz;
   public static final double ANGULAR_VELOCITY_FILTER_FREQ_HZ = velocitySensorFrequencyHz;
   public static final double LINEAR_ACCELERATION_FILTER_FREQ_HZ = velocitySensorFrequencyHz;

   // State Estimator Filter Parameters
   public static final double pointVelocityXYMeasurementStandardDeviation = 2.0; //8.0; //2.0;
   public static final double pointVelocityZMeasurementStandardDeviation = 2.0; //8.0; //2.0;

   public static final double pointPositionXYMeasurementStandardDeviation = 0.1; //0.4; //0.1;
   public static final double pointPositionZMeasurementStandardDeviation = 0.1; //0.4; //0.1;
}
