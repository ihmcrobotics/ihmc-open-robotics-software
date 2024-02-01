package us.ihmc.perception.logging;

public class PerceptionLoggerConstants
{
   public static final String HDF5_FILE_EXTENSION = ".hdf5";

   public static final int COMPRESSED_IMAGE_BUFFER_SIZE = 1000000;
   public static final int BYTE_BUFFER_SIZE = 1000000;
   public static final int INT_BUFFER_SIZE = 100000;
   public static final int FLOAT_BUFFER_SIZE = 10000;
   public static final int LONG_BUFFER_SIZE = 10000;
   public static final int DOUBLE_BUFFER_SIZE = 10000;

   public static final int DEFAULT_BLOCK_SIZE = 100;
   public static final int LEGACY_BLOCK_SIZE = 10;

   public static final String ROBOT_CONFIGURATION_DATA_NAME = "/robot/root/position/";
   public static final String ROBOT_CONFIGURATION_DATA_MONOTONIC_TIME = "/robot/configuration/timestamps/";

   public static final String ROOT_POSITION_NAME = "/robot/root/position/";
   public static final String ROOT_ORIENTATION_NAME = "/robot/root/orientation/";
   public static final String JOINT_ANGLES_NAME = "/robot/joint_angles/";
   public static final String JOINT_VELOCITIES_NAME = "/robot/joint_velocities/";
   public static final String JOINT_TORQUES_NAME = "/robot/joint_torques/";

   public static final String OUSTER_CLOUD_NAME = "/os_cloud_node/points/";
   public static final String OUSTER_SENSOR_TIME = "/ouster/sensor/time";
   public static final String OUSTER_DEPTH_NAME = "/ouster/depth/";
   public static final String OUSTER_SENSOR_POSITION = "/ouster/sensor/position";
   public static final String OUSTER_SENSOR_ORIENTATION = "/ouster/sensor/orientation";

   public static final String BLACKFLY_COLOR_TIME = "/blackfly/color/time";
   public static final String BLACKFLY_COLOR_NAME = "/blackfly/color";

   public static final String D435_SENSOR_TIME = "/d435/sensor/time";
   public static final String D435_DEPTH_NAME = "/d435/depth/";
   public static final String D435_COLOR_NAME = "/d435/color/";
   public static final String D435_SENSOR_POSITION = "/d435/sensor/position/";
   public static final String D435_SENSOR_ORIENTATION = "/d435/sensor/orientation/";

   public static final String ZED2_SENSOR_TIME = "/zed2/sensor/time";
   public static final String ZED2_COLOR_NAME = "/zed2/color/";
   public static final String ZED2_DEPTH_NAME = "/zed2/depth/";
   public static final String ZED2_TIME_NAME = "/zed2/time/";

   public static final String L515_SENSOR_TIME = "/l515/sensor/time";
   public static final String L515_DEPTH_NAME = "/l515/depth/";
   public static final String L515_COLOR_NAME = "/l515/color/";
   public static final String L515_SENSOR_POSITION = "/l515/sensor/position/";
   public static final String L515_SENSOR_ORIENTATION = "/l515/sensor/orientation/";

   public static final String MOCAP_RIGID_BODY_TIME = "/mocap/rigid_body/position";
   public static final String MOCAP_RIGID_BODY_POSITION = "/mocap/rigid_body/position";
   public static final String MOCAP_RIGID_BODY_ORIENTATION = "/mocap/rigid_body/orientation";

   public static final String INTERNAL_HEIGHT_MAP_NAME = "/internal/height/";
   public static final String CROPPED_HEIGHT_MAP_NAME = "/cropped/height/";
   public static final String SENSOR_CROPPED_HEIGHT_MAP_NAME = "/sensor/cropped/height/";
   public static final String FOOTSTEP_SIDE = "/plan/footstep/side/";
   public static final String FOOTSTEP_POSITION = "/plan/footstep/position/";
   public static final String FOOTSTEP_ORIENTATION = "/plan/footstep/orientation/";
   public static final String START_FOOTSTEP_POSITION = "/start/footstep/position/";
   public static final String START_FOOTSTEP_ORIENTATION = "/start/footstep/orientation/";
   public static final String GOAL_FOOTSTEP_POSITION = "/goal/footstep/position/";
   public static final String GOAL_FOOTSTEP_ORIENTATION = "/goal/footstep/orientation/";
   public static final String INITIAL_FOOTSTEP_SIDE = "/initial/side/";
}
