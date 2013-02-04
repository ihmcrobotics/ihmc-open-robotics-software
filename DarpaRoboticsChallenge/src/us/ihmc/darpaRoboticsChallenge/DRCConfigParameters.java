package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.graphics3DAdapter.camera.VideoSettings;
import us.ihmc.graphics3DAdapter.camera.VideoSettings.Quality;
import us.ihmc.graphics3DAdapter.camera.VideoSettingsH264LowLatency;


public class DRCConfigParameters
{
   // Set whether or not to use GFE Robot Model
   public static final boolean USE_GFE_ROBOT_MODEL = true;

   // Convenience field
   public static final boolean USE_R2_ROBOT_MODEL = !USE_GFE_ROBOT_MODEL;

   public static final boolean STREAM_VIDEO = true;

   // Networking
   public static final String SCS_MACHINE_IP_ADDRESS = "localhost"; //"10.100.0.37";
   public static final String OPERATOR_INTERFACE_IP_ADDRESS = "localhost"; //"10.4.8.1";
   public static final int BG_VIDEO_SERVER_PORT_NUMBER = 2099;

   public static final int ROBOT_DATA_RECEIVER_PORT_NUMBER = 7777;
   public static final long JOINT_DATA_IDENTIFIER = 5L;

   public static final int FOOTSTEP_PATH_PORT_NUMBER = 3333;
   public static final long FOOTSTEP_PATH_DATA_IDENTIFIER = 3333L;

   public static final int FOOTSTEP_STATUS_PORT_NUMBER = 4444;
   public static final long FOOTSTEP_STATUS_DATA_IDENTIFIER = 4444L;

   public static final int PAUSE_COMMAND_PORT_NUMBER = 5555;
   public static final long PAUSE_COMMAND_DATA_IDENTIFIER = 5555L;

   public static final int HEAD_ORIENTATION_PORT_NUMBER = 6666;
   public static final long HEAD_ORIENTATION_DATA_IDENTIFIER = 6666L;

   public static final int LIDAR_DATA_PORT_NUMBER = 4697;
   public static final long LIDAR_DATA_IDENTIFIER = 4697L;
   public static final int LIDAR_X_RESOLUTION_OVERRIDE = 50;
   
   public static final long ROBOT_JOINT_SERVER_UPDATE_MILLIS = 100;

   public static final boolean STREAM_LIDAR = true;
   
//   public static final VideoSettings VIDEOSETTINGS = new VideoSettingsH264LowLatency(800, 600, Quality.MEDIUMs);
   public static final VideoSettings VIDEOSETTINGS = new VideoSettingsH264LowLatency(200, 150, Quality.LOW);

   static final int LIDAR_UPDATE_RATE_OVERRIDE = 1;

   static final double LIDAR_VERTICAL_SCAN_ANGLE = 3.2;// optimistic, to say the least

   static final double LIDAR_HORIZONTAL_SCAN_ANGLE = 0.3;

   static final int LIDAR_SWEEPS_PER_SCAN = 60;

   static final int LIDAR_POINTS_PER_SWEEP = 10;

   static final boolean OVERRIDE_DRC_LIDAR_CONFIG = true;

}
