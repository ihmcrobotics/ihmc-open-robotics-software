package us.ihmc.gdx.simulation.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;

public class GDXSimulatedSensorFactory
{
   private static final boolean LOW_RESOLUTION_SENSORS = Boolean.parseBoolean(System.getProperty("low.resolution.sensors", "true"));

   public static GDXHighLevelDepthSensorSimulator createMultisenseLidar(ROS2SyncedRobotModel syncedRobot, ROS2NodeInterface ros2Node)
   {
      int pointsPerSweep = 720;
      int height = 1;
      double fovX = 170.0;
      double fovY = (fovX * (double) height) / (double) pointsPerSweep;
      double minRange = 0.1;
      double maxRange = 30.0f;
      double fx = 500.0;
      double fy = 500.0;
      CameraPinholeBrown depthCameraIntrinsics = new CameraPinholeBrown(fx, fy, 0, pointsPerSweep / 2.0, height / 2.0, pointsPerSweep, height);
      return new GDXHighLevelDepthSensorSimulator("MultiSense Lidar",
                                                  null,
                                                  null,
                                                  null,
                                                  depthCameraIntrinsics,
                                                  null,
                                                  null,
                                                  ros2Node,
                                                  ROS2Tools.MULTISENSE_LIDAR_SCAN,
                                                  null,
                                                  syncedRobot.getReferenceFrames().getLidarSensorFrame(),
                                                  syncedRobot::getTimestamp,
                                                  fovY,
                                                  pointsPerSweep,
                                                  height,
                                                  minRange,
                                                  maxRange,
                                                  5.0,
                                                  false);
   }

   public static GDXHighLevelDepthSensorSimulator createMultisenseLeftEye(ROS2SyncedRobotModel syncedRobot, ROS2NodeInterface ros2Node)
   {
      double fovY = 49.0;
      int imageWidth = 1024;
      int imageHeight = 544;
      double minRange = 0.05;
      double maxRange = 30.0;
      double publishRateHz = 5.0;
      // TODO: Fix this so we can simulate video only
//      return new GDXHighLevelImageSensorSimulator("MultiSense Left Eye",
//                                                  null,
//                                                  null,
//                                                  null,
//                                                  ros2Node,
//                                                  ROS2Tools.VIDEO,
//                                                  syncedRobot.getReferenceFrames().getLidarSensorFrame(),
//                                                  syncedRobot::getTimestamp,
//                                                  fovY,
//                                                  imageWidth,
//                                                  imageHeight,
//                                                  minRange,
//                                                  maxRange,
//                                                  publishRateHz);
      double fx = 500.0;
      double fy = 500.0;
      CameraPinholeBrown depthCameraIntrinsics = new CameraPinholeBrown(fx, fy, 0, imageWidth / 2.0, imageHeight / 2.0, imageWidth, imageHeight);
      return new GDXHighLevelDepthSensorSimulator("MultiSense Left Eye",
                                                  null,
                                                  null,
                                                  null,
                                                  depthCameraIntrinsics,
                                                  null,
                                                  null,
                                                  ros2Node,
                                                  null,
                                                  ROS2Tools.VIDEO,
                                                  syncedRobot.getReferenceFrames().getHeadCameraFrame(),
                                                  syncedRobot::getTimestamp,
                                                  fovY,
                                                  imageWidth,
                                                  imageHeight,
                                                  minRange,
                                                  maxRange,
                                                  publishRateHz,
                                                  false);
   }

   public static GDXHighLevelDepthSensorSimulator createChestD435ForObjectDetection(ROS2SyncedRobotModel syncedRobot, RosNodeInterface ros1Node)
   {
      double publishRateHz = 5.0;
      double verticalFOV = 57.0;
      int imageWidth = 640;//320;
      int imageHeight = 360; //180;
      double fx = 500.0;
      double fy = 500.0;
      if (LOW_RESOLUTION_SENSORS)
      {
         imageWidth /= 2;
         imageHeight /= 2;
         fx /= 2;
         fy /= 2;
      }
      double minRange = 0.105;
      double maxRange = 5.0;
      CameraPinholeBrown depthCameraIntrinsics = new CameraPinholeBrown(fx, fy, 0, imageWidth / 2.0, imageHeight / 2.0, imageWidth, imageHeight);
      return new GDXHighLevelDepthSensorSimulator("Detection D435",
                                                  ros1Node,
                                                  RosTools.D435_DEPTH,
                                                  RosTools.D435_DEPTH_CAMERA_INFO,
                                                  depthCameraIntrinsics,
                                                  RosTools.D435_VIDEO,
                                                  RosTools.D435_CAMERA_INFO,
                                                  null,
                                                  null,
                                                  null,
                                                  syncedRobot.getReferenceFrames().getObjectDetectionCameraFrame(),
                                                  syncedRobot::getTimestamp,
                                                  verticalFOV,
                                                  imageWidth,
                                                  imageHeight,
                                                  minRange,
                                                  maxRange,
                                                  publishRateHz,
                                                  false);
   }

   public static GDXHighLevelDepthSensorSimulator createChestL515ForMapSense(ROS2SyncedRobotModel syncedRobot, RosNodeInterface ros1Node)
   {
      double publishRateHz = 5.0;
      double verticalFOV = 55.0;
      int imageWidth = 640;
      int imageHeight = 480;
      double fx = 500.0;
      double fy = 500.0;
//      if (LOW_RESOLUTION_SENSORS)
//      {
//         imageWidth /= 2;
//         imageHeight /= 2;
//         fx /= 2;
//         fy /= 2;
//      }
      double minRange = 0.105;
      double maxRange = 5.0;
      CameraPinholeBrown depthCameraIntrinsics = new CameraPinholeBrown(fx, fy, 0, imageWidth / 2.0, imageHeight / 2.0, imageWidth, imageHeight);
      return new GDXHighLevelDepthSensorSimulator("Stepping L515",
                                                  ros1Node,
                                                  RosTools.MAPSENSE_DEPTH_IMAGE,
                                                  RosTools.MAPSENSE_DEPTH_CAMERA_INFO,
                                                  depthCameraIntrinsics,
                                                  RosTools.L515_VIDEO,
                                                  RosTools.L515_COLOR_CAMERA_INFO,
                                                  null,
                                                  null,
                                                  null,
                                                  syncedRobot.getReferenceFrames().getSteppingCameraFrame(),
                                                  syncedRobot::getTimestamp,
                                                  verticalFOV,
                                                  imageWidth,
                                                  imageHeight,
                                                  minRange,
                                                  maxRange,
                                                  publishRateHz,
                                                  false);
   }

   public static GDXHighLevelDepthSensorSimulator createOusterLidar(ROS2SyncedRobotModel syncedRobot, ROS2NodeInterface ros2Node)
   {
      double publishRateHz = 5.0;
      double verticalFOV = 90.0;
      int imageWidth = 1024;
      int imageHeight = 128;
      double fx = 500.0;
      double fy = 500.0;
      if (LOW_RESOLUTION_SENSORS)
      {
         imageWidth /= 2;
         imageHeight /= 2;
         fx /= 2;
         fy /= 2;
      }
      double minRange = 0.105;
      double maxRange = 15.0;
      CameraPinholeBrown depthCameraIntrinsics = new CameraPinholeBrown(fx, fy, 0, imageWidth / 2.0, imageHeight / 2.0, imageWidth, imageHeight);
      return new GDXHighLevelDepthSensorSimulator("Ouster Lidar",
                                                  null,
                                                  null,
                                                  null,
                                                  depthCameraIntrinsics,
                                                  null,
                                                  null,
                                                  ros2Node,
                                                  ROS2Tools.MULTISENSE_LIDAR_SCAN,
                                                  null,
                                                  syncedRobot.getReferenceFrames().getOusterLidarFrame(),
                                                  syncedRobot::getTimestamp,
                                                  verticalFOV,
                                                  imageWidth,
                                                  imageHeight,
                                                  minRange,
                                                  maxRange,
                                                  publishRateHz,
                                                  false);
   }
}
