package us.ihmc.gdx.simulation.sensors;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;

import java.util.function.LongSupplier;

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
      return new GDXHighLevelDepthSensorSimulator("MultiSense Lidar",
                                                  null,
                                                  null,
                                                  null,
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
      return new GDXHighLevelDepthSensorSimulator("MultiSense Left Eye",
                                                  null,
                                                  null,
                                                  null,
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
      if (LOW_RESOLUTION_SENSORS)
      {
         imageWidth /= 2;
         imageHeight /= 2;
      }
      double minRange = 0.105;
      double maxRange = 5.0;
      return new GDXHighLevelDepthSensorSimulator("Detection D435",
                                                  ros1Node,
                                                  RosTools.D435_DEPTH,
                                                  RosTools.D435_DEPTH_CAMERA_INFO,
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
      int imageWidth = 1024;
      int imageHeight = 768;
//      if (LOW_RESOLUTION_SENSORS)
//      {
//         imageWidth /= 2;
//         imageHeight /= 2;
//      }
      double minRange = 0.105;
      double maxRange = 5.0;
      return new GDXHighLevelDepthSensorSimulator("Stepping L515",
                                                  ros1Node,
                                                  RosTools.MAPSENSE_DEPTH_IMAGE,
                                                  RosTools.MAPSENSE_DEPTH_CAMERA_INFO,
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
      if (LOW_RESOLUTION_SENSORS)
      {
         imageWidth /= 2;
         imageHeight /= 2;
      }
      double minRange = 0.105;
      double maxRange = 15.0;
      return new GDXHighLevelDepthSensorSimulator("Ouster Lidar",
                                                  null,
                                                  null,
                                                  null,
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

   public static GDXHighLevelDepthSensorSimulator createBlackflyFisheyeImageOnlyNoComms(ReferenceFrame sensorFrame)
   {
      double publishRateHz = 1.0;
      double verticalFOV = 100.0;
      int imageWidth = 1024;
      int imageHeight = 1024;
      double minRange = 0.105;
      double maxRange = 5.0;
      LongSupplier timeSupplier = null;
      return new GDXHighLevelDepthSensorSimulator("Blackfly Fisheye",
                                                  null,
                                                  null,
                                                  null,
                                                  null,
                                                  null,
                                                  null,
                                                  null,
                                                  null,
                                                  sensorFrame,
                                                  timeSupplier,
                                                  verticalFOV,
                                                  imageWidth,
                                                  imageHeight,
                                                  minRange,
                                                  maxRange,
                                                  publishRateHz,
                                                  false);
   }

   public static GDXHighLevelDepthSensorSimulator createL515ImageOnlyNoComms(ReferenceFrame sensorFrame)
   {
      double publishRateHz = 1.0;
      double verticalFOV = 55.0;
      int imageWidth = 640;
      int imageHeight = 480;
      double minRange = 0.105;
      double maxRange = 5.0;
      LongSupplier timeSupplier = null;
      return new GDXHighLevelDepthSensorSimulator("L515",
                                                  null,
                                                  null,
                                                  null,
                                                  null,
                                                  null,
                                                  null,
                                                  null,
                                                  null,
                                                  sensorFrame,
                                                  timeSupplier,
                                                  verticalFOV,
                                                  imageWidth,
                                                  imageHeight,
                                                  minRange,
                                                  maxRange,
                                                  publishRateHz,
                                                  false);
   }
}
