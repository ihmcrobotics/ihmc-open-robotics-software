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
      double publishRateHz = 5.0;
      GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("MultiSense Lidar",
                                                                                                            syncedRobot.getReferenceFrames()
                                                                                                                       .getLidarSensorFrame(),
                                                                                                            syncedRobot::getTimestamp,
                                                                                                            fovY,
                                                                                                            pointsPerSweep,
                                                                                                            height,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            publishRateHz);
      highLevelDepthSensorSimulator.setupForROS2PointCloud(ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);
      return highLevelDepthSensorSimulator;
   }

   public static GDXHighLevelDepthSensorSimulator createMultisenseLeftEye(ROS2SyncedRobotModel syncedRobot, ROS2NodeInterface ros2Node)
   {
      double fovY = 49.0;
      int imageWidth = 1024;
      int imageHeight = 544;
      double minRange = 0.05;
      double maxRange = 30.0;
      double publishRateHz = 5.0;
      GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("MultiSense Left Eye",
                                                                                                            syncedRobot.getReferenceFrames()
                                                                                                                       .getHeadCameraFrame(),
                                                                                                            syncedRobot::getTimestamp,
                                                                                                            fovY,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            publishRateHz);
      return highLevelDepthSensorSimulator;
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
      GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("Detection D435",
                                                                                                            syncedRobot.getReferenceFrames()
                                                                                                                       .getObjectDetectionCameraFrame(),
                                                                                                            syncedRobot::getTimestamp,
                                                                                                            verticalFOV,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            publishRateHz);
      highLevelDepthSensorSimulator.setupForROS1Depth(ros1Node, RosTools.D435_DEPTH, RosTools.D435_DEPTH_CAMERA_INFO);
      highLevelDepthSensorSimulator.setupForROS1Color(ros1Node, RosTools.D435_VIDEO, RosTools.D435_CAMERA_INFO);
      return highLevelDepthSensorSimulator;
   }

   public static GDXHighLevelDepthSensorSimulator createChestL515ForMapSense(ROS2SyncedRobotModel syncedRobot)
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
      GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("Stepping L515",
                                                                                                            syncedRobot.getReferenceFrames()
                                                                                                                       .getSteppingCameraFrame(),
                                                                                                            syncedRobot::getTimestamp,
                                                                                                            verticalFOV,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            publishRateHz);
      return highLevelDepthSensorSimulator;
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
      GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("Ouster Lidar",
                                                                                                            syncedRobot.getReferenceFrames()
                                                                                                                       .getOusterLidarFrame(),
                                                                                                            syncedRobot::getTimestamp,
                                                                                                            verticalFOV,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            publishRateHz);
      highLevelDepthSensorSimulator.setupForROS2PointCloud(ros2Node, ROS2Tools.MULTISENSE_LIDAR_SCAN);
      return highLevelDepthSensorSimulator;
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
      GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("Blackfly Fisheye",
                                                                                                            sensorFrame,
                                                                                                            timeSupplier,
                                                                                                            verticalFOV,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            publishRateHz);
      return highLevelDepthSensorSimulator;
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
      GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("L515",
                                                                                                            sensorFrame,
                                                                                                            timeSupplier,
                                                                                                            verticalFOV,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            publishRateHz);
      return highLevelDepthSensorSimulator;
   }
}
