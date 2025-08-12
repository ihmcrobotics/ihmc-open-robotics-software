package us.ihmc.rdx.simulation.sensors;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.realsense.RealSenseConfiguration;
import us.ihmc.sensors.realsense.RealSenseImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;

import java.util.function.LongSupplier;

public class RDXSimulatedSensorFactory
{
   private static final boolean LOW_RESOLUTION_SENSORS = Boolean.parseBoolean(System.getProperty("low.resolution.sensors", "true"));

   public static RDXHighLevelDepthSensorSimulator createMultisenseLidar(ROS2SyncedRobotModel syncedRobot, ROS2Node ros2Node)
   {
      int pointsPerSweep = 720;
      int height = 1;
      double fovX = 170.0;
      double fovY = (fovX * (double) height) / (double) pointsPerSweep;
      double minRange = 0.1;
      double maxRange = 30.0f;
      double publishRateHz = 5.0;
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new RDXHighLevelDepthSensorSimulator("MultiSense Lidar",
                                                                                                            syncedRobot.getReferenceFrames()
                                                                                                                       .getLidarSensorFrame(),
                                                                                                            syncedRobot::getTimestamp,
                                                                                                            fovY,
                                                                                                            pointsPerSweep,
                                                                                                            height,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            0.001,
                                                                                                            0.001,
                                                                                                            false,
                                                                                                            publishRateHz);
      highLevelDepthSensorSimulator.setupForROS2PointCloud(ros2Node, PerceptionAPI.MULTISENSE_LIDAR_SCAN);
      return highLevelDepthSensorSimulator;
   }

   public static RDXHighLevelDepthSensorSimulator createMultisenseLeftEye(ROS2SyncedRobotModel syncedRobot, ROS2Node ros2Node)
   {
      double fovY = 49.0;
      int imageWidth = 1024;
      int imageHeight = 544;
      double minRange = 0.05;
      double maxRange = 30.0;
      double publishRateHz = 5.0;
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new RDXHighLevelDepthSensorSimulator("MultiSense Left Eye",
                                                                                                            syncedRobot.getReferenceFrames()
                                                                                                                       .getHeadCameraFrame(),
                                                                                                            syncedRobot::getTimestamp,
                                                                                                            fovY,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            0.001,
                                                                                                            0.001,
                                                                                                            false,
                                                                                                            publishRateHz);
      return highLevelDepthSensorSimulator;
   }

   public static RDXHighLevelDepthSensorSimulator createChestD435ForObjectDetection(ROS2SyncedRobotModel syncedRobot)
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
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new RDXHighLevelDepthSensorSimulator("Detection D435",
                                                                                                            syncedRobot.getReferenceFrames()
                                                                                                                       .getObjectDetectionCameraFrame(),
                                                                                                            syncedRobot::getTimestamp,
                                                                                                            verticalFOV,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            0.001,
                                                                                                            0.001,
                                                                                                            false,
                                                                                                            publishRateHz);
      return highLevelDepthSensorSimulator;
   }

   public static RDXHighLevelDepthSensorSimulator createChestD455ForMapSense(ROS2SyncedRobotModel syncedRobot)
   {
      return createRealsenseD455(syncedRobot.getReferenceFrames().getSteppingCameraFrame(), syncedRobot::getTimestamp);
   }

   public static RDXHighLevelDepthSensorSimulator createRealsenseD455(ReferenceFrame sensorFrame, LongSupplier timestampSupplier)
   {
      // These specs were pulled from the internet :)
      double publishRateHz = 30.0;
      double verticalFOV = 58.0;
      int imageWidth = 1280;
      int imageHeight = 720;
      double minRange = 0.5;
      double maxRange = 18.0;
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new RDXHighLevelDepthSensorSimulator("D455 RealSense",
                                                                                                            sensorFrame,
                                                                                                            timestampSupplier,
                                                                                                            verticalFOV,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            0.005,
                                                                                                            0.009,
                                                                                                            true,
                                                                                                            publishRateHz);
      return highLevelDepthSensorSimulator;
   }

   public static RDXHighLevelDepthSensorSimulator createChestL515ForMapSense(ROS2SyncedRobotModel syncedRobot)
   {
      return createRealsenseL515(syncedRobot.getReferenceFrames().getSteppingCameraFrame(), syncedRobot::getTimestamp);
   }

   public static RDXHighLevelDepthSensorSimulator createRealsenseL515(ReferenceFrame sensorFrame, LongSupplier timestampSupplier)
   {
      double publishRateHz = 20.0;
      double verticalFOV = 55.0;
      int imageWidth = 1024;
      int imageHeight = 768;
      double minRange = 0.105;
      double maxRange = 5.0;
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new RDXHighLevelDepthSensorSimulator("Stepping L515",
                                                                                                            sensorFrame,
                                                                                                            timestampSupplier,
                                                                                                            verticalFOV,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            0.005,
                                                                                                            0.009,
                                                                                                            true,
                                                                                                            publishRateHz);
      return highLevelDepthSensorSimulator;
   }

   public static RDXHighLevelDepthSensorSimulator createOusterLidar(ROS2SyncedRobotModel syncedRobot)
   {
      return createOusterLidar(syncedRobot.getReferenceFrames().getOusterLidarFrame(), syncedRobot::getTimestamp);
   }

   public static RDXHighLevelDepthSensorSimulator createOusterLidar(ReferenceFrame sensorFrame, LongSupplier timestampSupplier)
   {
      double publishRateHz = 4.0;
      double verticalFOV = 80.0;
      int imageWidth = 1024;
      int imageHeight = 128;
      double minRange = 0.105;
      double maxRange = 15.0;
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new RDXHighLevelDepthSensorSimulator("Ouster Lidar",
                                                                                                            sensorFrame,
                                                                                                            timestampSupplier,
                                                                                                            verticalFOV,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            0.015,
                                                                                                            0.05,
                                                                                                            false,
                                                                                                            publishRateHz);
      return highLevelDepthSensorSimulator;
   }

   public static RDXHighLevelDepthSensorSimulator createBlackflyFisheye(ROS2SyncedRobotModel syncedRobot)
   {
      return createBlackflyFisheye(syncedRobot.getReferenceFrames().getStereoCameraFrame(RobotSide.RIGHT), syncedRobot::getTimestamp);
   }

   public static RDXHighLevelDepthSensorSimulator createBlackflyFisheyeImageOnlyNoComms(ReferenceFrame sensorFrame)
   {
      return createBlackflyFisheye(sensorFrame, null);
   }

   public static RDXHighLevelDepthSensorSimulator createBlackflyFisheye(ReferenceFrame sensorFrame, LongSupplier timeSupplier)
   {
      double publishRateHz = 20.0;
      double verticalFOV = 100.0;
      int imageWidth = 1024;
      int imageHeight = 1024;
      double minRange = 0.105;
      double maxRange = 5.0;
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator = new RDXHighLevelDepthSensorSimulator("Blackfly Fisheye",
                                                                                                            sensorFrame,
                                                                                                            timeSupplier,
                                                                                                            verticalFOV,
                                                                                                            imageWidth,
                                                                                                            imageHeight,
                                                                                                            minRange,
                                                                                                            maxRange,
                                                                                                            0.001,
                                                                                                            0.001,
                                                                                                            false,
                                                                                                            publishRateHz);
      return highLevelDepthSensorSimulator;
   }

   public static RDXHighLevelDepthSensorSimulator createChestRightBlackflyForObjectDetection(ROS2SyncedRobotModel syncedRobot)
   {
      return createChestRightBlackflyForObjectDetection(syncedRobot.getReferenceFrames(), syncedRobot::getTimestamp);
   }

   public static RDXHighLevelDepthSensorSimulator createChestRightBlackflyForObjectDetection(HumanoidReferenceFrames referenceFrames,
                                                                                             LongSupplier timeSupplier)
   {
      double publishRateHz = 20.0;
      double verticalFOV = 100.0;
      int imageWidth = 1024;
      int imageHeight = 1024;
      double minRange = 0.105;
      double maxRange = 5.0;
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator
            = new RDXHighLevelDepthSensorSimulator("Blackfly Right for Object Detection",
                                                   referenceFrames.getStereoCameraFrame(RobotSide.RIGHT),
                                                   timeSupplier,
                                                   verticalFOV,
                                                   imageWidth,
                                                   imageHeight,
                                                   minRange,
                                                   maxRange,
                                                   0.01,
                                                   0.01,
                                                   false,
                                                   publishRateHz);
      highLevelDepthSensorSimulator.setupForROS2Color(PerceptionAPI.BLACKFLY_VIDEO.get(RobotSide.RIGHT));
      return highLevelDepthSensorSimulator;
   }

   public static RDXHighLevelDepthSensorSimulator createChestZED2ForObjectDetection(ROS2SyncedRobotModel syncedRobot)
   {
      return createChestZED2ForObjectDetection(syncedRobot.getReferenceFrames(), syncedRobot::getTimestamp);
   }

   public static RDXHighLevelDepthSensorSimulator createChestZED2ForObjectDetection(HumanoidReferenceFrames referenceFrames, LongSupplier timeSupplier)
   {
      double publishRateHz = 20.0;
      double verticalFOV = 70.0;
      int imageWidth = 1280;
      int imageHeight = 720;
      double minRange = 0.2;
      double maxRange = 40.0;
      RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator
            = new RDXHighLevelDepthSensorSimulator("ZED 2",
                                                   referenceFrames.getExperimentalCameraFrame(),
                                                   timeSupplier,
                                                   verticalFOV,
                                                   imageWidth,
                                                   imageHeight,
                                                   minRange,
                                                   maxRange,
                                                   0.01,
                                                   0.01,
                                                   false,
                                                   publishRateHz);
      return highLevelDepthSensorSimulator;
   }

   public static RDXSimulatedImageSensor createZEDMiniImageSensor()
   {
      ZEDModelData modelData = ZEDModelData.ZED_MINI;
      RDXSimulatedImageSensor zed = new RDXSimulatedImageSensor(modelData.name(), 15.0);
      zed.addCamera("left",
                    1280,
                    720,
                    52.0f,
                    modelData.getMinimumDepthDistance(),
                    modelData.getMaximumDepthDistance(),
                    true,
                    ZEDImageSensor.LEFT_COLOR_IMAGE_KEY,
                    true,
                    ZEDImageSensor.DEPTH_IMAGE_KEY,
                    15,
                    new RigidBodyTransform(new RotationMatrix(), new Vector3D(0.0, modelData.getCenterToCameraDistance(), 0.0)));
      zed.addCamera("right",
                    1280,
                    720,
                    52.0f,
                    modelData.getMinimumDepthDistance(),
                    modelData.getMaximumDepthDistance(),
                    true,
                    ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY,
                    false,
                    -1,
                    0,
                    new RigidBodyTransform(new RotationMatrix(), new Vector3D(0.0, -modelData.getCenterToCameraDistance(), 0.0)));

      return zed;
   }

   public static RDXSimulatedImageSensor createZED2iImageSensor()
   {
      ZEDModelData modelData = ZEDModelData.ZED_2I;
      RDXSimulatedImageSensor zed = new RDXSimulatedImageSensor(modelData.name(), 15.0);
      zed.addCamera("left",
                    1280,
                    720,
                    70.0f,
                    modelData.getMinimumDepthDistance(),
                    modelData.getMaximumDepthDistance(),
                    true,
                    ZEDImageSensor.LEFT_COLOR_IMAGE_KEY,
                    true,
                    ZEDImageSensor.DEPTH_IMAGE_KEY,
                    15,
                    new RigidBodyTransform(new RotationMatrix(), new Vector3D(0.0, modelData.getCenterToCameraDistance(), 0.0)));
      zed.addCamera("right",
                    1280,
                    720,
                    70.0f,
                    modelData.getMinimumDepthDistance(),
                    modelData.getMaximumDepthDistance(),
                    true,
                    ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY,
                    false,
                    -1,
                    0,
                    new RigidBodyTransform(new RotationMatrix(), new Vector3D(0.0, -modelData.getCenterToCameraDistance(), 0.0)));

      return zed;
   }

   public static RDXSimulatedImageSensor createD455ImageSensor()
   {
      RealSenseConfiguration config = RealSenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ;
      RDXSimulatedImageSensor d455 = new RDXSimulatedImageSensor(config.name().split("_")[0], config.getDepthFPS());
      RigidBodyTransform colorToDepthTransform = new RigidBodyTransform(new YawPitchRoll(0.0053, 0.0045, -0.0044), new Vector3D(0.0, -0.059, 0.0));
      d455.addCamera("color",
                     config.getColorWidth(),
                     config.getColorHeight(),
                     58.0f,
                     0.6f,
                     20.0f,
                     true,
                     RealSenseImageSensor.COLOR_IMAGE_KEY,
                     false,
                     -1,
                     0,
                     colorToDepthTransform);
      d455.addCamera("depth",
                     config.getDepthWidth(),
                     config.getDepthHeight(),
                     58.0f,
                     0.6f,
                     6.0f,
                     false,
                     -1,
                     true,
                     RealSenseImageSensor.DEPTH_IMAGE_KEY,
                     20,
                     new RigidBodyTransform());

      return d455;
   }
}
