package us.ihmc.rdx.simulation.sensors;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.sensors.realsense.RealSenseConfiguration;
import us.ihmc.sensors.realsense.RealSenseImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;

import java.util.function.LongSupplier;

public class RDXSimulatedSensorFactory
{
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

   public static RDXHighLevelDepthSensorSimulator createChestZED2ForObjectDetection(HumanoidReferenceFrames referenceFrames, LongSupplier timeSupplier)
   {
      double publishRateHz = 20.0;
      double verticalFOV = ZEDModelData.ZED_2.getVerticalFOV();
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
                    ZEDModelData.ZED_MINI.getVerticalFOV(),
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
                    ZEDModelData.ZED_MINI.getVerticalFOV(),
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

   public static RDXSimulatedImageSensor createZEDXMiniImageSensor()
   {
      ZEDModelData modelData = ZEDModelData.ZED_X_MINI;
      RDXSimulatedImageSensor zed = new RDXSimulatedImageSensor(modelData.name(), 15.0);
      zed.addCamera("left",
                    960,
                    600,
                    ZEDModelData.ZED_X_MINI.getVerticalFOV(),
                    modelData.getMinimumDepthDistance(),
                    modelData.getMaximumDepthDistance(),
                    true,
                    ZEDImageSensor.LEFT_COLOR_IMAGE_KEY,
                    true,
                    ZEDImageSensor.DEPTH_IMAGE_KEY,
                    15,
                    new RigidBodyTransform(new RotationMatrix(), new Vector3D(0.0, modelData.getCenterToCameraDistance(), 0.0)));
      zed.addCamera("right",
                    960,
                    600,
                    ZEDModelData.ZED_X_MINI.getVerticalFOV(),
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
                    ZEDModelData.ZED_2I.getVerticalFOV(),
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
                    ZEDModelData.ZED_2I.getVerticalFOV(),
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
                     65.0f,
                     0.52f,
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
