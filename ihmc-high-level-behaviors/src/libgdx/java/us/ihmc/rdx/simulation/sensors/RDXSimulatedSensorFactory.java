package us.ihmc.rdx.simulation.sensors;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.sensors.realsense.RealSenseConfiguration;
import us.ihmc.sensors.realsense.RealSenseImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;

public class RDXSimulatedSensorFactory
{
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
