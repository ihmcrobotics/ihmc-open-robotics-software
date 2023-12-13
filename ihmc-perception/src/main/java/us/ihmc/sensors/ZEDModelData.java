package us.ihmc.sensors;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public enum ZEDModelData
{
   /**
    * Values used in initializing each camera model.
    * For sensors with wide and narrow FOV lens options, values from the wide lens models have been used.
    * This has been done as sl_get_camera_model() does not differentiate between 2.2mm and 4mm lens models.
    * Assuming the use of wide angle (2.2mm) models seems safe for now (2023).
    */
   ZED(0.06, 0.2f, 40.0f),
   ZED_MINI(0.0315, 0.1f, 20.0f),
   ZED_2(0.06, 0.3f, 40.0f),
   ZED_2I(0.06, 0.2f, 40.0f),
   ZED_X(0.06, 0.3f, 20.0f),
   ZED_X_MINI(0.025, 0.1f, 8.0f);

   private final double centerToCameraDistance;
   private final float minimumDepthDistance;
   private final float maximumDepthDistance;

   ZEDModelData(double centerToCameraDistance, float minimumDepthDistance, float maximumDepthDistance)
   {
      this.centerToCameraDistance = centerToCameraDistance;
      this.minimumDepthDistance = minimumDepthDistance;
      this.maximumDepthDistance = maximumDepthDistance;
   }

   public double getCenterToCameraDistance()
   {
      return centerToCameraDistance;
   }

   public float getMinimumDepthDistance()
   {
      return minimumDepthDistance;
   }

   public float getMaximumDepthDistance()
   {
      return maximumDepthDistance;
   }

   public static ReferenceFrame createCameraReferenceFrame(RobotSide cameraSide, ReferenceFrame zed2CenterFrame)
   {
      RigidBodyTransform zed2LeftCameraToCenterTransform = new RigidBodyTransform();
      zed2LeftCameraToCenterTransform.getTranslation().set(0.0, cameraSide.negateIfRightSide(0.06), 0.0);

      return ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("ZED2%sCameraFrame".formatted(cameraSide.getPascalCaseName()),
                                                                               zed2CenterFrame,
                                                                               zed2LeftCameraToCenterTransform);
   }
}
