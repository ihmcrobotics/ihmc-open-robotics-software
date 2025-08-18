package us.ihmc.sensors.zed;

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
   ZED(0.06, 0.2f, 40.0f, 68.0f),
   ZED_MINI(0.0315, 0.1f, 20.0f, 52.0f),
   ZED_2(0.06, 0.3f, 40.0f, 68.0f),
   ZED_2I(0.06, 0.2f, 40.0f, 68.0f),
   ZED_X(0.06, 0.3f, 20.0f, 78.0f),
   ZED_X_MINI(0.025, 0.1f, 8.0f, 78.0f);

   private final double centerToCameraDistance;
   private final float minimumDepthDistance;
   private final float maximumDepthDistance;
   private final float verticalFOV;

   ZEDModelData(double centerToCameraDistance, float minimumDepthDistance, float maximumDepthDistance, float verticalFOV)
   {
      this.centerToCameraDistance = centerToCameraDistance;
      this.minimumDepthDistance = minimumDepthDistance;
      this.maximumDepthDistance = maximumDepthDistance;
      this.verticalFOV = verticalFOV;
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

   public float getVerticalFOV()
   {
      return verticalFOV; // We use HD720 by default and SVGA for the X models
   }
}
