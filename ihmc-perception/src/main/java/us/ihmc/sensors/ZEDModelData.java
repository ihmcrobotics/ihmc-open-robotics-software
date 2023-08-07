package us.ihmc.sensors;

public enum ZEDModelData
{
   /**
    * Values used in initializing each camera model.
    * For sensors with wide and narrow FOV lens options, values from the wide lens models have been used.
    */
   ZED(0.6, 0.2f, 40.0f),
   ZED_MINI(0.315, 0.1f, 20.0f),
   ZED_2(0.6, 0.3f, 40.0f),
   ZED_2I(0.6, 0.2f, 40.0f),
   ZED_X(0.6, 0.3f, 20.0f),
   ZED_X_MINI(0.25, 0.1f, 8.0f);

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
}
