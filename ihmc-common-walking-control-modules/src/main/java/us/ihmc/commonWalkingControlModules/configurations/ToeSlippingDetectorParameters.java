package us.ihmc.commonWalkingControlModules.configurations;

public class ToeSlippingDetectorParameters
{
   /**
    * Returns the forceMagnitudeThreshold. As long as the foot force magnitude remains above this
    * threshold, the toe will be considered as not slipping.
    */
   public double getForceMagnitudeThreshold()
   {
      return 25.0;
   }

   /**
    * Returns the velocityThreshold. This is one of the conditions to trigger the slipping detection: the
    * toe linear velocity magnitude has to be greater than this threshold.
    */
   public double getVelocityThreshold()
   {
      return 0.4;
   }

   /**
    * Returns the slippageDistanceThreshold. This is one of the condition to trigger the slipping
    * detection: the amount of slipping has to be greater than this threshold.
    */
   public double getSlippageDistanceThreshold()
   {
      return 0.04;
   }

   /**
    * Returns the filterBreakFrequency. This the break frequency to use for the internal low-pass filters.
    */
   public double getFilterBreakFrequency()
   {
      return 10.0;
   }
}
