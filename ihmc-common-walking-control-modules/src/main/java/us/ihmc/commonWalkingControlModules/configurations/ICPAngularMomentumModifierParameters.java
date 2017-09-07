package us.ihmc.commonWalkingControlModules.configurations;

public class ICPAngularMomentumModifierParameters
{
   /**
    * Indicates whether or not we want to modify the ICP plan setpoints to account for the generated angular momentum.
    * This effectively moves the desired CMP location and desired ICP velocity to offset by the difference between
    * the ICP location and the estimated CoP location. The difference between these two points is a function of the
    * angular momentum.
    * @return modify ICP setpoints based on angular momentum.
    */
   public boolean getModifyICPPlanByAngularMomentumRate()
   {
      return false;
   }

   /**
    * Filter alpha for the difference between the desired CMP location and the estimated CoP location. Without filtering,
    * when using the module indicated by {@link #getModifyICPPlanByAngularMomentumRate()}, the controller tends to go unstable.
    * @return filter alpha
    */
   public double getCMPOffsetAlphaFilter()
   {
      return 0.9;
   }

   /**
    * Gain to multiply the forward CoP-CMP difference by in the sole frame.
    * @return gain
    */
   public double getAngularMomentumRateForwardGain()
   {
      return 1.2;
   }

   /**
    * Gain to multiply the lateral CoP-CMP difference by in the sole frame.
    * @return gain
    */
   public double getAngularMomentumRateLateralGain()
   {
      return 0.6;
   }
}
