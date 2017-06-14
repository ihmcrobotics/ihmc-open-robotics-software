package us.ihmc.commonWalkingControlModules.configurations;

public class LeapOfFaithParameters
{
   public final double massScale;

   public LeapOfFaithParameters()
   {
      this(1.0);
   }

   public LeapOfFaithParameters(double massScale)
   {
      this.massScale = massScale;
   }

   /**
    * Specifies whether or not to apply an external force to the end of the swing foot to force it to touchdown.
    * @return whether or not to push with the foot
    */
   public boolean useFootForce()
   {
      return false;
   }

   /**
    * This is the gain used to determine an additional downward force to be applied to the swing foot to force it to contact the ground.
    * It is multiplied by the time past when we expected touchdown to compute the external force to be exerted by the foot.
    *
    * @return force gain
    */
   public double getFootForceGain()
   {
      return 100.0 * massScale;
   }

   /**
    * Specifies whether or not to start rotating the pelvis to set the foot down.
    * @return whether or not to rotate the pelvis
    */
   public boolean usePelvisRotation()
   {
      return false;
   }

   /**
    * Gain to modify the desired pelvis yaw cause the pelvis to yaw towards the step to help set the foot down.
    * It is multiplied by the time past when we expected touchdown to compute the angle offset of the pelvis yaw.
    *
    * @return yaw gain
    */
   public double getPelvisYawGain()
   {
      return 1.0;
   }

   /**
    * Gain to modify the desired pelvis roll cause the pelvis to roll towards the step to help set the foot down.
    * It is multiplied by the time past when we expected touchdown to compute the angle offset of the pelvis roll.
    *
    * @return roll gain
    */
   public double getPelvisRollGain()
   {
      return 10.0;
   }
}
