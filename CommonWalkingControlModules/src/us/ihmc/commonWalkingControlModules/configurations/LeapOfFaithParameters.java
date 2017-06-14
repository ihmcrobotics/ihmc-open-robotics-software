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

   public boolean useFootForce()
   {
      return false;
   }

   /**
    * This is the gain used to determine an additional downward force to be applied to the swing foot to force it to contact the ground.
    * It is multiplied by the time
    * @return
    */
   public double getFootForceGain()
   {
      return 100.0 * massScale;
   }

   public boolean usePelvisRelaxation()
   {
      return false;
   }

   public double getPelvisYawGain()
   {
      return 1.0;
   }

   public double getPelvisRollGain()
   {
      return 10.0;
   }
}
