package us.ihmc.commonWalkingControlModules.configurations;

public class LeapOfFaithParameters
{
   public boolean useFootForce()
   {
      return false;
   }

   public double getFootForceGain()
   {
      return 100.0;
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
