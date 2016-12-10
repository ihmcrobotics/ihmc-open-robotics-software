package us.ihmc.commonWalkingControlModules.configurations;

public class JointPrivilegedConfigurationParameters
{
   public JointPrivilegedConfigurationParameters()
   {
   }

   public double getConfigurationGain()
   {
      return 40.0;
   }

   public double getVelocityGain()
   {
      return 6.0;
   }

   public double getMaxVelocity()
   {
      return 2.0;
   }

   public double getMaxAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }

   public double getWeight()
   {
      return 5.0;
   }
}
