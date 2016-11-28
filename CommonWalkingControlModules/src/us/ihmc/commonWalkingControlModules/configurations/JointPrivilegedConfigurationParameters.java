package us.ihmc.commonWalkingControlModules.configurations;

public class JointPrivilegedConfigurationParameters
{
   public JointPrivilegedConfigurationParameters()
   {
   }

   public double getConfigurationGain()
   {
      return 100.0;
   }

   public double getVelocityGain()
   {
      return 10.0;
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
      return 10.0;
   }
}
