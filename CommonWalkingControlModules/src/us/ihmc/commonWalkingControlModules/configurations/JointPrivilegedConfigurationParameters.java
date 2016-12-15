package us.ihmc.commonWalkingControlModules.configurations;

public class JointPrivilegedConfigurationParameters
{
   public JointPrivilegedConfigurationParameters()
   {
   }

   public double getDefaultConfigurationGain()
   {
      return 40.0;
   }

   public double getDefaultVelocityGain()
   {
      return 6.0;
   }

   public double getDefaultMaxVelocity()
   {
      return 2.0;
   }

   public double getDefaultMaxAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }

   public double getDefaultWeight()
   {
      return 5.0;
   }
}
