package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;

public class AtlasJointPrivilegedConfigurationParameters extends JointPrivilegedConfigurationParameters
{
   private final boolean runningOnRealRobot;

   public AtlasJointPrivilegedConfigurationParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultConfigurationGain()
   {
      return 40.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultVelocityGain()
   {
      return 6.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultMaxVelocity()
   {
      return 2.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultMaxAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultWeight()
   {
      return 5.0;
   }
}
