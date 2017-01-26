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
   public double getConfigurationGain()
   {
      return 40.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getVelocityGain()
   {
      return 6.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxVelocity()
   {
      return 2.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxAcceleration()
   {
      return Double.POSITIVE_INFINITY;
   }

   /** {@inheritDoc} */
   @Override
   public double getWeight()
   {
      return 5.0;
   }
}
