package us.ihmc.zulu.parameters.controller;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;

public class ZuluJointPrivilegedConfigurationParameters extends JointPrivilegedConfigurationParameters
{
   /** {@inheritDoc} */
   @Override
   public double getDefaultConfigurationGain()
   {
      return 40.0;
   }

   @Override
   public double getNullspaceProjectionAlpha()
   {
      return 0.005;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultVelocityGain()
   {
      return 2.0;
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