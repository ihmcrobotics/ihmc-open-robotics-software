package us.ihmc.atlas.parameters;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;

public class AtlasLegConfigurationParameters extends LegConfigurationParameters
{
   private final boolean runningOnRealRobot;

   public AtlasLegConfigurationParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   /** {@inheritDoc} */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }

   @Override
   /** {@inheritDoc} */
   public LegConfigurationGains getBentLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setJointSpaceKp(runningOnRealRobot ? 40.0 : 100.0);
      gains.setJointSpaceKd(6.0);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public LegConfigurationGains getStraightLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setActuatorSpaceKp(750.0);
      gains.setJointSpaceKd(runningOnRealRobot ? 3.0 : 6.0);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public double getLegPrivilegedHighWeight()
   {
      return runningOnRealRobot ? 50.0 : 100.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getFractionOfSwingToStraightenLeg()
   {
      return runningOnRealRobot ? 0.6 : 0.4;
   }

   /** {@inheritDoc} */
   @Override
   public double getKneeAngleWhenExtended()
   {
      return runningOnRealRobot ? 0.2 : 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getKneeAngleWhenStraight()
   {
      return runningOnRealRobot ? 0.35 : 0.25;
   }
}
