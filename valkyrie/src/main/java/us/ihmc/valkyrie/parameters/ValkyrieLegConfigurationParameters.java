package us.ihmc.valkyrie.parameters;

import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;

public class ValkyrieLegConfigurationParameters extends LegConfigurationParameters
{
   private final boolean runningOnRealRobot;

   public ValkyrieLegConfigurationParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   /** {@inheritDoc} */
   @Override
   public LegConfigurationGains getBentLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setJointSpaceKp(runningOnRealRobot ? 40.0 : 150.0);
      gains.setJointSpaceKd(6.0);

      return gains;
   }

   /** {@inheritDoc} */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }
}
