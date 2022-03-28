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
   public LegConfigurationGains getBentLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setJointSpaceKp(runningOnRealRobot ? 40.0 : 150.0);
      gains.setJointSpaceKd(6.0);

      return gains;
   }
}
