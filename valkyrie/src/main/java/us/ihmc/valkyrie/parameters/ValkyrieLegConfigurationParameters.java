package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationGains;

public class ValkyrieLegConfigurationParameters extends LegConfigurationParameters
{
   private final RobotTarget target;

   public ValkyrieLegConfigurationParameters(RobotTarget target)
   {
      this.target = target;
   }

   /** {@inheritDoc} */
   @Override
   public LegConfigurationGains getBentLegGains()
   {
      LegConfigurationGains gains = new LegConfigurationGains();
      gains.setJointSpaceKp((target == RobotTarget.REAL_ROBOT) ? 40.0 : 150.0);
      gains.setJointSpaceKd(6.0);

      return gains;
   }
}
