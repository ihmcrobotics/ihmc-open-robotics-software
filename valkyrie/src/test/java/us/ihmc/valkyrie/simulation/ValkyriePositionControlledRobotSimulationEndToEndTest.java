package us.ihmc.valkyrie.simulation;

import us.ihmc.avatar.HumanoidPositionControlledRobotSimulationEndToEndTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyriePositionControlledRobotSimulationEndToEndTest extends HumanoidPositionControlledRobotSimulationEndToEndTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
   }
}
