package us.ihmc.valkyrie.simulation;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.HumanoidPositionControlledRobotSimulationEndToEndTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyriePositionControlledRobotSimulationEndToEndTest extends HumanoidPositionControlledRobotSimulationEndToEndTest
{
   private ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   protected HighLevelControllerParameters getPositionControlParameters(HighLevelControllerName positionControlState)
   {
      return new ValkyrieSimulationPositionControlParameters(robotModel.getHighLevelControllerParameters(), robotModel.getJointMap(), positionControlState);
   }

   @Test
   @Override
   public void testFreezeController(TestInfo testInfo) throws Exception
   {
      super.testFreezeController(testInfo);
   }

   @Test
   @Override
   public void testPositionController(TestInfo testInfo) throws Exception
   {
      super.testPositionController(testInfo);
   }
}
