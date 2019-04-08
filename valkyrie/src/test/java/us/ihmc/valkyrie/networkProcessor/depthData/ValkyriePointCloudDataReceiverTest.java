package us.ihmc.valkyrie.networkProcessor.depthData;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.depthData.HumanoidPointCloudDataReceiverTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Disabled
public class ValkyriePointCloudDataReceiverTest extends HumanoidPointCloudDataReceiverTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   @Test
   public void testIsReceivingScansAnd95PercentOfPointsAreCorrect() throws SimulationExceededMaximumTimeException
   {
      super.testIsReceivingScansAnd95PercentOfPointsAreCorrect();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }
}
