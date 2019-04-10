package us.ihmc.valkyrie.simulation;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.DRCFlatGroundWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

//This test is slow but very important, let's keep it in the FAST build please. (Sylvain)
public class ValkyrieFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{
   private DRCRobotModel robotModel;

   @Override
   public boolean doPelvisWarmup()
   {
      return true;
   }

   @Override
	@Test
   public void testFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
      super.testFlatGroundWalking();
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
