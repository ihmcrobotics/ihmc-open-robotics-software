package us.ihmc.acsell;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.steppr.parameters.BonoRobotModel;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.TestPlanTarget;

@ContinuousIntegrationPlan(targets = TestPlanTarget.Flaky)
public class BonoFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{
   private BonoRobotModel robotModel;

	@ContinuousIntegrationTest(estimatedDuration = 128.2)
	@Test(timeout = 640000)
   public void testBONOFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      robotModel = new BonoRobotModel(false, false);
      super.testFlatGroundWalking(robotModel, false);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.BONO);
   }
}
