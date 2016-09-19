package us.ihmc.valkyrie.simulation;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.valkyrie.ValkyrieRobotModel;

//This test is slow but very important, let's keep it in the FAST build please. (Sylvain)
@DeployableTestClass(targets = {TestPlanTarget.Fast, TestPlanTarget.Video})
public class ValkyrieFlatGroundWalkingTest extends DRCFlatGroundWalkingTest
{
   private DRCRobotModel robotModel;

	@DeployableTestMethod(estimatedDuration = 273.1)
	@Test(timeout = 1400000)
   public void testValkyrieFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
      super.testFlatGroundWalking(robotModel, true);
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
