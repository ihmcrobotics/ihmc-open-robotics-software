package us.ihmc.valkyrie.obstacleCourse;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseTrialsTerrainTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@DeployableTestClass(targets = {TestPlanTarget.Slow, TestPlanTarget.Video})
public class ValkyrieObstacleCourseTrialsTerrainTest extends DRCObstacleCourseTrialsTerrainTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);

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

   @Override
	@DeployableTestMethod(estimatedDuration = 79.4)
   @Test(timeout = 400000)
   public void testTrialsTerrainZigzagHurdlesScript() throws SimulationExceededMaximumTimeException
   {
      super.testTrialsTerrainZigzagHurdlesScript();
   }

   @Override
	@DeployableTestMethod(estimatedDuration = 147.5)
   @Test(timeout = 740000)
   public void testWalkingOntoAndOverSlopesSideways() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingOntoAndOverSlopesSideways();
   }

   @Override
	@DeployableTestMethod(estimatedDuration = 137.4)
   @Test(timeout = 690000)
   public void testTrialsTerrainSlopeScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      super.testTrialsTerrainSlopeScriptRandomFootSlip();
   }

   @Override
	@DeployableTestMethod(estimatedDuration = 131.3)
   @Test(timeout = 660000)
   public void testTrialsTerrainSlopeScript() throws SimulationExceededMaximumTimeException
   {
      super.testTrialsTerrainSlopeScript();
   }

   @Override
	@DeployableTestMethod(estimatedDuration = 67.7)
   @Test(timeout = 340000)
   public void testTrialsTerrainZigzagHurdlesScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      robotModel.addMoreFootContactPointsSimOnly(8, 3, true);
      super.testTrialsTerrainZigzagHurdlesScriptRandomFootSlip();
   }
}
