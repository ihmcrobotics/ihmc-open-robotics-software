package us.ihmc.valkyrie;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseTrialsTerrainTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = {TestPlanTarget.Slow, TestPlanTarget.InDevelopment, TestPlanTarget.VideoB})
public class ValkyrieObstacleCourseTrialsTerrainTest extends DRCObstacleCourseTrialsTerrainTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 46.0)
   @Test(timeout = 230000)
   public void testTrialsTerrainCinderblockEntireFieldScript() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment, TestPlanTarget.VideoB);
      super.testTrialsTerrainCinderblockEntireFieldScript();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 79.4)
   @Test(timeout = 400000)
   public void testTrialsTerrainZigzagHurdlesScript() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testTrialsTerrainZigzagHurdlesScript();
   }
   
   /**
    * Need to rerecord
    */
   @Override
   @Ignore
   @DeployableTestMethod(estimatedDuration = 20.0)
   @Test(timeout = 1200000)
   public void testTrialsTerrainUpSlantedCinderblocksScript() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testTrialsTerrainUpSlantedCinderblocksScript();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 147.5)
   @Test(timeout = 740000)
   public void testWalkingOntoAndOverSlopesSideways() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testWalkingOntoAndOverSlopesSideways();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 137.4)
   @Test(timeout = 690000)
   public void testTrialsTerrainSlopeScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testTrialsTerrainSlopeScriptRandomFootSlip();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 32.3)
   @Test(timeout = 160000)
   public void testTrialsTerrainCinderblockFieldPartTwoScript() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment, TestPlanTarget.VideoB);
      super.testTrialsTerrainCinderblockFieldPartTwoScript();
   }
   
   /**
    * Need to rerecord
    */
   @Override
   @Ignore
   @DeployableTestMethod(estimatedDuration = 20.0)
   @Test(timeout=1200000)
   public void testTrialsTerrainUpFlatCinderblocksScript() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testTrialsTerrainUpFlatCinderblocksScript();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 131.3)
   @Test(timeout = 660000)
   public void testTrialsTerrainSlopeScript() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testTrialsTerrainSlopeScript();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 54.0)
   @Test(timeout = 270000)
   public void testTrialsTerrainCinderblockFieldPartOneScript() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment, TestPlanTarget.VideoB);
      super.testTrialsTerrainCinderblockFieldPartOneScript();
   }
   
   @Override
	@DeployableTestMethod(estimatedDuration = 67.7)
   @Test(timeout = 340000)
   public void testTrialsTerrainZigzagHurdlesScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testTrialsTerrainZigzagHurdlesScriptRandomFootSlip();
   }
}
