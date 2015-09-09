package us.ihmc.valkyrie;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseTrialsTerrainTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(planType = {BambooPlanType.Slow, BambooPlanType.InDevelopment, BambooPlanType.VideoB})
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
	@DeployableTestMethod(duration = 46.0)
   @Test(timeout = 230000)
   public void testTrialsTerrainCinderblockEntireFieldScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment, BambooPlanType.VideoB);
      super.testTrialsTerrainCinderblockEntireFieldScript();
   }
   
   @Override
	@DeployableTestMethod(duration = 79.4)
   @Test(timeout = 400000)
   public void testTrialsTerrainZigzagHurdlesScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainZigzagHurdlesScript();
   }
   
   /**
    * Need to rerecord
    */
   @Override
   @Ignore
   @DeployableTestMethod(duration = 20.0, quarantined = true)
   @Test(timeout = 1200000)
   public void testTrialsTerrainUpSlantedCinderblocksScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainUpSlantedCinderblocksScript();
   }
   
   @Override
	@DeployableTestMethod(duration = 147.5)
   @Test(timeout = 740000)
   public void testWalkingOntoAndOverSlopesSideways() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testWalkingOntoAndOverSlopesSideways();
   }
   
   @Override
	@DeployableTestMethod(duration = 137.4)
   @Test(timeout = 690000)
   public void testTrialsTerrainSlopeScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainSlopeScriptRandomFootSlip();
   }
   
   @Override
	@DeployableTestMethod(duration = 32.3)
   @Test(timeout = 160000)
   public void testTrialsTerrainCinderblockFieldPartTwoScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment, BambooPlanType.VideoB);
      super.testTrialsTerrainCinderblockFieldPartTwoScript();
   }
   
   /**
    * Need to rerecord
    */
   @Override
   @Ignore
   @DeployableTestMethod(duration = 20.0, quarantined = true)
   @Test(timeout=1200000)
   public void testTrialsTerrainUpFlatCinderblocksScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainUpFlatCinderblocksScript();
   }
   
   @Override
	@DeployableTestMethod(duration = 131.3)
   @Test(timeout = 660000)
   public void testTrialsTerrainSlopeScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainSlopeScript();
   }
   
   @Override
	@DeployableTestMethod(duration = 54.0)
   @Test(timeout = 270000)
   public void testTrialsTerrainCinderblockFieldPartOneScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment, BambooPlanType.VideoB);
      super.testTrialsTerrainCinderblockFieldPartOneScript();
   }
   
   @Override
	@DeployableTestMethod(duration = 67.7)
   @Test(timeout = 340000)
   public void testTrialsTerrainZigzagHurdlesScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainZigzagHurdlesScriptRandomFootSlip();
   }
}
