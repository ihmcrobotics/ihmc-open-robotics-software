package us.ihmc.valkyrie;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseTrialsTerrainTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooAnnotations.QuarantinedTest;
import us.ihmc.tools.agileTesting.BambooPlanType;

@BambooPlan(planType = {BambooPlanType.Slow, BambooPlanType.InDevelopment, BambooPlanType.VideoB})
public class ValkyrieObstacleCourseTrialsTerrainTest extends DRCObstacleCourseTrialsTerrainTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(false, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }
   
   @Override
   @EstimatedDuration(duration = 90.6)
   @Test(timeout = 1800000)
   public void testTrialsTerrainCinderblockEntireFieldScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment, BambooPlanType.VideoB);
      super.testTrialsTerrainCinderblockEntireFieldScript();
   }
   
   @Override
   @EstimatedDuration(duration = 23.3)
   @Test(timeout = 1200000)
   public void testTrialsTerrainZigzagHurdlesScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainZigzagHurdlesScript();
   }
   
   @Override
   @Ignore
   @QuarantinedTest("Need to rerecord")
   @EstimatedDuration(duration = 20.0)
   @Test(timeout = 1200000)
   public void testTrialsTerrainUpSlantedCinderblocksScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainUpSlantedCinderblocksScript();
   }
   
   @Override
   @EstimatedDuration(duration = 43.4)
   @Test(timeout = 1200000)
   public void testWalkingOntoAndOverSlopesSideways() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testWalkingOntoAndOverSlopesSideways();
   }
   
   @Override
   @EstimatedDuration(duration = 45.8)
   @Test(timeout = 1200000)
   public void testTrialsTerrainSlopeScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainSlopeScriptRandomFootSlip();
   }
   
   @Override
   @EstimatedDuration(duration = 83.9)
   @Test(timeout = 1200000)
   public void testTrialsTerrainCinderblockFieldPartTwoScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment, BambooPlanType.VideoB);
      super.testTrialsTerrainCinderblockFieldPartTwoScript();
   }
   
   @Override
   @Ignore
   @QuarantinedTest("Need to rerecord")
   @EstimatedDuration(duration = 20.0)
   @Test(timeout=1200000)
   public void testTrialsTerrainUpFlatCinderblocksScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainUpFlatCinderblocksScript();
   }
   
   @Override
   @EstimatedDuration(duration = 44.2)
   @Test(timeout = 1200000)
   public void testTrialsTerrainSlopeScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainSlopeScript();
   }
   
   @Override
   @EstimatedDuration(duration = 64.7)
   @Test(timeout = 1200000)
   public void testTrialsTerrainCinderblockFieldPartOneScript() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment, BambooPlanType.VideoB);
      super.testTrialsTerrainCinderblockFieldPartOneScript();
   }
   
   @Override
   @EstimatedDuration(duration = 30.0)
   @Test(timeout = 1200000)
   public void testTrialsTerrainZigzagHurdlesScriptRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testTrialsTerrainZigzagHurdlesScriptRandomFootSlip();
   }
}
