package us.ihmc.valkyrie;

import javax.vecmath.Vector3d;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooAnnotations.QuarantinedTest;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

@BambooPlan(planType = {BambooPlanType.Slow, BambooPlanType.InDevelopment, BambooPlanType.VideoB})
public class ValkyrieObstacleCourseFlatTest extends DRCObstacleCourseFlatTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(false, false);
   
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
   protected Vector3d getFootSlipVector()
   {
      return new Vector3d(0.02, -0.02, 0.0);
   }
   
   @Override
   protected DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {
      return (DoubleYoVariable) scs.getVariable("MomentumBasedControllerFactory.PelvisOrientationManager.RootJointAngularAccelerationControlModule.v1PelvisAxisAngleOrientationController",
                                                "v1PelvisOrientationErrorMagnitude");
   }

   @Override
   protected double getFootSlipTimeDeltaAfterTouchdown()
   {
      return 0.1;
   }
   
   @Ignore
   @QuarantinedTest("Doesn't work with Valkyrie yet. Need to get it working some day")
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees();
   }
   
   
   @Override
	@EstimatedDuration(duration = 111.7)
   @Test(timeout = 558712)
   public void testSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testSimpleFlatGroundScriptWithOscillatingFeet();
   }
   
   
   @Override
	@EstimatedDuration(duration = 34.7)
   @Test(timeout = 173495)
   public void testRotatedStepInTheAir() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testRotatedStepInTheAir();
   }
   
   @Override
	@EstimatedDuration(duration = 18.6)
   @Test(timeout = 93215)
   public void testSimpleFlatGroundScriptWithRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testSimpleFlatGroundScriptWithRandomFootSlip();
   }
   
   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testWalkingUpToRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testWalkingUpToRampWithShortSteps();
   }
   
   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testSideStepsWithSlipping() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testSideStepsWithSlipping();
   }
   
   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly();
   }
   
   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testStandingWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testStandingWithOscillatingFeet();
   }
   
   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testStandingForACoupleSeconds();
   }
   
   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testSideStepsWithRandomSlipping() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testSideStepsWithRandomSlipping();
   }
   
   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testLongStepsMaxHeightPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testLongStepsMaxHeightPauseAndResume();
   }

   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testTurningInPlaceAndPassingPI() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testTurningInPlaceAndPassingPI();
   }
   
   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testChestControlWithPackets() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testChestControlWithPackets();
   }
   
   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testStandingOnUnevenTerrainForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testStandingOnUnevenTerrainForACoupleSeconds();
   }
   
   
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testForMemoryLeaks() throws Exception
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow, BambooPlanType.VideoB);
      super.testForMemoryLeaks();
   }
}
