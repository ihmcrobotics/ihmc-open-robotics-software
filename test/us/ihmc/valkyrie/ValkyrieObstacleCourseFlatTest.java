package us.ihmc.valkyrie;

import javax.vecmath.Vector3d;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = {TestPlanTarget.Slow, TestPlanTarget.InDevelopment, TestPlanTarget.VideoB})
public class ValkyrieObstacleCourseFlatTest extends DRCObstacleCourseFlatTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(DRCRobotModel.RobotTarget.SCS, false);

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
      return (DoubleYoVariable) scs.getVariable(
          "MomentumBasedControllerFactory.PelvisOrientationManager.RootJointAngularAccelerationControlModule.v1PelvisAxisAngleOrientationController",
          "v1PelvisOrientationErrorMagnitude");
   }

   @Override
   protected double getFootSlipTimeDeltaAfterTouchdown()
   {
      return 0.1;
   }

   /**
    * Doesn't work with Valkyrie yet. Need to get it working some day.
    */
   @Ignore
   @Override
   @DeployableTestMethod
   @Test(timeout = 300000)
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 105.9)
   @Test(timeout = 530000)
   public void testSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testSimpleFlatGroundScriptWithOscillatingFeet();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 41.4)
   @Test(timeout = 210000)
   public void testRotatedStepInTheAir() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment, TestPlanTarget.VideoB);
      super.testRotatedStepInTheAir();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 92.6)
   @Test(timeout = 460000)
   public void testSimpleFlatGroundScriptWithRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testSimpleFlatGroundScriptWithRandomFootSlip();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 60.5)
   @Test(timeout = 300000)
   public void testWalkingUpToRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testWalkingUpToRampWithShortSteps();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 55.3)
   @Test(timeout = 280000)
   public void testSideStepsWithSlipping() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testSideStepsWithSlipping();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 25.6)
   @Test(timeout = 130000)
   public void testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 32.3)
   @Test(timeout = 160000)
   public void testStandingWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testStandingWithOscillatingFeet();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 22.3)
   @Test(timeout = 110000)
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testStandingForACoupleSeconds();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 57.6)
   @Test(timeout = 290000)
   public void testSideStepsWithRandomSlipping() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testSideStepsWithRandomSlipping();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 76.2)
   @Test(timeout = 380000)
   public void testLongStepsMaxHeightPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testLongStepsMaxHeightPauseAndResume();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 26.5)
   @Test(timeout = 130000)
   public void testTurningInPlaceAndPassingPI() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testTurningInPlaceAndPassingPI();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 50.0)
   @Test(timeout = 250000)
   public void testChestControlWithPackets() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testChestControlWithPackets();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 43.7)
   @Test(timeout = 220000)
   public void testStandingOnUnevenTerrainForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testStandingOnUnevenTerrainForACoupleSeconds();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 209.5)
   @Test(timeout = 1000000)
   public void testForMemoryLeaks() throws Exception
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow, TestPlanTarget.VideoB);
      super.testForMemoryLeaks();
   }
}
