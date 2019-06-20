package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.HumanoidPointyRocksTest;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.variable.YoDouble;

public class AtlasPointyRocksTest extends HumanoidPointyRocksTest
{
   private final DRCRobotModel robotModel = new TestModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   /**
    * Hard test: Atlas walks forward and steps on unknown contacts including lines that need to be explored.
    */
   @Override
   @Disabled
   @Test
   public void testWalkingForwardWithHalfFootContactChangesStopBetweenSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingForwardWithHalfFootContactChangesStopBetweenSteps();
   }

   /**
    * Tests the foothold detection and makes sure the detected area matches the real one.
    */
   @Override
   @Test
   public void testStandingWithGCPointsChangingOnTheFly() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      super.testStandingWithGCPointsChangingOnTheFly();
   }

   /**
    * The robot walks continuously for a few steps with unknown half foot contacts.
    */
   @Override
   @Disabled
   @Test
   public void testWalkingForwardWithHalfFootContactChangesContinuousSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingForwardWithHalfFootContactChangesContinuousSteps();
   }

   /**
    * The robot walks forward with partial footholds. The controller knows about the foothold beforehand.
    */
   @Override
   @Test
   public void testWalkingForwardWithPartialFootholdsAndStopBetweenSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingForwardWithPartialFootholdsAndStopBetweenSteps();
   }

   /**
    * This test steps in place with partial footholds. The controller knows about the foothold beforehand.
    */
   @Override
   @Test
   public void testTakingStepsWithActualAndPredictedFootPolygonsChanging() throws SimulationExceededMaximumTimeException
   {
      super.testTakingStepsWithActualAndPredictedFootPolygonsChanging();
   }

   /**
    * The robot takes a step while on a partial foothold and receives a push that requires the use of angular momentum to recover.
    */
   @Override
   @Test
   public void testSidePushDuringSwing() throws SimulationExceededMaximumTimeException
   {
      super.testSidePushDuringSwing();
   }

   /**
    * In this test, the robot is standing, but then the floor is dropped out from underneath it. So the robot has to detect the rotation
    * and hold position. Then it takes some steps in place with the part of foot changing each step.
    */
   @Override
   @Test
   @Disabled // is a duplicate of other tests with less asserts.
   public void testStandingAndStepsInPlaceWithHalfFootContactsChanges() throws SimulationExceededMaximumTimeException
   {
      super.testStandingAndStepsInPlaceWithHalfFootContactsChanges();
   }

   /**
    * The robot walks thinking it has small footholds but actually has full footholds.
    */
   @Override
   @Test
   @Disabled // does only test stuff that is already covered by other tests in a easier setup
   public void testWalkingWithLinePredictedSupportPolygonButFullActualPolygon() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingWithLinePredictedSupportPolygonButFullActualPolygon();
   }

   @Override
   @Test
   @Disabled // not very interesting test the push does not do much
   public void testHoldPositionByStandingOnOneLegAndGettingPushedSideways() throws SimulationExceededMaximumTimeException
   {
      super.testHoldPositionByStandingOnOneLegAndGettingPushedSideways();
   }

   /**
    * Attempts to stand on a line for a while.
    */
   @Override
   @Disabled
   @Test
   public void testBalanceOnLine() throws SimulationExceededMaximumTimeException
   {
      super.testBalanceOnLine();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   protected YoDouble getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {

      return (YoDouble) scs.getVariable("root.atlas.DRCSimulation.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WholeBodyControllerCore.WholeBodyFeedbackController.pelvisOrientationFBController.pelvisAxisAngleOrientationController",
                                                "pelvisRotationErrorInBodyZ");
   }

   private class TestModel extends AtlasRobotModel
   {
      private final TestWalkingParameters walkingParameters;

      public TestModel(AtlasRobotVersion atlasVersion, RobotTarget target, boolean headless)
      {
         super(atlasVersion, target, headless);
         walkingParameters = new TestWalkingParameters(target, getJointMap(), getContactPointParameters());
      }

      @Override
      public double getSimulateDT()
      { // TODO See if feet can behave better with partial foothold, it causes the joints and and sim to freak out, so need a lower DT.
         return getEstimatorDT() / 10.0;
      }

      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return walkingParameters;
      }
   }

   private class TestWalkingParameters extends AtlasWalkingControllerParameters
   {
      private final TestICPOptimizationParameters icpOptimizationParameters;

      public TestWalkingParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
      {
         super(target, jointMap, contactPointParameters);
         icpOptimizationParameters = new TestICPOptimizationParameters();
      }

      @Override
      public boolean createFootholdExplorationTools()
      {
         return true;
      }

      @Override
      public ICPOptimizationParameters getICPOptimizationParameters()
      {
         return icpOptimizationParameters;
      }
   }

   private class TestICPOptimizationParameters extends AtlasICPOptimizationParameters
   {
      public TestICPOptimizationParameters()
      {
         super(false);
      }

      @Override
      public boolean useAngularMomentum()
      {
         return true;
      }
   }

}

