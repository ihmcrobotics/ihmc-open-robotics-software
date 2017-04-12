package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.HumanoidPointyRocksTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPointyRocksTest extends HumanoidPointyRocksTest
{
   private final DRCRobotModel robotModel = new TestModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);

   @ContinuousIntegrationTest(estimatedDuration = 45.9, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testWalkingForwardWithHalfFootContactChangesStopBetweenSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingForwardWithHalfFootContactChangesStopBetweenSteps();
   }

   @ContinuousIntegrationTest(estimatedDuration = 74.3, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 370000)
   /** {@inheritDoc} */
   public void testStandingWithGCPointsChangingOnTheFly() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      super.testStandingWithGCPointsChangingOnTheFly();
   }

   @ContinuousIntegrationTest(estimatedDuration = 69.2, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 350000)
   /** {@inheritDoc} */
   public void testWalkingForwardWithHalfFootContactChangesContinuousSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingForwardWithHalfFootContactChangesContinuousSteps();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.9, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testWalkingForwardWithPartialFootholdsAndStopBetweenSteps() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingForwardWithPartialFootholdsAndStopBetweenSteps();
   }

   @ContinuousIntegrationTest(estimatedDuration = 92.2, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 460000)
   /** {@inheritDoc} */
   public void testTakingStepsWithActualAndPredictedFootPolygonsChanging() throws SimulationExceededMaximumTimeException
   {
      super.testTakingStepsWithActualAndPredictedFootPolygonsChanging();
   }

   @ContinuousIntegrationTest(estimatedDuration = 36.2, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 180000)
   /** {@inheritDoc} */
   public void testSidePushDuringSwing() throws SimulationExceededMaximumTimeException
   {
      super.testSidePushDuringSwing();
   }

   @ContinuousIntegrationTest(estimatedDuration = 77.5, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 390000)
   /** {@inheritDoc} */
   public void testStandingAndStepsInPlaceWithHalfFootContactsChanges() throws SimulationExceededMaximumTimeException
   {
      super.testStandingAndStepsInPlaceWithHalfFootContactsChanges();
   }

   @ContinuousIntegrationTest(estimatedDuration = 58.3, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 290000)
   /** {@inheritDoc} */
   public void testWalkingWithLinePredictedSupportPolygonButFullActualPolygon() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingWithLinePredictedSupportPolygonButFullActualPolygon();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.9, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testHoldPositionByStandingOnOneLegAndGettingPushedSideways() throws SimulationExceededMaximumTimeException
   {
      super.testHoldPositionByStandingOnOneLegAndGettingPushedSideways();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.0, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 300000)
   /** {@inheritDoc} */
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
   protected DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {

      return (DoubleYoVariable) scs.getVariable("root.atlas.DRCSimulation.DRCControllerThread.DRCMomentumBasedController.HighLevelHumanoidControllerManager.MomentumBasedControllerFactory.WholeBodyControllerCore.WholeBodyFeedbackController.pelvisOrientationFBController.pelvisAxisAngleOrientationController",
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
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return walkingParameters;
      }

   }

   private class TestWalkingParameters extends AtlasWalkingControllerParameters
   {
      private ExplorationParameters explorationParameters = null;

      public TestWalkingParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
      {
         super(target, jointMap, contactPointParameters);
      }

      @Override
      public ExplorationParameters getOrCreateExplorationParameters(YoVariableRegistry registry)
      {
         if (explorationParameters == null)
            explorationParameters = new ExplorationParameters(registry);
         return explorationParameters;
      }

   }

}

