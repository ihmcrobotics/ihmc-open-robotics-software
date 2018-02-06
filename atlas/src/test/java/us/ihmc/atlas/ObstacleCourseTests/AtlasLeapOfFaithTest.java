package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.AvatarLeapOfFaithTest;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasLeapOfFaithTest extends AvatarLeapOfFaithTest
{
   private final DRCRobotModel robotModel = new TestModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @ContinuousIntegrationTest(estimatedDuration = 90.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testUnknownStepDownTwoFeetOnEachStep() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.08;
      super.testUnknownStepDownTwoFeetOnEachStep(stepDownHeight);
   }

   @ContinuousIntegrationTest(estimatedDuration = 90.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testUnknownStepDownOneFootOnEachStep() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.08;
      super.testUnknownStepDownOneFootOnEachStep(stepDownHeight);
   }

   @ContinuousIntegrationTest(estimatedDuration = 90.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testUnknownStepDownOneFootOnEachStepLong() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.10;
      super.testUnknownStepDownOneFootOnEachStepLong(stepDownHeight);
   }

   @ContinuousIntegrationTest(estimatedDuration = 90.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testUnknownStepDownOneFootOnEachStepWithUncertainty() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.07;
      super.testUnknownStepDownOneFootOnEachStepWithUncertainty(stepDownHeight);
   }

   @ContinuousIntegrationTest(estimatedDuration = 110.0, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testRandomHeightField() throws SimulationExceededMaximumTimeException
   {
      double maxStepIncrease = 0.07;
      double maxStepHeight = 0.04;
      double minStepHeight = -0.10;
      super.testRandomHeightField(maxStepHeight, minStepHeight, maxStepIncrease);
   }

   @ContinuousIntegrationTest(estimatedDuration = 90.0, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testDropOffsWhileWalking() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.10;
      super.testDropOffsWhileWalking(stepDownHeight);
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
      private final TestLeapOfFaithParameters leapOfFaithParameters;
      public TestWalkingParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
      {
         super(target, jointMap, contactPointParameters);

         leapOfFaithParameters = new TestLeapOfFaithParameters();
      }

      @Override
      public double nominalHeightAboveAnkle()
      {
         return 0.74;
      }

      @Override
      public double maximumHeightAboveAnkle()
      {
         return 0.85;
      }

      @Override
      public LeapOfFaithParameters getLeapOfFaithParameters()
      {
         return leapOfFaithParameters;
      }
   }

   private class TestLeapOfFaithParameters extends LeapOfFaithParameters
   {
      public TestLeapOfFaithParameters()
      {
         super();
      }

      @Override
      public boolean scaleFootWeight()
      {
         return true;
      }

      @Override
      public boolean usePelvisRotation()
      {
         return true;
      }

      @Override
      public boolean relaxPelvisControl()
      {
         return true;
      }

   }

   public static void main(String[] args) throws Exception
   {
      AtlasLeapOfFaithTest test = new AtlasLeapOfFaithTest();
      test.testUnknownStepDownOneFootOnEachStep();
   }
}
