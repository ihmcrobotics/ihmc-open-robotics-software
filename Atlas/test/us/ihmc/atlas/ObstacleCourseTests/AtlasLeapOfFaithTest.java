package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Test;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.obstacleCourseTests.AvatarLeapOfFaithTest;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasLeapOfFaithTest extends AvatarLeapOfFaithTest
{
   private final DRCRobotModel robotModel = new TestModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);

   @ContinuousIntegrationTest(estimatedDuration = 45.9, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testUnknownStepDownTwoFeetOnEachStep() throws SimulationExceededMaximumTimeException
   {
      super.testUnknownStepDownTwoFeetOnEachStep();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.9, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testUnknownStepDownOneFootOnEachStep() throws SimulationExceededMaximumTimeException
   {
      super.testUnknownStepDownOneFootOnEachStep();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.9, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testUnknownStepDownOneFootOnEachStepLong() throws SimulationExceededMaximumTimeException
   {
      super.testUnknownStepDownOneFootOnEachStepLong();
   }

   @ContinuousIntegrationTest(estimatedDuration = 45.9, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 230000)
   /** {@inheritDoc} */
   public void testUnknownStepDownOneFootOnEachStepWithUncertainty() throws SimulationExceededMaximumTimeException
   {
      super.testUnknownStepDownOneFootOnEachStepWithUncertainty();
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

      public TestModel(AtlasRobotVersion atlasVersion, DRCRobotModel.RobotTarget target, boolean headless)
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
      public TestWalkingParameters(DRCRobotModel.RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
      {
         super(target, jointMap, contactPointParameters);

         leapOfFaithParameters = new TestLeapOfFaithParameters();
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
         return false;
      }
   }

   public static void main(String[] args) throws Exception
   {
      AtlasLeapOfFaithTest test = new AtlasLeapOfFaithTest();
      test.testRandomHeightField();
   }
}
