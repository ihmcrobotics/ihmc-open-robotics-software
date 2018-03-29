package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Ignore;
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
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasLeapOfFaithTest extends AvatarLeapOfFaithTest
{
   private final DRCRobotModel robotModel = new TestModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   /** {@inheritDoc} */
   @ContinuousIntegrationTest(estimatedDuration = 90.0)
   @Ignore("Revisit when there are contact patches.")
   @Test(timeout = 230000)
   public void testUnknownStepDownTwoFeetOnEachStep() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.08;
      super.testUnknownStepDownTwoFeetOnEachStep(stepDownHeight);
   }

   /** {@inheritDoc} */
   @ContinuousIntegrationTest(estimatedDuration = 90.0)
   @Test(timeout = 230000)
   public void testUnknownStepDownOneFootOnEachStep() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.08;
      double stepLength = 0.31;
      double stairLength = 0.35;
      super.testUnknownStepDownOneFootOnEachStep(stepLength, stairLength, stepDownHeight);
   }

   /** {@inheritDoc} */
   @ContinuousIntegrationTest(estimatedDuration = 90.0)
   @Test(timeout = 230000)
   public void testUnknownStepDownOneFootOnEachStepLong() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.10;
      double stepLength = 0.37;
      double stairLength = 0.5;
      super.testUnknownStepDownOneFootOnEachStepLong(stepLength, stairLength, stepDownHeight);
   }

   /** {@inheritDoc} */
   @ContinuousIntegrationTest(estimatedDuration = 90.0)
   @Test(timeout = 230000)
   public void testUnknownStepDownOneFootOnEachStepWithUncertainty() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.07;
      super.testUnknownStepDownOneFootOnEachStepWithUncertainty(stepDownHeight);
   }

   /** {@inheritDoc} */
   @ContinuousIntegrationTest(estimatedDuration = 110.0)
   @Ignore("Re-enable when planar region constraints are used.")
   @Test(timeout = 230000)
   public void testRandomHeightField() throws SimulationExceededMaximumTimeException
   {
      double maxStepIncrease = 0.07;
      double maxStepHeight = 0.04;
      double minStepHeight = -0.10;
      super.testRandomHeightField(maxStepHeight, minStepHeight, maxStepIncrease);
   }

   /** {@inheritDoc} */
   @ContinuousIntegrationTest(estimatedDuration = 90.0)
   @Test(timeout = 230000)
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
}
