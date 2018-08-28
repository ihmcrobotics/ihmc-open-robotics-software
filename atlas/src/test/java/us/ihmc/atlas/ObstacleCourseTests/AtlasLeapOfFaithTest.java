package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasSteppingParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.AvatarLeapOfFaithTest;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class AtlasLeapOfFaithTest extends AvatarLeapOfFaithTest
{
   private final DRCRobotModel robotModel = new TestModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   /** {@inheritDoc} */
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 90.0)
   @Ignore("Revisit when there are contact patches.")
   @Test(timeout = 230000)
   public void testUnknownStepDownTwoFeetOnEachStep() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.08;
      setStepDownHeight(stepDownHeight);
      super.testUnknownStepDownTwoFeetOnEachStep();
   }

   /** {@inheritDoc} */
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 74.2)
   @Test(timeout = 370000)
   public void testUnknownStepDownOneFootOnEachStep() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.08;
      double stepLength = 0.31;
      double stairLength = 0.35;
      setStepDownHeight(stepDownHeight);
      setStairLength(stairLength);
      setStepLength(stepLength);
      super.testUnknownStepDownOneFootOnEachStep();
   }

   /** {@inheritDoc} */
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 70.4)
   @Test(timeout = 350000)
   public void testUnknownStepDownOneFootOnEachStepLong() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.10;
      double stepLength = 0.37;
      double stairLength = 0.5;
      setStepDownHeight(stepDownHeight);
      setStairLength(stairLength);
      setStepLength(stepLength);
      super.testUnknownStepDownOneFootOnEachStepLong();
   }

   /** {@inheritDoc} */
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 68.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 340000)
   public void testUnknownStepDownOneFootOnEachStepWithUncertainty() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.07;
      double stepLength = 0.345;
      double stairLength = 0.38;
      setStepDownHeight(stepDownHeight);
      setStepLength(stepLength);
      setStairLength(stairLength);
      super.testUnknownStepDownOneFootOnEachStepWithUncertainty();
   }

   /** {@inheritDoc} */
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 110.0)
   @Ignore("Re-enable when planar region constraints are used.")
   @Test(timeout = 230000)
   public void testRandomHeightField() throws SimulationExceededMaximumTimeException
   {
      super.testRandomHeightField();
   }

   /** {@inheritDoc} */
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 107.1)
   @Test(timeout = 540000)
   public void testDropOffsWhileWalking() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.095;
      setStepDownHeight(stepDownHeight);
      super.testDropOffsWhileWalking();
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
      private final TestSteppingParameters steppingParameters;

      public TestWalkingParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
      {
         super(target, jointMap, contactPointParameters);

         leapOfFaithParameters = new TestLeapOfFaithParameters();
         steppingParameters = new TestSteppingParameters(jointMap);
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

      @Override
      public SteppingParameters getSteppingParameters()
      {
         return steppingParameters;
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

   private class TestSteppingParameters extends AtlasSteppingParameters
   {
      public TestSteppingParameters(AtlasJointMap jointMap)
      {
         super(jointMap);
      }

      @Override
      public double getMaxStepLength()
      {
         return 0.8;
      }

      @Override
      public double getMaxStepWidth()
      {
         return getMaxStepLength();
      }
   }
}
