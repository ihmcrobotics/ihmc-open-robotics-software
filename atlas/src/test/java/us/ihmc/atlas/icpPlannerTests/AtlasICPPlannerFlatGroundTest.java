package us.ihmc.atlas.icpPlannerTests;

import org.junit.Test;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.icpPlannerTests.AvatarICPPlannerFlatGroundTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasICPPlannerFlatGroundTest extends AvatarICPPlannerFlatGroundTest
{
   private final DRCRobotModel robotModel = new TestModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);


   @Override
   @ContinuousIntegrationTest(estimatedDuration = 74.3, categoriesOverride = {IntegrationCategory.IN_DEVELOPMENT})
   @Test(timeout = 370000)
   /** {@inheritDoc} */
   public void testChangeOfSupport() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      super.testChangeOfSupport();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 64.8, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 320000)
   /** {@inheritDoc} */
   public void testPauseWalkingInSwing() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      super.testPauseWalkingInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 69.9, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 350000)
   /** {@inheritDoc} */
   public void testPauseWalkingInTransferFirstStep() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      super.testPauseWalkingInTransferFirstStep();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 68.1, categoriesOverride = {IntegrationCategory.FAST})
   @Test(timeout = 340000)
   /** {@inheritDoc} */
   public void testPauseWalkingInTransfer() throws SimulationExceededMaximumTimeException, RuntimeException
   {
      super.testPauseWalkingInTransfer();
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
      public TestWalkingParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
      {
         super(target, jointMap, contactPointParameters);
      }

      @Override
      public boolean createFootholdExplorationTools()
      {
         return true;
      }
   }

}

