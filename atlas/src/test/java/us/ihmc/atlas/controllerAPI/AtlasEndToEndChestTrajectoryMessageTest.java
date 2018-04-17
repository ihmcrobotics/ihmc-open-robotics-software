package us.ihmc.atlas.controllerAPI;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndChestTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasEndToEndChestTrajectoryMessageTest extends EndToEndChestTrajectoryMessageTest
{

   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

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
   @ContinuousIntegrationTest(estimatedDuration = 35.8)
   @Test(timeout = 180000)
   public void testLookingLeftAndRight() throws Exception
   {
      super.testLookingLeftAndRight();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 37.1)
   @Test(timeout = 190000)
   public void testLookingLeftAndRightInVariousTrajectoryFrames() throws Exception
   {
      super.testLookingLeftAndRightInVariousTrajectoryFrames();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 34.9)
   @Test(timeout = 170000)
   public void testMessageWithALotOfTrajectoryPoints() throws Exception
   {
      super.testMessageWithALotOfTrajectoryPoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 35.4)
   @Test(timeout = 180000)
   public void testMessageWithALotOfTrajectoryPointsExpressedInPelvisZUp() throws Exception
   {
      super.testMessageWithALotOfTrajectoryPointsExpressedInPelvisZUp();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 24.5)
   @Test(timeout = 120000)
   public void testMultipleTrajectoryPoints() throws Exception
   {
      super.testMultipleTrajectoryPoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 43.2)
   @Test(timeout = 220000)
   public void testQueuedMessages() throws Exception
   {
      super.testQueuedMessages();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 26.4)
   @Test(timeout = 130000)
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      super.testQueueStoppedWithOverrideMessage();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 41.2)
   @Test(timeout = 210000)
   public void testQueueWithUsingDifferentTrajectoryFrameWithoutOverride() throws Exception
   {
      super.testQueueWithUsingDifferentTrajectoryFrameWithoutOverride();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 17.3)
   @Test(timeout = 87000)
   public void testQueueWithWrongPreviousId() throws Exception
   {
      super.testQueueWithWrongPreviousId();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.5)
   @Test(timeout = 130000)
   public void testSelectionMatrixDisableRandomAxisWithSingleTrajectoryPoint() throws Exception
   {
      super.testSelectionMatrixDisableRandomAxisWithSingleTrajectoryPoint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 26.7)
   @Test(timeout = 130000)
   public void testSelectionMatrixWithAllAxisOffUsingSingleTrajectoryPoint() throws Exception
   {
      super.testSelectionMatrixWithAllAxisOffUsingSingleTrajectoryPoint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 23.8)
   @Test(timeout = 120000)
   public void testSettingWeightMatrixUsingSingleTrajectoryPoint() throws Exception
   {
      super.testSettingWeightMatrixUsingSingleTrajectoryPoint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 27.3)
   @Test(timeout = 140000)
   public void testSingleTrajectoryPoint() throws Exception
   {
      super.testSingleTrajectoryPoint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 34.2)
   @Test(timeout = 170000)
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }
}
