package us.ihmc.openAlexander.controllerAPI;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndChestTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.Axis3D;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class OpenAlexanderEndToEndChestTrajectoryMessageTest extends EndToEndChestTrajectoryMessageTest
{

   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ALEXANDER);
   }

   @Override
   public Axis3D getAxisToMoveForQueuedMessages()
   {
      return Axis3D.Z;
   }

   @Override
   public Axis3D getAxisToMoveForALotOfMessages()
   {
      return Axis3D.Z;
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testLookingLeftAndRight() throws Exception
   {
      super.testLookingLeftAndRight();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testLookingLeftAndRightInVariousTrajectoryFrames() throws Exception
   {
      super.testLookingLeftAndRightInVariousTrajectoryFrames();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testMessageWithALotOfTrajectoryPoints() throws Exception
   {
      super.testMessageWithALotOfTrajectoryPoints();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testMessageWithALotOfTrajectoryPointsExpressedInPelvisZUp() throws Exception
   {
      super.testMessageWithALotOfTrajectoryPointsExpressedInPelvisZUp();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testMultipleTrajectoryPoints() throws Exception
   {
      super.testMultipleTrajectoryPoints();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testQueuedMessages() throws Exception
   {
      super.testQueuedMessages();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      super.testQueueStoppedWithOverrideMessage();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testQueueWithUsingDifferentTrajectoryFrameWithoutOverride() throws Exception
   {
      super.testQueueWithUsingDifferentTrajectoryFrameWithoutOverride();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testQueueWithWrongPreviousId() throws Exception
   {
      super.testQueueWithWrongPreviousId();
   }

   // This doesn't work for Alexander, since we only have a yaw joint.
   @Tag("controller-api-slow")
   @Disabled
   @Override
   @Test
   public void testSelectionMatrixDisableRandomAxisWithSingleTrajectoryPoint() throws Exception
   {
      super.testSelectionMatrixDisableRandomAxisWithSingleTrajectoryPoint();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testSelectionMatrixWithAllAxisOffUsingSingleTrajectoryPoint() throws Exception
   {
      super.testSelectionMatrixWithAllAxisOffUsingSingleTrajectoryPoint();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testSettingWeightMatrixUsingSingleTrajectoryPoint() throws Exception
   {
      super.testSettingWeightMatrixUsingSingleTrajectoryPoint();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testSingleTrajectoryPoint() throws Exception
   {
      super.testSingleTrajectoryPoint();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testStreaming() throws Exception
   {
      super.testStreaming();
   }

   @Tag("controller-api-slow")
   @Override
   @Test
   public void testStopAllTrajectoryRepeatedly() throws Exception
   {
      super.testStopAllTrajectoryRepeatedly();
   }
}
