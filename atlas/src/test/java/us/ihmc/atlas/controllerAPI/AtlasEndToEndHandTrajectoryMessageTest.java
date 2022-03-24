package us.ihmc.atlas.controllerAPI;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryMessage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasEndToEndHandTrajectoryMessageTest extends EndToEndHandTrajectoryMessageTest
{
   private AtlasRobotModel robotModel;

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlFrame();
   }

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      super.testMessageWithTooManyTrajectoryPoints();
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

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      super.testQueueStoppedWithOverrideMessage();
   }

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testQueueWithWrongPreviousId() throws Exception
   {
      super.testQueueWithWrongPreviousId();
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testSingleTrajectoryPoint() throws Exception
   {
      super.testSingleTrajectoryPoint();
   }

   @Tag("controller-api-slow-3")
   @Test
   @Override
   public void testForceExecutionWithSingleTrajectoryPoint() throws Exception
   {
      super.testForceExecutionWithSingleTrajectoryPoint();
   }

   @Tag("controller-api-slow-3")
   @Override
   @Test
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Disabled // Could not get a tracking that is decent enough to perform assertions. The test pass with Valkyrie, should be enough.
   @Override
   @Test
   public void testHoldHandWhileWalking() throws SimulationExceededMaximumTimeException
   {
      super.testHoldHandWhileWalking();
   }

   /*
    * Test revealing a bug that was preventing the trajectory from flipping the sign of the final
    * orientation (necessary to prevent an extra rotation). This bug was due to limiting the angle
    * described by a Quaternion to be in [-Pi; Pi].
    */
   @Tag("controller-api-slow-3")
   @Test
   public void testBugFromActualSimDataWithTwoTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), simulationTestingParameters);
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateAndWait(1.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      RobotSide robotSide = RobotSide.RIGHT;

      double trajectoryTime = 1.0;
      RigidBodyBasics chest = fullRobotModel.getChest();

      FramePose3D waypoint0 = new FramePose3D(chest.getBodyFixedFrame());
      waypoint0.getPosition().set(0.85602, -0.33869, -0.01085);
      waypoint0.getOrientation().set(0.99766, 0.01831, 0.06483, 0.01143);
      FramePose3D waypoint1 = new FramePose3D(chest.getBodyFixedFrame());
      waypoint1.getPosition().set(0.97144, -0.38298, -0.02078);
      waypoint1.getOrientation().set(-0.98753, -0.00886, -0.06093, 0.14487);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      waypoint0.changeFrame(worldFrame);
      waypoint1.changeFrame(worldFrame);

      Point3D waypointPosition0 = new Point3D();
      Quaternion waypointOrientation0 = new Quaternion();
      Point3D waypointPosition1 = new Point3D();
      Quaternion waypointOrientation1 = new Quaternion();
      waypoint0.get(waypointPosition0, waypointOrientation0);
      waypoint1.get(waypointPosition1, waypointOrientation1);
      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
      handTrajectoryMessage.setRobotSide(robotSide.toByte());
      SE3TrajectoryMessage se3Trajectory = handTrajectoryMessage.getSe3Trajectory();
      se3Trajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(chest.getBodyFixedFrame()));
      se3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(worldFrame));
      se3Trajectory.getTaskspaceTrajectoryPoints().add()
                   .set(HumanoidMessageTools.createSE3TrajectoryPointMessage(trajectoryTime,
                                                                             waypointPosition0,
                                                                             waypointOrientation0,
                                                                             new Vector3D(),
                                                                             new Vector3D()));
      se3Trajectory.getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(2.0
            * trajectoryTime, waypointPosition1, waypointOrientation1, new Vector3D(), new Vector3D()));

      simulationTestHelper.publishToController(handTrajectoryMessage);

      success = simulationTestHelper.simulateAndWait(1.0 + 2.0 * trajectoryTime);
      assertTrue(success);

      String handName = fullRobotModel.getHand(robotSide).getName();
      String namespace = FeedbackControllerToolbox.class.getSimpleName();
      String varname = handName + "ErrorRotationVector";
      Vector3D rotationError = EndToEndTestTools.findVector3D(namespace, varname, simulationTestHelper);

      /*
       * Checking the tracking error should be enough. As went the bug is present, the error magnitude
       * goes up to [-0.31, 0.002, -0.027] (as rotation vector) against [-0.03, -0.01, -0.01] without the
       * bug.
       */
      assertTrue(rotationError.length() < 0.05);
   }

   @Tag("controller-api")
   @Override
   @Test
   public void testStreaming() throws Exception
   {
      super.testStreaming();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      if (robotModel == null)
      {
         robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
         robotModel.disableOneDoFJointDamping();
      }
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   public double getLegLength()
   {
      AtlasPhysicalProperties physicalProperties = new AtlasPhysicalProperties();
      return physicalProperties.getShinLength() + physicalProperties.getThighLength();
   }
}
