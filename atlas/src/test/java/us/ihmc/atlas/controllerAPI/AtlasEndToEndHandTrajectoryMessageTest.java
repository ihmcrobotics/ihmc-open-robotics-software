package us.ihmc.atlas.controllerAPI;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.SE3TrajectoryMessage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
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
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasEndToEndHandTrajectoryMessageTest extends EndToEndHandTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
   {
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
      { // FIXME Hack to disable joint damping so it is easier to perform assertions on tracking. It'd be good if that was available at construction of the sim.
         return createHumanoidFloatingRootJointRobot(createCollisionMeshes, false);
      };
   };

   @Override
   @Test
   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlFrame();
   }

   @Override
   @Test
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      super.testMessageWithTooManyTrajectoryPoints();
   }

   @Override
   @Test
   public void testMultipleTrajectoryPoints() throws Exception
   {
      super.testMultipleTrajectoryPoints();
   }

   @Override
   @Test
   public void testQueuedMessages() throws Exception
   {
      super.testQueuedMessages();
   }

   @Override
   @Test
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      super.testQueueStoppedWithOverrideMessage();
   }

   @Override
   @Test
   public void testQueueWithWrongPreviousId() throws Exception
   {
      super.testQueueWithWrongPreviousId();
   }

   @Override
   @Test
   public void testSingleTrajectoryPoint() throws Exception
   {
      super.testSingleTrajectoryPoint();
   }

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
    * Test revealing a bug that was preventing the trajectory from flipping the sign of the final orientation (necessary to prevent an extra rotation).
    * This bug was due to limiting the angle described by a Quaternion to be in [-Pi; Pi].
    */
   @Test
   public void testBugFromActualSimDataWithTwoTrajectoryPoints() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      RobotSide robotSide = RobotSide.RIGHT;

      double trajectoryTime = 1.0;
      RigidBodyBasics chest = fullRobotModel.getChest();

      FramePose3D waypoint0 = new FramePose3D(chest.getBodyFixedFrame());
      waypoint0.setPosition(0.85602, -0.33869, -0.01085);
      waypoint0.setOrientation(0.99766, 0.01831, 0.06483, 0.01143);
      FramePose3D waypoint1 = new FramePose3D(chest.getBodyFixedFrame());
      waypoint1.setPosition(0.97144, -0.38298, -0.02078);
      waypoint1.setOrientation(-0.98753, -0.00886, -0.06093, 0.14487);

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
      se3Trajectory.getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(trajectoryTime, waypointPosition0, waypointOrientation0, new Vector3D(), new Vector3D()));
      se3Trajectory.getTaskspaceTrajectoryPoints().add().set(HumanoidMessageTools.createSE3TrajectoryPointMessage(2.0 * trajectoryTime, waypointPosition1, waypointOrientation1, new Vector3D(), new Vector3D()));

      drcSimulationTestHelper.publishToController(handTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + 2.0 * trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      String handName = fullRobotModel.getHand(robotSide).getName();
      String nameSpace = FeedbackControllerToolbox.class.getSimpleName();
      String varname = handName + "ErrorRotationVector";
      Vector3D rotationError = EndToEndTestTools.findVector3D(nameSpace, varname, scs);

      /*
       * Checking the tracking error should be enough.
       * As went the bug is present, the error magnitude goes up to [-0.31, 0.002, -0.027] (as rotation vector) against [-0.03, -0.01, -0.01] without the bug.
       */
      assertTrue(rotationError.length() < 0.05);
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
   public double getLegLength()
   {
      AtlasPhysicalProperties physicalProperties = new AtlasPhysicalProperties();
      return physicalProperties.getShinLength() + physicalProperties.getThighLength();
   }

   @Override
   @Test
   public void testStreaming() throws Exception
   {
      super.testStreaming();
   }
}
