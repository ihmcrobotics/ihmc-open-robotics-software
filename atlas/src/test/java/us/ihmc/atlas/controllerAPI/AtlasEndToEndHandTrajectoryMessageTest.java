package us.ihmc.atlas.controllerAPI;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.avatar.controllerAPI.EndToEndHandTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasEndToEndHandTrajectoryMessageTest extends EndToEndHandTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 60000)
   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlFrame();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 60000)
   public void testMessageWithTooManyTrajectoryPoints() throws Exception
   {
      super.testMessageWithTooManyTrajectoryPoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 60000)
   public void testMultipleTrajectoryPoints() throws Exception
   {
      super.testMultipleTrajectoryPoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 40.0)
   @Test(timeout = 60000)
   public void testQueuedMessages() throws Exception
   {
      super.testQueuedMessages();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 60000)
   public void testQueueStoppedWithOverrideMessage() throws Exception
   {
      super.testQueueStoppedWithOverrideMessage();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 60000)
   public void testQueueWithWrongPreviousId() throws Exception
   {
      super.testQueueWithWrongPreviousId();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 60000)
   public void testSingleTrajectoryPoint() throws Exception
   {
      super.testSingleTrajectoryPoint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 60000)
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   /*
    * Test revealing a bug that was preventing the trajectory from flipping the sign of the final orientation (necessary to prevent an extra rotation).
    * This bug was due to limiting the angle described by a Quaternion to be in [-Pi; Pi].
    */
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 60000)
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
      RigidBody chest = fullRobotModel.getChest();

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
      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide, 2);
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setTrajectoryReferenceFrame(chest.getBodyFixedFrame());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrame(worldFrame);
      handTrajectoryMessage.getSe3Trajectory().setTrajectoryPoint(0, trajectoryTime, waypointPosition0, waypointOrientation0, new Vector3D(), new Vector3D(), worldFrame);
      handTrajectoryMessage.getSe3Trajectory().setTrajectoryPoint(1, 2.0 * trajectoryTime, waypointPosition1, waypointOrientation1, new Vector3D(), new Vector3D(), worldFrame);

      drcSimulationTestHelper.send(handTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + 2.0 * trajectoryTime);
      assertTrue(success);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      String handName = fullRobotModel.getHand(robotSide).getName();
      String nameSpace = FeedbackControllerToolbox.class.getSimpleName();
      String varname = handName + "ErrorRotationVector";
      Vector3D rotationError = findVector3d(nameSpace, varname, scs);

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

}
