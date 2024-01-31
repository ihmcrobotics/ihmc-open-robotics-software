package us.ihmc.avatar.behaviorTests;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.avatar.testTools.scs2.SCS2BehaviorTestHelper;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WholeBodyInverseKinematicsBehavior;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class WholeBodyInverseKinematicsBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private boolean isKinematicsToolboxVisualizerEnabled = false;
   private SCS2BehaviorTestHelper behaviorTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (behaviorTestHelper != null)
      {
         behaviorTestHelper.finishTest();
         behaviorTestHelper = null;
      }

      if (kinematicsToolboxModule != null)
      {
         kinematicsToolboxModule.destroy();
         kinematicsToolboxModule = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @BeforeEach
   public void setUp() throws IOException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      CommonAvatarEnvironmentInterface envrionment = new FlatGroundEnvironment();

      SCS2AvatarTestingSimulation simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(),
                                                                                                                        envrionment,
                                                                                                                        simulationTestingParameters);
      simulationTestHelper.start();
      behaviorTestHelper = new SCS2BehaviorTestHelper(simulationTestHelper);

      setupKinematicsToolboxModule();
   }

   @Test
   public void testSolvingForAHandPose()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // simulate for a while to make sure the robot is still so small time differences between frame changes in the
      // controller and the unit test will not affect the outcome too much.
      boolean success = behaviorTestHelper.simulateNow(3.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.RIGHT;

      behaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(behaviorTestHelper.getRobotName(),
                                                                                     getRobotModel(),
                                                                                     behaviorTestHelper.getYoTime(),
                                                                                     behaviorTestHelper.getROS2Node(),
                                                                                     behaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrame = behaviorTestHelper.getReferenceFrames().getHandFrame(robotSide);

      FullHumanoidRobotModel fullRobotModel = behaviorTestHelper.getControllerFullRobotModel();
      RigidBodyBasics chest = fullRobotModel.getChest();
      ReferenceFrame chestControlFrame = chest.getBodyFixedFrame();
      FrameQuaternion initialChestOrientation = new FrameQuaternion(chestControlFrame);
      initialChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      ReferenceFrame pelvisControlFrame = fullRobotModel.getPelvis().getBodyFixedFrame();
      FrameQuaternion initialPelvisOrientation = new FrameQuaternion(pelvisControlFrame);
      initialPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D desiredHandPose = new FramePose3D(handControlFrame);
      desiredHandPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPose.prependTranslation(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setDesiredHandPose(robotSide, desiredHandPose);
      ik.holdCurrentChestOrientation();
      ik.holdCurrentPelvisOrientation();
      ik.holdCurrentPelvisHeight();

      behaviorTestHelper.updateRobotModel();
      FramePose3D desiredHandPoseCopy = new FramePose3D(desiredHandPose);
      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      desiredHandPoseCopy.changeFrame(chestFrame);

      behaviorTestHelper.dispatchBehavior(ik);

      double totalSimDuration = 0.0;
      while (!ik.isDone())
      {
         double simDuration = 0.1;
         success = behaviorTestHelper.simulateNow(simDuration);
         assertTrue(success);

         totalSimDuration += simDuration;

         if (totalSimDuration >= 10)
            fail("IK is not converging");
      }

      assertFalse(ik.hasSolverFailed(), "Bad solution: " + ik.getSolutionQuality());

      success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      String pelvisName = fullRobotModel.getPelvis().getName();
      QuaternionReadOnly controllerDesiredChestOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(chest.getName(), behaviorTestHelper);
      QuaternionReadOnly controllerDesiredPelvisOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(pelvisName, behaviorTestHelper);

      double angleEpsilon = Math.toRadians(2.0);

      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(initialChestOrientation, controllerDesiredChestOrientation, angleEpsilon);
      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(initialPelvisOrientation, controllerDesiredPelvisOrientation, angleEpsilon);

      String handName = fullRobotModel.getHand(robotSide).getName();
      Point3DReadOnly controllerDesiredHandPosition = EndToEndTestTools.findFeedbackControllerDesiredPosition(handName, behaviorTestHelper);

      Point3D handPosition = new Point3D(desiredHandPose.getPosition());

      double positionEpsilon = 1.0e-4;
      double positionDifference = handPosition.distance(controllerDesiredHandPosition);

      assertTrue(positionDifference < positionEpsilon, "Position difference: " + positionDifference);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForBothHandPoses()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // simulate for a while to make sure the robot is still so small time differences between frame changes in the
      // controller and the unit test will not affect the outcome too much.
      boolean success = behaviorTestHelper.simulateNow(3.0);
      assertTrue(success);

      behaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(behaviorTestHelper.getRobotName(),
                                                                                     getRobotModel(),
                                                                                     behaviorTestHelper.getYoTime(),
                                                                                     behaviorTestHelper.getROS2Node(),
                                                                                     behaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrameR = behaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.RIGHT);
      ReferenceFrame handControlFrameL = behaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.LEFT);

      FramePose3D desiredHandPoseR = new FramePose3D(handControlFrameR);
      desiredHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPoseR.prependTranslation(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setDesiredHandPose(RobotSide.RIGHT, desiredHandPoseR);
      FramePose3D desiredHandPoseL = new FramePose3D(handControlFrameL);
      desiredHandPoseL.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPoseL.prependTranslation(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setDesiredHandPose(RobotSide.LEFT, desiredHandPoseL);
      ik.holdCurrentChestOrientation();
      ik.holdCurrentPelvisOrientation();

      behaviorTestHelper.updateRobotModel();
      FramePose3D desiredHandPoseLCopy = new FramePose3D(desiredHandPoseL);
      FramePose3D desiredHandPoseRCopy = new FramePose3D(desiredHandPoseR);
      ReferenceFrame chestFrame = behaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      desiredHandPoseLCopy.changeFrame(chestFrame);
      desiredHandPoseRCopy.changeFrame(chestFrame);

      behaviorTestHelper.dispatchBehavior(ik);

      double totalSimDuration = 0.0;
      while (!ik.isDone())
      {
         double simDuration = 0.1;
         success = behaviorTestHelper.simulateNow(simDuration);
         assertTrue(success);

         totalSimDuration += simDuration;

         if (totalSimDuration >= 10)
            fail("IK is not converging");
      }

      assertFalse(ik.hasSolverFailed(), "Bad solution: " + ik.getSolutionQuality());

      success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      String rightHandName = behaviorTestHelper.getControllerFullRobotModel().getHand(RobotSide.RIGHT).getName();
      String leftHandName = behaviorTestHelper.getControllerFullRobotModel().getHand(RobotSide.LEFT).getName();

      QuaternionReadOnly controllerDesiredHandOrientationR = EndToEndTestTools.findFeedbackControllerDesiredOrientation(rightHandName, behaviorTestHelper);
      Quaternion desiredHandOrientationR = new Quaternion(desiredHandPoseR.getOrientation());
      QuaternionReadOnly controllerDesiredHandOrientationL = EndToEndTestTools.findFeedbackControllerDesiredOrientation(leftHandName, behaviorTestHelper);
      Quaternion desiredHandOrientationL = new Quaternion(desiredHandPoseL.getOrientation());

      double handAngleEpsilon = Math.toRadians(1.0);

      assertTrue(isOrientationEqual(desiredHandOrientationR, controllerDesiredHandOrientationR, handAngleEpsilon));
      assertTrue(isOrientationEqual(desiredHandOrientationL, controllerDesiredHandOrientationL, handAngleEpsilon));

      Point3DReadOnly controllerDesiredHandPositionR = EndToEndTestTools.findFeedbackControllerDesiredPosition(rightHandName, behaviorTestHelper);
      Point3DReadOnly controllerDesiredHandPositionL = EndToEndTestTools.findFeedbackControllerDesiredPosition(leftHandName, behaviorTestHelper);
      Point3D rightPosition = new Point3D(desiredHandPoseR.getPosition());
      Point3D leftPosition = new Point3D(desiredHandPoseL.getPosition());
      double rightDifference = rightPosition.distance(controllerDesiredHandPositionR);
      double leftDifference = leftPosition.distance(controllerDesiredHandPositionL);

      double positionEpsilon = 1.0e-4;

      assertTrue(rightDifference < positionEpsilon, "Position difference: " + rightDifference);
      assertTrue(leftDifference < positionEpsilon, "Position difference: " + leftDifference);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForHandSelectionMatrix()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // simulate for a while to make sure the robot is still so small time differences between frame changes in the
      // controller and the unit test will not affect the outcome too much.
      boolean success = behaviorTestHelper.simulateNow(3.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.RIGHT;

      behaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(behaviorTestHelper.getRobotName(),
                                                                                     getRobotModel(),
                                                                                     behaviorTestHelper.getYoTime(),
                                                                                     behaviorTestHelper.getROS2Node(),
                                                                                     behaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrame = behaviorTestHelper.getReferenceFrames().getHandFrame(robotSide);

      RigidBodyBasics chest = behaviorTestHelper.getControllerFullRobotModel().getChest();
      ReferenceFrame chestControlFrame = chest.getBodyFixedFrame();
      FrameQuaternion initialChestOrientation = new FrameQuaternion(chestControlFrame);
      initialChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion offsetOrientation = new Quaternion();
      offsetOrientation.setYawPitchRoll(0.0, 0.0, 0.1);
      FramePose3D desiredHandPose = new FramePose3D(handControlFrame);
      desiredHandPose.getOrientation().set(offsetOrientation);
      desiredHandPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPose.prependTranslation(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setDesiredHandPose(robotSide, desiredHandPose);
      ik.holdCurrentChestOrientation();
      ik.holdCurrentPelvisOrientation();

      behaviorTestHelper.updateRobotModel();
      FramePose3D desiredHandPoseCopy = new FramePose3D(desiredHandPose);
      ReferenceFrame chestFrame = behaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      desiredHandPoseCopy.changeFrame(chestFrame);

      behaviorTestHelper.dispatchBehavior(ik);

      double totalSimDuration = 0.0;
      while (!ik.isDone())
      {
         double simDuration = 0.1;
         success = behaviorTestHelper.simulateNow(simDuration);
         assertTrue(success);

         totalSimDuration += simDuration;

         if (totalSimDuration >= 10)
            fail("IK is not converging");
      }

      assertFalse(ik.hasSolverFailed(), "Bad solution: " + ik.getSolutionQuality());

      success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      String handName = behaviorTestHelper.getControllerFullRobotModel().getHand(robotSide).getName();
      QuaternionReadOnly controllerDesiredHandOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(handName, behaviorTestHelper);
      Quaternion desiredHandOrientation = new Quaternion(desiredHandPose.getOrientation());

      double handAngleEpsilon = Math.toRadians(1);

      assertTrue(isOrientationEqual(desiredHandOrientation, controllerDesiredHandOrientation, handAngleEpsilon));

      QuaternionReadOnly controllerDesiredChestOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(chest.getName(), behaviorTestHelper);

      double chestAngleEpsilon = Math.toRadians(10);

      assertTrue(isOrientationEqual(initialChestOrientation, controllerDesiredChestOrientation, chestAngleEpsilon));

      Point3DReadOnly controllerDesiredHandPosition = EndToEndTestTools.findFeedbackControllerDesiredPosition(handName, behaviorTestHelper);

      Point3D handPosition = new Point3D(desiredHandPose.getPosition());

      double positionEpsilon = 1.0e-4;
      double positionDifference = handPosition.distance(controllerDesiredHandPosition);

      assertTrue(positionDifference < positionEpsilon, "Position difference: " + positionDifference);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForHandAngularLinearControl()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // simulate for a while to make sure the robot is still so small time differences between frame changes in the
      // controller and the unit test will not affect the outcome too much.
      boolean success = behaviorTestHelper.simulateNow(3.0);
      assertTrue(success);

      behaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(behaviorTestHelper.getRobotName(),
                                                                                     getRobotModel(),
                                                                                     behaviorTestHelper.getYoTime(),
                                                                                     behaviorTestHelper.getROS2Node(),
                                                                                     behaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrameR = behaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.RIGHT);
      ReferenceFrame handControlFrameL = behaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.LEFT);

      Quaternion offsetOrientationRight = new Quaternion();
      offsetOrientationRight.setYawPitchRoll(0.0, 0.0, 1.0);
      FramePose3D desiredHandPoseR = new FramePose3D(handControlFrameR);
      desiredHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion handQuatRight = new Quaternion(desiredHandPoseR.getOrientation());
      handQuatRight.multiply(handQuatRight, offsetOrientationRight);
      desiredHandPoseR.getOrientation().set(handQuatRight);
      desiredHandPoseR.prependTranslation(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setHandLinearControlOnly(RobotSide.RIGHT);
      ik.setDesiredHandPose(RobotSide.RIGHT, desiredHandPoseR);

      Quaternion offsetOrientationLeft = new Quaternion();
      offsetOrientationLeft.setYawPitchRoll(1.0, 1.0, 0.0);
      FramePose3D desiredHandPoseL = new FramePose3D(handControlFrameL);
      desiredHandPoseL.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion handQuatLeft = new Quaternion(desiredHandPoseL.getOrientation());
      handQuatLeft.multiply(handQuatLeft, offsetOrientationLeft);
      desiredHandPoseL.getOrientation().set(handQuatLeft);
      desiredHandPoseL.prependTranslation(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setHandLinearControlOnly(RobotSide.LEFT);
      ik.setDesiredHandPose(RobotSide.LEFT, desiredHandPoseL);

      ik.holdCurrentChestOrientation();
      ik.holdCurrentPelvisOrientation();

      behaviorTestHelper.updateRobotModel();
      FramePose3D desiredHandPoseLCopy = new FramePose3D(desiredHandPoseL);
      FramePose3D desiredHandPoseRCopy = new FramePose3D(desiredHandPoseR);
      ReferenceFrame chestFrame = behaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      desiredHandPoseLCopy.changeFrame(chestFrame);
      desiredHandPoseRCopy.changeFrame(chestFrame);

      behaviorTestHelper.dispatchBehavior(ik);

      double totalSimDuration = 0.0;
      while (!ik.isDone())
      {
         double simDuration = 0.1;
         success = behaviorTestHelper.simulateNow(simDuration);
         assertTrue(success);

         totalSimDuration += simDuration;

         if (totalSimDuration >= 10)
            fail("IK is not converging");
      }

      assertFalse(ik.hasSolverFailed(), "Bad solution: " + ik.getSolutionQuality());

      success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      FramePose3D currentHandPoseR = new FramePose3D(handControlFrameR);
      currentHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());
      double currentRollR = currentHandPoseR.getRoll();
      FramePose3D currentHandPoseL = new FramePose3D(handControlFrameL);
      currentHandPoseL.changeFrame(ReferenceFrame.getWorldFrame());
      double currentYawL = currentHandPoseL.getYaw();
      double currentPitchL = currentHandPoseL.getPitch();

      double angleEpsilon = Math.toRadians(2);

      assertNotEquals(currentRollR, desiredHandPoseR.getRoll(), angleEpsilon, "Current roll: " + currentRollR);
      assertNotEquals(currentYawL, desiredHandPoseL.getYaw(), angleEpsilon, "Current yaw: " + currentYawL);
      assertNotEquals(currentPitchL, desiredHandPoseL.getPitch(), angleEpsilon, "Current pitch: " + currentPitchL);

      String leftHandName = behaviorTestHelper.getControllerFullRobotModel().getHand(RobotSide.LEFT).getName();
      String rightHandName = behaviorTestHelper.getControllerFullRobotModel().getHand(RobotSide.RIGHT).getName();

      Point3DReadOnly controllerDesiredHandPositionR = EndToEndTestTools.findFeedbackControllerDesiredPosition(rightHandName, behaviorTestHelper);
      Point3DReadOnly controllerDesiredHandPositionL = EndToEndTestTools.findFeedbackControllerDesiredPosition(leftHandName, behaviorTestHelper);
      Point3D rightPosition = new Point3D(desiredHandPoseR.getPosition());
      Point3D leftPosition = new Point3D(desiredHandPoseL.getPosition());
      double rightDifference = rightPosition.distance(controllerDesiredHandPositionR);
      double leftDifference = leftPosition.distance(controllerDesiredHandPositionL);

      double positionEpsilon = 1.0e-3;

      assertTrue(rightDifference < positionEpsilon, "Position difference: " + rightDifference);
      assertTrue(leftDifference < positionEpsilon, "Position difference: " + leftDifference);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForHandRollConstraint()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // simulate for a while to make sure the robot is still so small time differences between frame changes in the
      // controller and the unit test will not affect the outcome too much.
      boolean success = behaviorTestHelper.simulateNow(3.0);
      assertTrue(success);

      behaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(behaviorTestHelper.getRobotName(),
                                                                                     getRobotModel(),
                                                                                     behaviorTestHelper.getYoTime(),
                                                                                     behaviorTestHelper.getROS2Node(),
                                                                                     behaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrame = behaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.RIGHT);

      Quaternion offsetOrientation = new Quaternion();
      offsetOrientation.setYawPitchRoll(0.0, 0.0, 1.0);
      FramePose3D desiredHandPose = new FramePose3D(handControlFrame);
      desiredHandPose.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion handQuat = new Quaternion(desiredHandPose.getOrientation());
      handQuat.multiply(handQuat, offsetOrientation);
      desiredHandPose.getOrientation().set(handQuat);
      desiredHandPose.prependTranslation(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setHandLinearControlAndYawPitchOnly(RobotSide.RIGHT);
      ik.setDesiredHandPose(RobotSide.RIGHT, desiredHandPose);

      ik.holdCurrentChestOrientation();
      ik.holdCurrentPelvisOrientation();

      behaviorTestHelper.updateRobotModel();
      FramePose3D desiredHandPoseCopy = new FramePose3D(desiredHandPose);
      ReferenceFrame chestFrame = behaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      desiredHandPoseCopy.changeFrame(chestFrame);

      behaviorTestHelper.dispatchBehavior(ik);

      double totalSimDuration = 0.0;
      while (!ik.isDone())
      {
         double simDuration = 0.1;
         success = behaviorTestHelper.simulateNow(simDuration);
         assertTrue(success);

         totalSimDuration += simDuration;

         if (totalSimDuration >= 10)
            fail("IK is not converging");
      }

      assertFalse(ik.hasSolverFailed(), "Bad solution: " + ik.getSolutionQuality());

      success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      String handName = behaviorTestHelper.getControllerFullRobotModel().getHand(RobotSide.RIGHT).getName();
      Point3DReadOnly controllerDesiredHandPosition = EndToEndTestTools.findFeedbackControllerDesiredPosition(handName, behaviorTestHelper);

      FramePose3D currentHandPose = new FramePose3D(handControlFrame);
      currentHandPose.changeFrame(ReferenceFrame.getWorldFrame());
      double currentRoll = currentHandPose.getRoll();

      double angleEpsilon = Math.toRadians(5);

      assertNotEquals(currentRoll, desiredHandPose.getRoll(), angleEpsilon, "Current roll " + currentRoll);

      Point3D handPosition = new Point3D(desiredHandPose.getPosition());

      double positionEpsilon = 1.0e-4;
      double positionDifference = handPosition.distance(controllerDesiredHandPosition);

      assertTrue(positionDifference < positionEpsilon, "Position difference: " + positionDifference);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForChestAngularControl()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      behaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(behaviorTestHelper.getRobotName(),
                                                                                     getRobotModel(),
                                                                                     behaviorTestHelper.getYoTime(),
                                                                                     behaviorTestHelper.getROS2Node(),
                                                                                     behaviorTestHelper.getSDFFullRobotModel());

      Quaternion offsetOrientationChest = new Quaternion();
      offsetOrientationChest.setYawPitchRoll(0.3, 0.0, 0.1);
      ReferenceFrame chestControlFrame = behaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      FrameQuaternion desiredChestOrientation = new FrameQuaternion(chestControlFrame);
      double initialChestPitch = desiredChestOrientation.getPitch();
      double initialChestYaw = desiredChestOrientation.getYaw();
      desiredChestOrientation.set(offsetOrientationChest);
      desiredChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      ik.setTrajectoryTime(0.5);
      ik.setChestAngularControl(true, false, false);
      ik.setDesiredChestOrientation(desiredChestOrientation);

      behaviorTestHelper.dispatchBehavior(ik);

      double totalSimDuration = 0.0;
      while (!ik.isDone())
      {
         double simDuration = 0.1;
         success = behaviorTestHelper.simulateNow(simDuration);
         assertTrue(success);

         totalSimDuration += simDuration;

         if (totalSimDuration >= 10)
            fail("IK is not converging");
      }

      assertFalse(ik.hasSolverFailed(), "Bad solution: " + ik.getSolutionQuality());

      success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      FrameQuaternion currentChestOrientation = new FrameQuaternion(chestControlFrame);
      currentChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      double currentChestRoll = currentChestOrientation.getRoll();
      double currentChestYaw = currentChestOrientation.getYaw();
      double currentChestPitch = currentChestOrientation.getPitch();

      double angleEpsilon = Math.toRadians(1);

      assertEquals(desiredChestOrientation.getRoll(),
                   currentChestRoll,
                   angleEpsilon,
                   "Expected: " + desiredChestOrientation.getRoll() + " Received: " + currentChestRoll);
      assertEquals(initialChestYaw, currentChestYaw, angleEpsilon, "Expected: " + initialChestYaw + " Received: " + currentChestYaw);
      assertEquals(initialChestPitch, currentChestPitch, angleEpsilon, "Expected: " + initialChestPitch + " Received: " + currentChestPitch);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForPelvisAngularControl()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      behaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(behaviorTestHelper.getRobotName(),
                                                                                     getRobotModel(),
                                                                                     behaviorTestHelper.getYoTime(),
                                                                                     behaviorTestHelper.getROS2Node(),
                                                                                     behaviorTestHelper.getSDFFullRobotModel());

      Quaternion offsetOrientationPelvis = new Quaternion();
      offsetOrientationPelvis.setYawPitchRoll(0.3, 0.0, 0.1);
      ReferenceFrame pelvisControlFrame = behaviorTestHelper.getControllerFullRobotModel().getPelvis().getBodyFixedFrame();
      FrameQuaternion desiredPelvisOrientation = new FrameQuaternion(pelvisControlFrame);
      double initialPelvisPitch = desiredPelvisOrientation.getPitch();
      double initialPelvisYaw = desiredPelvisOrientation.getYaw();
      desiredPelvisOrientation.set(offsetOrientationPelvis);
      desiredPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      ik.setTrajectoryTime(0.5);
      ik.setPelvisAngularControl(true, false, false);
      ik.setDesiredPelvisOrientation(desiredPelvisOrientation);

      behaviorTestHelper.dispatchBehavior(ik);

      double totalSimDuration = 0.0;
      while (!ik.isDone())
      {
         double simDuration = 0.1;
         success = behaviorTestHelper.simulateNow(simDuration);
         assertTrue(success);

         totalSimDuration += simDuration;

         if (totalSimDuration >= 10)
            fail("IK is not converging");
      }

      assertFalse(ik.hasSolverFailed(), "Bad solution: " + ik.getSolutionQuality());

      success = behaviorTestHelper.simulateNow(1.0);
      assertTrue(success);

      FrameQuaternion currentPelvisOrientation = new FrameQuaternion(pelvisControlFrame);
      currentPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      double currentPelvisRoll = currentPelvisOrientation.getRoll();
      double currentPelvisYaw = currentPelvisOrientation.getYaw();
      double currentPelvisPitch = currentPelvisOrientation.getPitch();

      double angleEpsilon = Math.toRadians(1.5);

      assertEquals(desiredPelvisOrientation.getRoll(),
                   currentPelvisRoll,
                   angleEpsilon,
                   "Expected: " + desiredPelvisOrientation.getRoll() + " Received: " + currentPelvisRoll);
      assertEquals(initialPelvisYaw, currentPelvisYaw, angleEpsilon, "Expected: " + initialPelvisYaw + " Received: " + currentPelvisYaw);
      assertEquals(initialPelvisPitch, currentPelvisPitch, angleEpsilon, "Expected: " + initialPelvisPitch + " Received: " + currentPelvisPitch);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private boolean isOrientationEqual(QuaternionReadOnly initialQuat, QuaternionReadOnly finalQuat, double angleEpsilon)
   {
      Quaternion quatDifference = new Quaternion(initialQuat);
      quatDifference.multiplyConjugateOther(finalQuat);

      AxisAngle angleDifference = new AxisAngle();
      angleDifference.set(quatDifference);
      AngleTools.trimAngleMinusPiToPi(angleDifference.getAngle());

      return Math.abs(angleDifference.getAngle()) < angleEpsilon;
   }

   private void setupKinematicsToolboxModule() throws IOException
   {
      DRCRobotModel robotModel = getRobotModel();
      kinematicsToolboxModule = new KinematicsToolboxModule(robotModel, isKinematicsToolboxVisualizerEnabled, PubSubImplementation.INTRAPROCESS);
   }
}
