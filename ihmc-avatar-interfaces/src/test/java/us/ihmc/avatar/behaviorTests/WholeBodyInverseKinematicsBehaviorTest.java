package us.ihmc.avatar.behaviorTests;

import static us.ihmc.robotics.Assert.*;

import java.io.IOException;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.testTools.DRCBehaviorTestHelper;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.commons.thread.ThreadTools;
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
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@Tag("humanoid-behaviors")
public abstract class WholeBodyInverseKinematicsBehaviorTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private boolean isKinematicsToolboxVisualizerEnabled = false;
   private DRCBehaviorTestHelper drcBehaviorTestHelper;
   private KinematicsToolboxModule kinematicsToolboxModule;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcBehaviorTestHelper != null)
      {
         drcBehaviorTestHelper.closeAndDispose();
         drcBehaviorTestHelper = null;
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

      drcBehaviorTestHelper = new DRCBehaviorTestHelper(envrionment, getSimpleRobotName(), null, simulationTestingParameters, getRobotModel());

      setupKinematicsToolboxModule();
   }

   @Test
   public void testSolvingForAHandPose() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // simulate for a while to make sure the robot is still so small time differences between frame changes in the
      // controller and the unit test will not affect the outcome too much.
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.RIGHT;

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(drcBehaviorTestHelper.getRobotName(), getRobotModel(),
                                                                                     drcBehaviorTestHelper.getYoTime(), drcBehaviorTestHelper.getRos2Node(),
                                                                                     drcBehaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrame = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(robotSide);

      FullHumanoidRobotModel fullRobotModel = drcBehaviorTestHelper.getControllerFullRobotModel();
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

      drcBehaviorTestHelper.updateRobotModel();
      FramePose3D desiredHandPoseCopy = new FramePose3D(desiredHandPose);
      ReferenceFrame chestFrame = fullRobotModel.getChest().getBodyFixedFrame();
      desiredHandPoseCopy.changeFrame(chestFrame);

      drcBehaviorTestHelper.dispatchBehavior(ik);

      while (!ik.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         assertTrue(success);
      }

      assertFalse("Bad solution: " + ik.getSolutionQuality(), ik.hasSolverFailed());

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      String pelvisName = fullRobotModel.getPelvis().getName();
      QuaternionReadOnly controllerDesiredChestOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(chest.getName(), scs);
      QuaternionReadOnly controllerDesiredPelvisOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(pelvisName, scs);

      double angleEpsilon = Math.toRadians(2.0);

      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(initialChestOrientation, controllerDesiredChestOrientation, angleEpsilon);
      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(initialPelvisOrientation, controllerDesiredPelvisOrientation, angleEpsilon);

      String handName = fullRobotModel.getHand(robotSide).getName();
      Point3DReadOnly controllerDesiredHandPosition = EndToEndTestTools.findFeedbackControllerDesiredPosition(handName, scs);

      Point3D handPosition = new Point3D(desiredHandPose.getPosition());

      double positionEpsilon = 1.0e-4;
      double positionDifference = handPosition.distance(controllerDesiredHandPosition);

      assertTrue("Position difference: " + positionDifference, positionDifference < positionEpsilon);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForBothHandPoses() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // simulate for a while to make sure the robot is still so small time differences between frame changes in the
      // controller and the unit test will not affect the outcome too much.
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      assertTrue(success);

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(drcBehaviorTestHelper.getRobotName(), getRobotModel(),
                                                                                     drcBehaviorTestHelper.getYoTime(), drcBehaviorTestHelper.getRos2Node(),
                                                                                     drcBehaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrameR = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.RIGHT);
      ReferenceFrame handControlFrameL = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.LEFT);

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

      drcBehaviorTestHelper.updateRobotModel();
      FramePose3D desiredHandPoseLCopy = new FramePose3D(desiredHandPoseL);
      FramePose3D desiredHandPoseRCopy = new FramePose3D(desiredHandPoseR);
      ReferenceFrame chestFrame = drcBehaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      desiredHandPoseLCopy.changeFrame(chestFrame);
      desiredHandPoseRCopy.changeFrame(chestFrame);

      drcBehaviorTestHelper.dispatchBehavior(ik);

      while (!ik.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         assertTrue(success);
      }

      assertFalse("Bad solution: " + ik.getSolutionQuality(), ik.hasSolverFailed());

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      String rightHandName = drcBehaviorTestHelper.getControllerFullRobotModel().getHand(RobotSide.RIGHT).getName();
      String leftHandName = drcBehaviorTestHelper.getControllerFullRobotModel().getHand(RobotSide.LEFT).getName();

      QuaternionReadOnly controllerDesiredHandOrientationR = EndToEndTestTools.findFeedbackControllerDesiredOrientation(rightHandName, scs);
      Quaternion desiredHandOrientationR = new Quaternion(desiredHandPoseR.getOrientation());
      QuaternionReadOnly controllerDesiredHandOrientationL = EndToEndTestTools.findFeedbackControllerDesiredOrientation(leftHandName, scs);
      Quaternion desiredHandOrientationL = new Quaternion(desiredHandPoseL.getOrientation());

      double handAngleEpsilon = Math.toRadians(1.0);

      assertTrue(isOrientationEqual(desiredHandOrientationR, controllerDesiredHandOrientationR, handAngleEpsilon));
      assertTrue(isOrientationEqual(desiredHandOrientationL, controllerDesiredHandOrientationL, handAngleEpsilon));

      Point3DReadOnly controllerDesiredHandPositionR = EndToEndTestTools.findFeedbackControllerDesiredPosition(rightHandName, scs);
      Point3DReadOnly controllerDesiredHandPositionL = EndToEndTestTools.findFeedbackControllerDesiredPosition(leftHandName, scs);
      Point3D rightPosition = new Point3D(desiredHandPoseR.getPosition());
      Point3D leftPosition = new Point3D(desiredHandPoseL.getPosition());
      double rightDifference = rightPosition.distance(controllerDesiredHandPositionR);
      double leftDifference = leftPosition.distance(controllerDesiredHandPositionL);

      double positionEpsilon = 1.0e-4;

      assertTrue("Position difference: " + rightDifference, rightDifference < positionEpsilon);
      assertTrue("Position difference: " + leftDifference, leftDifference < positionEpsilon);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForHandSelectionMatrix() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // simulate for a while to make sure the robot is still so small time differences between frame changes in the
      // controller and the unit test will not affect the outcome too much.
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      assertTrue(success);

      RobotSide robotSide = RobotSide.RIGHT;

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(drcBehaviorTestHelper.getRobotName(), getRobotModel(),
                                                                                     drcBehaviorTestHelper.getYoTime(), drcBehaviorTestHelper.getRos2Node(),
                                                                                     drcBehaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrame = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(robotSide);

      RigidBodyBasics chest = drcBehaviorTestHelper.getControllerFullRobotModel().getChest();
      ReferenceFrame chestControlFrame = chest.getBodyFixedFrame();
      FrameQuaternion initialChestOrientation = new FrameQuaternion(chestControlFrame);
      initialChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion offsetOrientation = new Quaternion();
      offsetOrientation.setYawPitchRoll(0.0, 0.0, 0.1);
      FramePose3D desiredHandPose = new FramePose3D(handControlFrame);
      desiredHandPose.setOrientation(offsetOrientation);
      desiredHandPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredHandPose.prependTranslation(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setDesiredHandPose(robotSide, desiredHandPose);
      ik.holdCurrentChestOrientation();
      ik.holdCurrentPelvisOrientation();

      drcBehaviorTestHelper.updateRobotModel();
      FramePose3D desiredHandPoseCopy = new FramePose3D(desiredHandPose);
      ReferenceFrame chestFrame = drcBehaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      desiredHandPoseCopy.changeFrame(chestFrame);

      drcBehaviorTestHelper.dispatchBehavior(ik);

      while (!ik.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         assertTrue(success);
      }

      assertFalse("Bad solution: " + ik.getSolutionQuality(), ik.hasSolverFailed());

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      String handName = drcBehaviorTestHelper.getControllerFullRobotModel().getHand(robotSide).getName();
      QuaternionReadOnly controllerDesiredHandOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(handName, scs);
      Quaternion desiredHandOrientation = new Quaternion(desiredHandPose.getOrientation());

      double handAngleEpsilon = Math.toRadians(1);

      assertTrue(isOrientationEqual(desiredHandOrientation, controllerDesiredHandOrientation, handAngleEpsilon));

      QuaternionReadOnly controllerDesiredChestOrientation = EndToEndTestTools.findFeedbackControllerDesiredOrientation(chest.getName(), scs);

      double chestAngleEpsilon = Math.toRadians(10);

      assertTrue(isOrientationEqual(initialChestOrientation, controllerDesiredChestOrientation, chestAngleEpsilon));

      Point3DReadOnly controllerDesiredHandPosition = EndToEndTestTools.findFeedbackControllerDesiredPosition(handName, scs);

      Point3D handPosition = new Point3D(desiredHandPose.getPosition());

      double positionEpsilon = 1.0e-4;
      double positionDifference = handPosition.distance(controllerDesiredHandPosition);

      assertTrue("Position difference: " + positionDifference, positionDifference < positionEpsilon);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForHandAngularLinearControl() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // simulate for a while to make sure the robot is still so small time differences between frame changes in the
      // controller and the unit test will not affect the outcome too much.
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      assertTrue(success);

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();
      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(drcBehaviorTestHelper.getRobotName(), getRobotModel(),
                                                                                     drcBehaviorTestHelper.getYoTime(), drcBehaviorTestHelper.getRos2Node(),
                                                                                     drcBehaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrameR = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.RIGHT);
      ReferenceFrame handControlFrameL = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.LEFT);

      Quaternion offsetOrientationRight = new Quaternion();
      offsetOrientationRight.setYawPitchRoll(0.0, 0.0, 1.0);
      FramePose3D desiredHandPoseR = new FramePose3D(handControlFrameR);
      desiredHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion handQuatRight = new Quaternion(desiredHandPoseR.getOrientation());
      handQuatRight.multiply(handQuatRight, offsetOrientationRight);
      desiredHandPoseR.setOrientation(handQuatRight);
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
      desiredHandPoseL.setOrientation(handQuatLeft);
      desiredHandPoseL.prependTranslation(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setHandLinearControlOnly(RobotSide.LEFT);
      ik.setDesiredHandPose(RobotSide.LEFT, desiredHandPoseL);

      ik.holdCurrentChestOrientation();
      ik.holdCurrentPelvisOrientation();

      drcBehaviorTestHelper.updateRobotModel();
      FramePose3D desiredHandPoseLCopy = new FramePose3D(desiredHandPoseL);
      FramePose3D desiredHandPoseRCopy = new FramePose3D(desiredHandPoseR);
      ReferenceFrame chestFrame = drcBehaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      desiredHandPoseLCopy.changeFrame(chestFrame);
      desiredHandPoseRCopy.changeFrame(chestFrame);

      drcBehaviorTestHelper.dispatchBehavior(ik);

      while (!ik.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         assertTrue(success);
      }

      assertFalse("Bad solution: " + ik.getSolutionQuality(), ik.hasSolverFailed());

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      FramePose3D currentHandPoseR = new FramePose3D(handControlFrameR);
      currentHandPoseR.changeFrame(ReferenceFrame.getWorldFrame());
      double currentRollR = currentHandPoseR.getRoll();
      FramePose3D currentHandPoseL = new FramePose3D(handControlFrameL);
      currentHandPoseL.changeFrame(ReferenceFrame.getWorldFrame());
      double currentYawL = currentHandPoseL.getYaw();
      double currentPitchL = currentHandPoseL.getPitch();

      double angleEpsilon = Math.toRadians(2);

      assertNotEquals("Current roll: " + currentRollR, currentRollR, desiredHandPoseR.getRoll(), angleEpsilon);
      assertNotEquals("Current yaw: " + currentYawL, currentYawL, desiredHandPoseL.getYaw(), angleEpsilon);
      assertNotEquals("Current pitch: " + currentPitchL, currentPitchL, desiredHandPoseL.getPitch(), angleEpsilon);

      String leftHandName = drcBehaviorTestHelper.getControllerFullRobotModel().getHand(RobotSide.LEFT).getName();
      String rightHandName = drcBehaviorTestHelper.getControllerFullRobotModel().getHand(RobotSide.RIGHT).getName();

      Point3DReadOnly controllerDesiredHandPositionR = EndToEndTestTools.findFeedbackControllerDesiredPosition(rightHandName, scs);
      Point3DReadOnly controllerDesiredHandPositionL = EndToEndTestTools.findFeedbackControllerDesiredPosition(leftHandName, scs);
      Point3D rightPosition = new Point3D(desiredHandPoseR.getPosition());
      Point3D leftPosition = new Point3D(desiredHandPoseL.getPosition());
      double rightDifference = rightPosition.distance(controllerDesiredHandPositionR);
      double leftDifference = leftPosition.distance(controllerDesiredHandPositionL);

      double positionEpsilon = 1.0e-3;

      assertTrue("Position difference: " + rightDifference, rightDifference < positionEpsilon);
      assertTrue("Position difference: " + leftDifference, leftDifference < positionEpsilon);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForHandRollConstraint() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      // simulate for a while to make sure the robot is still so small time differences between frame changes in the
      // controller and the unit test will not affect the outcome too much.
      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(3.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(drcBehaviorTestHelper.getRobotName(), getRobotModel(),
                                                                                     drcBehaviorTestHelper.getYoTime(), drcBehaviorTestHelper.getRos2Node(),
                                                                                     drcBehaviorTestHelper.getSDFFullRobotModel());

      ReferenceFrame handControlFrame = drcBehaviorTestHelper.getReferenceFrames().getHandFrame(RobotSide.RIGHT);

      Quaternion offsetOrientation = new Quaternion();
      offsetOrientation.setYawPitchRoll(0.0, 0.0, 1.0);
      FramePose3D desiredHandPose = new FramePose3D(handControlFrame);
      desiredHandPose.changeFrame(ReferenceFrame.getWorldFrame());

      Quaternion handQuat = new Quaternion(desiredHandPose.getOrientation());
      handQuat.multiply(handQuat, offsetOrientation);
      desiredHandPose.setOrientation(handQuat);
      desiredHandPose.prependTranslation(0.20, 0.0, 0.0);
      ik.setTrajectoryTime(0.5);
      ik.setHandLinearControlAndYawPitchOnly(RobotSide.RIGHT);
      ik.setDesiredHandPose(RobotSide.RIGHT, desiredHandPose);

      ik.holdCurrentChestOrientation();
      ik.holdCurrentPelvisOrientation();

      drcBehaviorTestHelper.updateRobotModel();
      FramePose3D desiredHandPoseCopy = new FramePose3D(desiredHandPose);
      ReferenceFrame chestFrame = drcBehaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      desiredHandPoseCopy.changeFrame(chestFrame);

      drcBehaviorTestHelper.dispatchBehavior(ik);

      while (!ik.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         assertTrue(success);
      }

      assertFalse("Bad solution: " + ik.getSolutionQuality(), ik.hasSolverFailed());

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      SimulationConstructionSet scs = drcBehaviorTestHelper.getSimulationConstructionSet();

      String handName = drcBehaviorTestHelper.getControllerFullRobotModel().getHand(RobotSide.RIGHT).getName();
      Point3DReadOnly controllerDesiredHandPosition = EndToEndTestTools.findFeedbackControllerDesiredPosition(handName, scs);

      FramePose3D currentHandPose = new FramePose3D(handControlFrame);
      currentHandPose.changeFrame(ReferenceFrame.getWorldFrame());
      double currentRoll = currentHandPose.getRoll();

      double angleEpsilon = Math.toRadians(5);

      assertNotEquals("Current roll " + currentRoll, currentRoll, desiredHandPose.getRoll(), angleEpsilon);

      Point3D handPosition = new Point3D(desiredHandPose.getPosition());

      double positionEpsilon = 1.0e-4;
      double positionDifference = handPosition.distance(controllerDesiredHandPosition);

      assertTrue("Position difference: " + positionDifference, positionDifference < positionEpsilon);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForChestAngularControl() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(drcBehaviorTestHelper.getRobotName(), getRobotModel(),
                                                                                     drcBehaviorTestHelper.getYoTime(), drcBehaviorTestHelper.getRos2Node(),
                                                                                     drcBehaviorTestHelper.getSDFFullRobotModel());

      Quaternion offsetOrientationChest = new Quaternion();
      offsetOrientationChest.setYawPitchRoll(0.3, 0.0, 0.1);
      ReferenceFrame chestControlFrame = drcBehaviorTestHelper.getControllerFullRobotModel().getChest().getBodyFixedFrame();
      FrameQuaternion desiredChestOrientation = new FrameQuaternion(chestControlFrame);
      double initialChestPitch = desiredChestOrientation.getPitch();
      double initialChestYaw = desiredChestOrientation.getYaw();
      desiredChestOrientation.set(offsetOrientationChest);
      desiredChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      ik.setTrajectoryTime(0.5);
      ik.setChestAngularControl(true, false, false);
      ik.setDesiredChestOrientation(desiredChestOrientation);

      drcBehaviorTestHelper.dispatchBehavior(ik);

      while (!ik.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         assertTrue(success);
      }

      assertFalse("Bad solution: " + ik.getSolutionQuality(), ik.hasSolverFailed());

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      FrameQuaternion currentChestOrientation = new FrameQuaternion(chestControlFrame);
      currentChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      double currentChestRoll = currentChestOrientation.getRoll();
      double currentChestYaw = currentChestOrientation.getYaw();
      double currentChestPitch = currentChestOrientation.getPitch();

      double angleEpsilon = Math.toRadians(1);

      assertEquals("Expected: " + desiredChestOrientation.getRoll() + " Received: " + currentChestRoll, desiredChestOrientation.getRoll(), currentChestRoll,
                   angleEpsilon);
      assertEquals("Expected: " + initialChestYaw + " Received: " + currentChestYaw, initialChestYaw, currentChestYaw, angleEpsilon);
      assertEquals("Expected: " + initialChestPitch + " Received: " + currentChestPitch, initialChestPitch, currentChestPitch, angleEpsilon);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSolvingForPelvisAngularControl() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      boolean success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      drcBehaviorTestHelper.updateRobotModel();

      WholeBodyInverseKinematicsBehavior ik = new WholeBodyInverseKinematicsBehavior(drcBehaviorTestHelper.getRobotName(), getRobotModel(),
                                                                                     drcBehaviorTestHelper.getYoTime(), drcBehaviorTestHelper.getRos2Node(),
                                                                                     drcBehaviorTestHelper.getSDFFullRobotModel());

      Quaternion offsetOrientationPelvis = new Quaternion();
      offsetOrientationPelvis.setYawPitchRoll(0.3, 0.0, 0.1);
      ReferenceFrame pelvisControlFrame = drcBehaviorTestHelper.getControllerFullRobotModel().getPelvis().getBodyFixedFrame();
      FrameQuaternion desiredPelvisOrientation = new FrameQuaternion(pelvisControlFrame);
      double initialPelvisPitch = desiredPelvisOrientation.getPitch();
      double initialPelvisYaw = desiredPelvisOrientation.getYaw();
      desiredPelvisOrientation.set(offsetOrientationPelvis);
      desiredPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      ik.setTrajectoryTime(0.5);
      ik.setPelvisAngularControl(true, false, false);
      ik.setDesiredPelvisOrientation(desiredPelvisOrientation);

      drcBehaviorTestHelper.dispatchBehavior(ik);

      while (!ik.isDone())
      {
         success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(0.1);
         assertTrue(success);
      }

      assertFalse("Bad solution: " + ik.getSolutionQuality(), ik.hasSolverFailed());

      success = drcBehaviorTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      FrameQuaternion currentPelvisOrientation = new FrameQuaternion(pelvisControlFrame);
      currentPelvisOrientation.changeFrame(ReferenceFrame.getWorldFrame());
      double currentPelvisRoll = currentPelvisOrientation.getRoll();
      double currentPelvisYaw = currentPelvisOrientation.getYaw();
      double currentPelvisPitch = currentPelvisOrientation.getPitch();

      double angleEpsilon = Math.toRadians(1.5);

      assertEquals("Expected: " + desiredPelvisOrientation.getRoll() + " Received: " + currentPelvisRoll, desiredPelvisOrientation.getRoll(), currentPelvisRoll,
                   angleEpsilon);
      assertEquals("Expected: " + initialPelvisYaw + " Received: " + currentPelvisYaw, initialPelvisYaw, currentPelvisYaw, angleEpsilon);
      assertEquals("Expected: " + initialPelvisPitch + " Received: " + currentPelvisPitch, initialPelvisPitch, currentPelvisPitch, angleEpsilon);

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
