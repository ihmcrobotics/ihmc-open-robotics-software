package us.ihmc.avatar.pushRecovery;

import java.io.InputStream;
import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.PushRecoveryStateEnum;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import static us.ihmc.robotics.Assert.*;

public abstract class DRCPushRecoveryTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private static String defaultScript = "scripts/ExerciseAndJUnitScripts/walkingPushTestScript.xml";

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static double simulationTime = 6.0;

   private PushRobotController pushRobotController;

   private double swingTime, transferTime;

   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();

   private SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   private YoEnum<HighLevelControllerName> currentHighLevelState;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      singleSupportStartConditions = null;
      doubleSupportStartConditions = null;
      pushRobotController = null;
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected abstract DRCRobotModel getRobotModel();

   @Test
   public void testPushICPOptimiWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest(getScriptFilePath(), false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 600.0;
      double duration = 0.1;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, simulationTime);
   }

   @Test
   public void testPushWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest(getScriptFilePath(), true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delayMultiplier = getPushWhileInSwingDelayMultiplier();
      double delay = delayMultiplier * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 550.0;
      double duration = 0.1;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, simulationTime);
   }

   @Test
   public void testRecoveringWithSwingSpeedUpWhileInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest(getScriptFilePath(), true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.25 * swingTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 450.0;
      double duration = 0.1;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, simulationTime);
   }

   @Test
   public void testPushWhileInTransfer() throws SimulationExceededMaximumTimeException
   {
      setupTest(getScriptFilePath(), false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = doubleSupportStartConditions.get(RobotSide.RIGHT);
      double delay = 0.5 * transferTime;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = 450.0;
      double duration = 0.1;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, simulationTime);
   }

   @Test
   public void testPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 1.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = 350.0;
      double duration = 0.15;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, simulationTime);
   }

   @Test
   public void testSidewaysPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.5;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double magnitude = 400.0;
      double duration = 0.35;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, simulationTime);
   }

   @Test
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 1.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = 350.0;
      double duration = 0.15;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, simulationTime);
   }

   @Test
   public void testLongForwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      forceDirection.normalize();
      double magnitude = 100.0;
      double duration = 1.0;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, duration + 2.0);
   }

   @Test
   public void testControllerFailureKicksIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, false);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double magnitude = 160.0;
      double duration = 3.0;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      assertFalse(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration + 2.0));
   }

   @Test
   public void testLongBackwardPushWhileStanding() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double magnitude = 100.0;
      double duration = 1.0;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, duration + 2.0);
   }

   @Test
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      forceDirection.normalize();
      double magnitude = 100.0;
      double duration = 1.0;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, duration + 2.0);
   }

   @Test
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double magnitude = 100.0;
      double duration = 1.0;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, duration + 2.0);
   }

   @Test
   public void testRecoveryWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(
            drcSimulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide, LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      drcSimulationTestHelper.publishToController(footPosePacket);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = 180.0;
      double duration = 0.2;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, 1.5);
   }

   @Test
   public void testRecoveryForwardWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(
              drcSimulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide, LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      drcSimulationTestHelper.publishToController(footPosePacket);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = 350.0;
      double duration = 0.2;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, 2.5);
   }

   @Test
   public void testRecoverySidewaysWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(
            drcSimulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide, LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      drcSimulationTestHelper.publishToController(footPosePacket);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = 200.0;
      double duration = 0.2;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, 4.0);
   }

   @Test
   public void testRecoveryAngledWhileInFlamingoStance() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(
            drcSimulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide, LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      drcSimulationTestHelper.publishToController(footPosePacket);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 1.0, 0.0);
      double magnitude = 350.0;
      double duration = 0.2;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, 2.5);
   }

   @Test
   public void testFailureAfterRecoveryStep() throws SimulationExceededMaximumTimeException
   {
      setupTest(null, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(
            drcSimulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide, LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      drcSimulationTestHelper.publishToController(footPosePacket);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 1.0, 0.0);
      double magnitude = 250.0;
      double duration = 0.3;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      final YoEnum<PushRecoveryStateEnum> pushRecoveryState = (YoEnum<PushRecoveryStateEnum>)  drcSimulationTestHelper.getYoVariable("PushRecoveryHighLevelHumanoidController", "pushRecoveryCurrentState");
      final YoInteger numberOfRecoveryStepsTaken = (YoInteger) drcSimulationTestHelper.getYoVariable("numberOfRecoveryStepsTaken");
      final YoInteger maxNumberOfSteps = (YoInteger) drcSimulationTestHelper.getYoVariable("maxNumberOfRecoveryStepsToTake");
      maxNumberOfSteps.set(1);

      AtomicBoolean tookStep = new AtomicBoolean(false);
      pushRecoveryState.addListener(state -> {
         if (pushRecoveryState.getEnumValue() == PushRecoveryStateEnum.TO_PUSH_RECOVERY_LEFT_SUPPORT
             || pushRecoveryState.getEnumValue() == PushRecoveryStateEnum.TO_PUSH_RECOVERY_RIGHT_SUPPORT)
            tookStep.set(true);
      });
      pushRobotController.queueForceDelayed(new PushRecoveryTransferStartCondition(pushRecoveryState), 0.02, forceDirection, magnitude, duration);

      assertFalse(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0));

      assertEquals(numberOfRecoveryStepsTaken.getIntegerValue(), 2);
      assertTrue(tookStep.get());

//      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, 2.5);
   }

   private void setupTest(String scriptName, boolean enablePushRecoveryOnFailure) throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());

      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");
      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
//      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel);
      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel.getChest().getParentJoint().getName(), new Vector3D(0, 0, getPushPositionZHeightInChestFrame()));
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());

      if (scriptName != null && !scriptName.isEmpty())
      {
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.001));
         InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
         drcSimulationTestHelper.loadScriptFile(scriptInputStream, ReferenceFrame.getWorldFrame());
      }

      // get rid of this once push recovery is enabled by default
//      YoBoolean enable = (YoBoolean) scs.findVariable("PushRecoveryControlModule", "enablePushRecovery");
//      enable.set(enablePushRecoveryControlModule);
      YoBoolean enableOnFailure = (YoBoolean) scs.findVariable(WalkingHighLevelHumanoidController.class.getSimpleName(),
            "enablePushRecoveryOnFailure");
      enableOnFailure.set(enablePushRecoveryOnFailure);
      currentHighLevelState = (YoEnum<HighLevelControllerName>) scs.findVariable("highLevelControllerNameCurrentState");

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked")
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) scs.findVariable(sidePrefix + "FootControlModule",
               footPrefix + "CurrentState");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.findVariable("WalkingHighLevelHumanoidController",
               "walkingCurrentState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      final YoEnum<PushRecoveryStateEnum> pushRecoveryState = (YoEnum<PushRecoveryStateEnum>) scs.findVariable("PushRecoveryController",
                                                                                                               "pushRecoveryCurrentState");

      setupCamera(scs);
      swingTime = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();
      ThreadTools.sleep(1000);
   }

   private void setupCamera(SimulationConstructionSet scs)
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void applyPushAndCheckFinalState(StateTransitionCondition pushCondition, double delay, Vector3D forceDirection,
                                            double magnitude, double pushDuration, double simulationDuration) throws SimulationExceededMaximumTimeException
   {
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, pushDuration);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationDuration));
      assertTrue(currentHighLevelState.getEnumValue().equals(HighLevelControllerName.WALKING));
   }

   private class PushRecoveryTransferStartCondition implements StateTransitionCondition
   {
      private final YoEnum<PushRecoveryStateEnum> pushRecoveryState;

      public PushRecoveryTransferStartCondition(YoEnum<PushRecoveryStateEnum> pushRecoveryState)
      {
         this.pushRecoveryState = pushRecoveryState;
      }

      @Override
      public boolean testCondition(double time)
      {
         return pushRecoveryState.getEnumValue() == PushRecoveryStateEnum.TO_PUSH_RECOVERY_LEFT_SUPPORT
                || pushRecoveryState.getEnumValue() == PushRecoveryStateEnum.TO_PUSH_RECOVERY_RIGHT_SUPPORT;
      }
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<ConstraintType> footConstraintType;

      public SingleSupportStartCondition(YoEnum<ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean testCondition(double time)
      {
         return footConstraintType.getEnumValue() == ConstraintType.SWING;
      }
   }

   private class DoubleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<WalkingStateEnum> walkingState;

      private final RobotSide side;

      public DoubleSupportStartCondition(YoEnum<WalkingStateEnum> walkingState, RobotSide side)
      {
         this.walkingState = walkingState;
         this.side = side;
      }

      @Override
      public boolean testCondition(double time)
      {
         if (side == RobotSide.LEFT)
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_LEFT_SUPPORT);
         }
         else
         {
            return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_RIGHT_SUPPORT);
         }
      }
   }

   public double getPushPositionZHeightInChestFrame()
   {
      return 0.3;
   }

   public String getScriptFilePath()
   {
     return defaultScript;
   }

   public double getPushWhileInSwingDelayMultiplier()
   {
      return 0.5;
   }
}
