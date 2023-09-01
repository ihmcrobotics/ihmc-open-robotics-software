package us.ihmc.avatar.pushRecovery;

import java.io.InputStream;
import java.util.concurrent.atomic.AtomicBoolean;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
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
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import static org.junit.jupiter.api.Assertions.*;

public abstract class AvatarPushRecoveryStandingTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private static String defaultScript = "scripts/ExerciseAndJUnitScripts/walkingPushTestScript.xml";

   private SCS2AvatarTestingSimulation simulationTestHelper;

   private static double simulationTime = 6.0;

   private PushRobotControllerSCS2 pushRobotController;

   private double magnitude;

   private SideDependentList<StateTransitionCondition> singleSupportStartConditions = new SideDependentList<>();

   private SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();

   private YoEnum<HighLevelControllerName> currentHighLevelState;

   public void setMagnitude(double magnitude)
   {
      this.magnitude = magnitude;
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      magnitude = Double.NaN;
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      singleSupportStartConditions = null;
      doubleSupportStartConditions = null;
      pushRobotController = null;
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   protected abstract DRCRobotModel getRobotModel();

   @Test
   public void testPushWhileStanding()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));

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
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));

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
   public void testLongForwardPushWhileStanding()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));

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
   public void testControllerFailureKicksIn()
   {
      setupTest(null, false);
      assertTrue(simulationTestHelper.simulateNow(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double magnitude = 160.0;
      double duration = 3.0;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      assertFalse(simulationTestHelper.simulateNow(duration + 2.0));
   }

   @Test
   public void testLongBackwardPushWhileStanding()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));

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
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      forceDirection.normalize();
      double duration = 1.0;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, duration + 2.0);
   }

   @Test
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double duration = 1.0;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, duration + 2.0);
   }

   @Test
   public void testRecoveryWhileInFlamingoStance()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(simulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide,
                                                                                                                                          LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      simulationTestHelper.publishToController(footPosePacket);
      assertTrue(simulationTestHelper.simulateNow(2.0));

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
   public void testRecoveryForwardWhileInFlamingoStance()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(simulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide,
                                                                                                                                          LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      simulationTestHelper.publishToController(footPosePacket);
      assertTrue(simulationTestHelper.simulateNow(2.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double magnitude = 275.0;
      double duration = 0.2;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, 4.0);
   }

   @Test
   public void testRecoverySidewaysWhileInFlamingoStance()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(simulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide,
                                                                                                                                          LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      simulationTestHelper.publishToController(footPosePacket);
      assertTrue(simulationTestHelper.simulateNow(2.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double magnitude = 200.0;
      double duration = 0.2;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, 4.0);
   }

   public double getAngledPushMagnitude()
   {
      return 350.0;
   }

   @Test
   public void testRecoveryAngledWhileInFlamingoStance()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(simulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide,
                                                                                                                                          LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      simulationTestHelper.publishToController(footPosePacket);
      assertTrue(simulationTestHelper.simulateNow(3.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 1.0, 0.0);
      double magnitude = getAngledPushMagnitude();
      double duration = 0.2;
      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, 2.5);
   }

   @Test
   public void testRecoveryPushForwardWhileInFlamingoStanceAndAfterTouchDown()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(simulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide,
                                                                                                                                          LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      simulationTestHelper.publishToController(footPosePacket);
      assertTrue(simulationTestHelper.simulateNow(2.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 0.2, 0.0);
      double magnitude = 300.0;
      double duration = 0.2;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
      assertTrue(simulationTestHelper.simulateNow(1.0));

      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, 0.7 * magnitude, duration);
      assertTrue(simulationTestHelper.simulateNow(4.0));
   }

   @Test
   public void testFailureAfterRecoveryStep()
   {
      setupTest(null, true);
      assertTrue(simulationTestHelper.simulateNow(1.0));
      RobotSide footSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(simulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide,
                                                                                                                                          LimbName.LEG));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      footPose.prependTranslation(0.0, 0.0, 0.2);
      Point3D desiredFootPosition = new Point3D();
      Quaternion desiredFootOrientation = new Quaternion();
      footPose.get(desiredFootPosition, desiredFootOrientation);
      FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
      simulationTestHelper.publishToController(footPosePacket);
      assertTrue(simulationTestHelper.simulateNow(3.0));

      // push timing:
      StateTransitionCondition pushCondition = null;
      double delay = 0.0;

      // push parameters:
      Vector3D forceDirection = new Vector3D(1.0, 1.0, 0.0);
      double magnitude = 250.0;
      double duration = 0.3;
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);

      @SuppressWarnings("unchecked")
      final YoEnum<PushRecoveryStateEnum> pushRecoveryState = (YoEnum<PushRecoveryStateEnum>) simulationTestHelper.findVariable("PushRecoveryHighLevelHumanoidController",
                                                                                                                                "pushRecoveryCurrentState");
      final YoInteger numberOfRecoveryStepsTaken = (YoInteger) simulationTestHelper.findVariable("numberOfRecoveryStepsTaken");
      final YoInteger maxNumberOfSteps = (YoInteger) simulationTestHelper.findVariable("maxNumberOfRecoveryStepsToTake");
      maxNumberOfSteps.set(1);

      AtomicBoolean tookStep = new AtomicBoolean(false);
      pushRecoveryState.addListener(state ->
      {
         if (pushRecoveryState.getEnumValue() == PushRecoveryStateEnum.TO_PUSH_RECOVERY_LEFT_SUPPORT
               || pushRecoveryState.getEnumValue() == PushRecoveryStateEnum.TO_PUSH_RECOVERY_RIGHT_SUPPORT)
            tookStep.set(true);
      });
      pushRobotController.queueForceDelayed(new PushRecoveryTransferStartCondition(pushRecoveryState), 0.02, forceDirection, magnitude, duration);

      assertFalse(simulationTestHelper.simulateNow(4.0));

      assertEquals(numberOfRecoveryStepsTaken.getIntegerValue(), 2);
      assertTrue(tookStep.get());

      //      applyPushAndCheckFinalState(pushCondition, delay, forceDirection, magnitude, duration, 2.5);
   }

   @SuppressWarnings("unchecked")
   private void setupTest(String scriptName, boolean enablePushRecoveryOnFailure)
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(getRobotModel(), flatGround, simulationTestingParameters);
      simulationTestHelper.start();
      FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
      //      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel);
      pushRobotController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationConstructionSet().getTime(),
                                                        simulationTestHelper.getRobot(),
                                                        fullRobotModel.getChest().getParentJoint().getName(),
                                                        new Vector3D(0, 0, getPushPositionZHeightInChestFrame()));
      simulationTestHelper.addYoGraphicDefinition(pushRobotController.getForceVizDefinition());

      if (scriptName != null && !scriptName.isEmpty())
      {
         assertTrue(simulationTestHelper.simulateNow(0.001));
         InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
         simulationTestHelper.loadScriptFile(scriptInputStream, ReferenceFrame.getWorldFrame());
      }

      // get rid of this once push recovery is enabled by default
      //      YoBoolean enable = (YoBoolean) scs.findVariable("PushRecoveryControlModule", "enablePushRecovery");
      //      enable.set(enablePushRecoveryControlModule);
      YoBoolean enableOnFailure = (YoBoolean) simulationTestHelper.findVariable(WalkingHighLevelHumanoidController.class.getSimpleName(),
                                                                                "enablePushRecoveryOnFailure");
      enableOnFailure.set(enablePushRecoveryOnFailure);
      currentHighLevelState = (YoEnum<HighLevelControllerName>) simulationTestHelper.findVariable("highLevelControllerNameCurrentState");

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) simulationTestHelper.findVariable(sidePrefix + "FootControlModule",
                                                                                                                      footPrefix + "CurrentState");
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable("WalkingHighLevelHumanoidController",
                                                                                                                    "walkingCurrentState");
         singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      setupCamera();
      ThreadTools.sleep(1000);
   }

   private void setupCamera()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private void applyPushAndCheckFinalState(StateTransitionCondition pushCondition,
                                            double delay,
                                            Vector3D forceDirection,
                                            double magnitude,
                                            double pushDuration,
                                            double simulationDuration)
        
   {
      pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, pushDuration);
      assertTrue(simulationTestHelper.simulateNow(simulationDuration));
      assertTrue(currentHighLevelState.getEnumValue().equals(HighLevelControllerName.WALKING), "Not back to walking");
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
