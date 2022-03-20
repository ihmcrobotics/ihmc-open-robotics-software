package us.ihmc.avatar.pushRecovery;

import static us.ihmc.robotics.Assert.assertTrue;

import controller_msgs.msg.dds.FootstepStatusMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.concurrent.atomic.AtomicInteger;

public abstract class AvatarQuickPushRecoveryWalkingTest implements MultiRobotTestInterface
{
   private static final double pushDuration = 0.05;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private double swingTime;
   private double transferTime;

   private Double pushChangeInVelocity;
   private double mass;

   private SideDependentList<AtomicInteger> footstepsCompletedPerSide;
   private SideDependentList<StateTransitionCondition> swingStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> swingFinishConditions = new SideDependentList<>();
   private PushRobotController pushRobotController;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      pushChangeInVelocity = null;
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
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      pushChangeInVelocity = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void setPushChangeInVelocity(double pushChangeInVelocity)
   {
      this.pushChangeInVelocity = pushChangeInVelocity;
   }

   @Test
   public void testOutwardPushLeftEarlySwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double percentInSwing = 0.2;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, swingStartConditions.get(side), swingTime, 6);
   }

   @Test
   public void testInwardPushLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double percentInSwing = 0.5;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, swingStartConditions.get(side), swingTime, 6);
   }

   @Test
   public void testOutwardPushMidLeftSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double percentInSwing = 0.5;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, swingStartConditions.get(side), swingTime, 6);
   }

   @Test
   public void testPushOutwardInRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double percentInSwing = 0.4;
      RobotSide side = RobotSide.RIGHT;

      StateTransitionCondition condition = time -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;
      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, condition, swingTime, 4);

      // push the robot again with new parameters
      forceDirection = new Vector3D(0.0, 1.0, 0.0);

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, swingStartConditions.get(RobotSide.LEFT), swingTime, 4);
   }

   @Test
   public void testBackwardPushInLeftSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double percentInSwing = 0.2;
      RobotSide side = RobotSide.LEFT;

      StateTransitionCondition condition = time -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, condition, swingTime, 6);
   }

   @Test
   public void testForwardPushInLeftSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double percentInSwing = 0.4;
      RobotSide side = RobotSide.LEFT;

      StateTransitionCondition condition = time -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 1;

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, condition, swingTime, 7);
   }

   @Test
   public void testForwardAndOutwardPushInLeftSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(1.0, 0.5, 0.0);
      double percentInSwing = 0.4;
      RobotSide side = RobotSide.LEFT;

      StateTransitionCondition condition = time -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 1;

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, condition, swingTime, 7);
   }


   @Test
   public void testOutwardPushInitialTransferToLeftStateAndLeftMidSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double percentInTransferState = 0.5;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInTransferState, swingFinishConditions.get(side), transferTime, 4);

      // push the robot again with new parameters
      forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double percentInSwing = 0.4;
      side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, swingStartConditions.get(side), swingTime, 4);
   }

   @Test
   public void testOutwardPushTransferToLeftState() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);

      // This doesn't work for 0.5 or higher. We need to do more development to get this working better. So for now, just stick with 0.4.
      // 0.9 * Math.random();
      double percentInTransferState = 0.4;
      RobotSide side = RobotSide.LEFT;

      StateTransitionCondition condition = time -> swingFinishConditions.get(side).testCondition(time) &&
                                                   footstepsCompletedPerSide.get(side.getOppositeSide()).get() > 0;
      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInTransferState, condition, transferTime, 6);
   }

   private void setupTest() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");
      ((YoBoolean) drcSimulationTestHelper.getYoVariable("controllerAllowStepAdjustment")).set(true);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addYoGraphic(pushRobotController.getForceVisualizer());
      drcSimulationTestHelper.setupCameraForUnitTest(new Point3D(0.6, 0.0, 0.6), new Point3D(10.0, 3.0, 3.0));

      footstepsCompletedPerSide = new SideDependentList<>(new AtomicInteger(), new AtomicInteger());
      drcSimulationTestHelper.createSubscriberFromController(FootstepStatusMessage.class, m ->
      {
         if (FootstepStatus.fromByte(m.getFootstepStatus()) == FootstepStatus.COMPLETED)
         {
            footstepsCompletedPerSide.get(RobotSide.fromByte(m.getRobotSide())).incrementAndGet();
         }
      });

      // get YoVariables
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         String footPrefix = sidePrefix + "Foot";
         @SuppressWarnings("unchecked") final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) scs.findVariable(
               sidePrefix + "FootControlModule", footPrefix + "CurrentState");
         @SuppressWarnings("unchecked") final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.findVariable(
               "WalkingHighLevelHumanoidController",
               "walkingCurrentState");
         swingStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         swingFinishConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      //      YoBoolean enable = (YoBoolean) scs.findVariable("PushRecoveryControlModule", "enablePushRecovery");
      //      enable.set(true);
   }

   private void walkForward() throws SimulationExceededMaximumTimeException
   {
      double stepLength = 0.3;
      double stepWidth = 0.14;
      int steps = 10;

      ReferenceFrame pelvisFrame = drcSimulationTestHelper.getControllerFullRobotModel().getPelvis().getBodyFixedFrame();

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage();
      for (int i = 1; i <= steps; i++)
      {
         RobotSide robotSide = i % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double footstepY = robotSide == RobotSide.LEFT ? stepWidth : -stepWidth;
         double footstepX = stepLength * i;
         FramePoint3D location = new FramePoint3D(pelvisFrame, footstepX, footstepY, 0.0);
         location.changeFrame(ReferenceFrame.getWorldFrame());
         location.setZ(0.0);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
         footsteps.getFootstepDataList().add().set(footstepData);
      }

      footsteps.setAreFootstepsAdjustable(true);
      drcSimulationTestHelper.publishToController(footsteps);
   }

   private void testPush(Vector3D forceDirection,
                         double velocityChange,
                         double percentInState,
                         StateTransitionCondition condition,
                         double stateTime,
                         int stepsToTake) throws SimulationExceededMaximumTimeException
   {
      walkForward();

      double totalMass = getRobotModel().createFullRobotModel().getTotalMass();
      double pushMagnitude = velocityChange / pushDuration * totalMass;

      double delay = stateTime * percentInState;
      pushRobotController.applyForceDelayed(condition, delay, forceDirection, pushMagnitude, pushDuration);

      double simulationDuration = getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime() + swingTime +
                                  (swingTime + transferTime) * stepsToTake;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationDuration));
   }

   private static class SingleSupportStartCondition implements StateTransitionCondition
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

   private static class DoubleSupportStartCondition implements StateTransitionCondition
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
}
