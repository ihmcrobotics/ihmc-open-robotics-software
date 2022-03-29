package us.ihmc.avatar.pushRecovery;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
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

import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AvatarLongPushRecoveryWalkingTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private double swingTime;
   private double transferTime;

   private SideDependentList<AtomicInteger> footstepsCompletedPerSide;
   private SideDependentList<StateTransitionCondition> swingStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> swingFinishConditions = new SideDependentList<>();
   private PushRobotController pushRobotController;

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
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public abstract double getForwardPushDelta();

   public abstract double getOutwardPushDelta();

   public abstract double getBackwardPushDelta();

   public abstract double getInwardPushDelta();

   @Test
   public void testInwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double pushDuration = 0.8 * swingTime;
      double percentInSwing = 0.1;
      RobotSide side = RobotSide.RIGHT;

      StateTransitionCondition condition = (time) -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 1;

      // apply the push
      testPush(forceDirection, getPushDelta(forceDirection, side), percentInSwing, pushDuration, condition, swingTime, 8);
   }

   @Test
   public void testOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double pushDuration = 0.8 * swingTime;
      double percentInSwing = 0.1;
      RobotSide side = RobotSide.RIGHT;

      StateTransitionCondition condition = (time) -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 1;

      // apply the push
      testPush(forceDirection, getPushDelta(forceDirection, side), percentInSwing, pushDuration, condition, swingTime, 8);
   }

   @Test
   public void testForwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double pushDuration = 0.8 * swingTime;
      double percentInSwing = 0.1;
      RobotSide side = RobotSide.RIGHT;

      StateTransitionCondition condition = (time) -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 1;

      // apply the push
      testPush(forceDirection, getPushDelta(forceDirection, side), percentInSwing, pushDuration, condition, swingTime, 8);
   }

   @Test
   public void testBackwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double pushDuration = 0.8 * swingTime;
      double percentInSwing = 0.1;
      RobotSide side = RobotSide.RIGHT;

      StateTransitionCondition condition = (time) -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 1;

      // apply the push
      testPush(forceDirection, getPushDelta(forceDirection, side), percentInSwing, pushDuration, condition, swingTime, 8);
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

   private void walkForward()
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
                         double pushDuration,
                         StateTransitionCondition condition,
                         double stateTime,
                         int stepsToSimulate) throws SimulationExceededMaximumTimeException
   {
      walkForward();

      double totalMass = getRobotModel().createFullRobotModel().getTotalMass();
      double pushMagnitude = velocityChange / pushDuration * totalMass;

      double delay = stateTime * percentInState;
      pushRobotController.applyForceDelayed(condition, delay, forceDirection, pushMagnitude, pushDuration);

      double simulationDuration = getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime() + swingTime
            + (stepsToSimulate - 1) * (swingTime + transferTime);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationDuration));
   }

   private double getPushDelta(Vector3DReadOnly pushDirection, RobotSide pushSide)
   {
      Vector3D direction = new Vector3D(pushDirection);
      direction.normalize();

      double xMax;
      double yMax;
      if (direction.getX() > 0.0)
         xMax = getForwardPushDelta();
      else
         xMax = getBackwardPushDelta();
      if (pushSide.negateIfRightSide(direction.getY()) > 0.0)
         yMax = getOutwardPushDelta();
      else
         yMax = getInwardPushDelta();

      double denom = MathTools.square(direction.getX() / xMax) + MathTools.square(direction.getY() / yMax);

      return Math.sqrt(1.0 / denom);
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
