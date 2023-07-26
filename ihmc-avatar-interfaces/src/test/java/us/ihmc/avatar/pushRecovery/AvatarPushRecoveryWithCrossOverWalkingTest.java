package us.ihmc.avatar.pushRecovery;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.MathTools;
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
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarPushRecoveryWithCrossOverWalkingTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private SCS2AvatarTestingSimulation simulationTestHelper;

   private double swingTime;
   private double transferTime;

   private SideDependentList<AtomicInteger> footstepsCompletedPerSide;
   private SideDependentList<StateTransitionCondition> swingStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> swingFinishConditions = new SideDependentList<>();
   private PushRobotControllerSCS2 pushRobotController;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public abstract double getForwardPushDelta();

   public abstract double getOutwardPushDelta();

   public abstract double getBackwardPushDelta();

   public abstract double getInwardPushDelta();

   @Test
   public void testInwardPushInSwing()
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      double pushDuration = 0.8 * swingTime;
      double percentInSwing = 0.1;
      RobotSide side = RobotSide.RIGHT;

      StateTransitionCondition condition = (time) -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;

      // apply the push
      testPush(forceDirection, getPushDelta(forceDirection, side), percentInSwing, pushDuration, condition, swingTime, 8);
   }

   @Test
   public void testOutwardPushInSwing()
   {
      simulationTestingParameters.setKeepSCSUp(true);
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double pushDuration = 0.4 * swingTime;
      double percentInSwing = 0.1;
      RobotSide side = RobotSide.RIGHT;

      StateTransitionCondition condition = (time) -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;

      // apply the push
      testPush(forceDirection, getPushDelta(forceDirection, side), percentInSwing, pushDuration, condition, swingTime, 8);
   }

   @Test
   public void testForwardPushInSwing()
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      double pushDuration = 0.8 * swingTime;
      double percentInSwing = 0.1;
      RobotSide side = RobotSide.RIGHT;

      StateTransitionCondition condition = (time) -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;

      // apply the push
      testPush(forceDirection, getPushDelta(forceDirection, side), percentInSwing, pushDuration, condition, swingTime, 8);
   }

   @Test
   public void testBackwardPushInSwing()
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      double pushDuration = 0.8 * swingTime;
      double percentInSwing = 0.1;
      RobotSide side = RobotSide.RIGHT;

      StateTransitionCondition condition = (time) -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;

      // apply the push
      testPush(forceDirection, getPushDelta(forceDirection, side), percentInSwing, pushDuration, condition, swingTime, 8);
   }

   private void setupTest()
   {
      DRCRobotModel robotModel = getRobotModel();
      swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, flatGround, simulationTestingParameters);
      simulationTestHelper.start();
      ((YoBoolean) simulationTestHelper.findVariable("controllerAllowStepAdjustment")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("controllerAllowCrossOverSteps")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("speedUpSwingDynamicsFromError")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("speedUpTransferDynamicsFromError")).set(true);
      ((YoDouble) simulationTestHelper.findVariable("icpDistanceFromFootPolygonThreshold")).set(0.35);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      pushRobotController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationConstructionSet().getTime(), simulationTestHelper.getRobot(), fullRobotModel);
      simulationTestHelper.addYoGraphicDefinition(pushRobotController.getForceVizDefinition());
      simulationTestHelper.setCamera(new Point3D(0.6, 0.0, 0.6), new Point3D(10.0, 3.0, 3.0));

      footstepsCompletedPerSide = new SideDependentList<>(new AtomicInteger(), new AtomicInteger());
      simulationTestHelper.createSubscriberFromController(FootstepStatusMessage.class, m ->
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
         @SuppressWarnings("unchecked")
         final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) simulationTestHelper.findVariable(sidePrefix + "FootControlModule",
                                                                                                                      footPrefix + "CurrentState");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable("WalkingHighLevelHumanoidController",
                                                                                                                    "walkingCurrentState");
         swingStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         swingFinishConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }

      assertTrue(simulationTestHelper.simulateNow(1.0));
   }

   private void walkForward()
   {
      double stepLength = 0.3;
      double stepWidth = 0.14;
      int steps = 10;

      ReferenceFrame pelvisFrame = simulationTestHelper.getControllerFullRobotModel().getPelvis().getBodyFixedFrame();

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
      footsteps.setOffsetFootstepsWithExecutionError(true);
      simulationTestHelper.publishToController(footsteps);
   }

   private void testPush(Vector3D forceDirection,
                         double velocityChange,
                         double percentInState,
                         double pushDuration,
                         StateTransitionCondition condition,
                         double stateTime,
                         int stepsToSimulate)
   {
      walkForward();

      double totalMass = getRobotModel().createFullRobotModel().getTotalMass();
      double pushMagnitude = velocityChange / pushDuration * totalMass;

      double delay = stateTime * percentInState;
      pushRobotController.applyForceDelayed(condition, delay, forceDirection, pushMagnitude, pushDuration);

      double simulationDuration = getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime() + swingTime
            + (stepsToSimulate - 1) * (swingTime + transferTime);
      assertTrue(simulationTestHelper.simulateNow(simulationDuration));
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
