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
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public abstract class AvatarQuickPushRecoveryWalkingTest implements MultiRobotTestInterface
{
   private static final double pushDuration = 0.05;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   static
   {
      simulationTestingParameters.setRunMultiThreaded(false);
   }

   private SCS2AvatarTestingSimulation simulationTestHelper;

   private double swingTime;
   private double transferTime;

   private Double pushChangeInVelocity;

   private SideDependentList<AtomicInteger> footstepsCompletedPerSide;
   private SideDependentList<StateTransitionCondition> swingStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> swingFinishConditions = new SideDependentList<>();
   private PushRobotControllerSCS2 pushRobotController;

   private boolean usePerfectSensors = false;

   private double pushHeightOffsetFromChest = 0.3;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      pushChangeInVelocity = null;
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

      pushChangeInVelocity = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void setPushChangeInVelocity(double pushChangeInVelocity)
   {
      this.pushChangeInVelocity = pushChangeInVelocity;
   }

   @Test
   public void testInwardPushLeftAtDifferentTimes()
   {
      setupTest();
      enableSpeedUp();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      RobotSide side = RobotSide.LEFT;

      walkForward(16);

      // apply the push at mid swing
      testPush(forceDirection, pushChangeInVelocity, 0.45, swingStartConditions.get(side), swingTime, 4);

      // apply the push at early swing
      testPush(forceDirection, pushChangeInVelocity, 0.2, swingStartConditions.get(side), swingTime, 4);

      // apply the push at late swing
      testPush(forceDirection, pushChangeInVelocity, 0.7, swingStartConditions.get(side), swingTime, 4);
   }

   @Test
   public void testOutwardPushLeftSwingAtDifferentTimes()
   {
      setupTest();
      enableSpeedUp();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
      RobotSide side = RobotSide.LEFT;

      walkForward(16);

      // apply the push at mid swing
      testPush(forceDirection, pushChangeInVelocity, 0.45, swingStartConditions.get(side), swingTime, 4);

      // apply the push at early swing
      testPush(forceDirection, pushChangeInVelocity, 0.2, swingStartConditions.get(side), swingTime, 4);

      // apply the push at late swing
      testPush(forceDirection, 0.6 * pushChangeInVelocity, 0.65, swingStartConditions.get(side), swingTime, 4);
   }

   @Test
   public void testPushOutwardInRightThenLeftMidSwing()
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double percentInSwing = 0.4;
      RobotSide side = RobotSide.RIGHT;

      walkForward();

      StateTransitionCondition condition = time -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;
      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, condition, swingTime, 4);

      // push the robot again with new parameters
      forceDirection = new Vector3D(0.0, 1.0, 0.0);

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, swingStartConditions.get(RobotSide.LEFT), swingTime, 4);
   }

   @Test
   public void testBackwardPushInLeftSwingAtDifferentTimes()
   {
      setupTest();
      enableSpeedUp();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
      RobotSide side = RobotSide.LEFT;
      
      walkForward(16);

      StateTransitionCondition condition = time -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;

      // apply the push at mid swing
      testPush(forceDirection, pushChangeInVelocity, 0.45, condition, swingTime, 4);
      simulationTestHelper.simulateNow(1.0);
      
      // apply the push at early swing
      testPush(forceDirection, pushChangeInVelocity, 0.2, condition, swingTime, 4);
      simulationTestHelper.simulateNow(1.0);

      // apply the push at late swing
      testPush(forceDirection, pushChangeInVelocity, 0.7, condition, swingTime, 4);
      simulationTestHelper.simulateNow(1.0);
   }

   @Test
   public void testForwardPushInLeftSwingAtDifferentTimes()
   {
      setupTest();
      enableSpeedUp();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
      RobotSide side = RobotSide.LEFT;

      walkForward(20);

      StateTransitionCondition condition = time -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, 0.5, condition, swingTime, 3);

      // reset and queue an early push
      footstepsCompletedPerSide.get(side).set(0);
      condition = time -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;

      testPush(forceDirection, pushChangeInVelocity, 0.1, condition, swingTime, 4);

      // reset and queue a late push
      for (RobotSide robotSide : RobotSide.values)
         footstepsCompletedPerSide.get(robotSide).set(0);

      condition = time -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 0;

      testPush(forceDirection, 0.7 * pushChangeInVelocity, 0.8, condition, swingTime, 6);
   }

   @Test
   public void testForwardAndOutwardPushInLeftSwing()
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(1.0, 0.5, 0.0);
      double percentInSwing = 0.4;
      RobotSide side = RobotSide.LEFT;

      walkForward();

      StateTransitionCondition condition = time -> swingStartConditions.get(side).testCondition(time) && footstepsCompletedPerSide.get(side).get() > 1;

      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInSwing, condition, swingTime, 7);
   }

   @Test
   public void testOutwardPushInitialTransferToLeftStateAndLeftMidSwing()
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
      double percentInTransferState = 0.5;
      RobotSide side = RobotSide.LEFT;

      walkForward();

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
   public void testOutwardPushTransferToLeftState()
   {
      setupTest();

      // setup all parameters
      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);

      // This doesn't work for 0.5 or higher. We need to do more development to get this working better. So for now, just stick with 0.4.
      // 0.9 * Math.random();
      double percentInTransferState = 0.4;
      RobotSide side = RobotSide.LEFT;

      walkForward();

      StateTransitionCondition condition = time -> swingFinishConditions.get(side).testCondition(time)
            && footstepsCompletedPerSide.get(side.getOppositeSide()).get() > 0;
      // apply the push
      testPush(forceDirection, pushChangeInVelocity, percentInTransferState, condition, transferTime, 6);
   }
   
   public void setPushHeightOffsetFromChest(double zHeight)
   {
      pushHeightOffsetFromChest = zHeight;
   }

   private void setupTest()
   {
      DRCRobotModel robotModel = getRobotModel();
      swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel, flatGround, simulationTestingParameters);
      factory.setUsePerfectSensors(usePerfectSensors);
      simulationTestHelper = factory.createAvatarTestingSimulation();
      
      simulationTestHelper.start();
      ((YoBoolean) simulationTestHelper.findVariable("speedUpTransferDynamicsFromError")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("controllerAllowStepAdjustment")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("controllerSwingSpeedUpEnabled")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("leftFootSwingIsSpeedUpEnabled")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("rightFootSwingIsSpeedUpEnabled")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("controllerAllowCrossOverSteps")).set(true);
      ((YoDouble) simulationTestHelper.findVariable("icpDistanceFromFootPolygonThreshold")).set(0.25);
      ((YoDouble) simulationTestHelper.findVariable("controllerminimumTimeForStepAdjustment")).set(-3.0);
      

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      
      YoDouble time = simulationTestHelper.getSimulationConstructionSet().getTime();
      Robot robot = simulationTestHelper.getRobot();
      String jointBeforeSpine = fullRobotModel.getChest().getParentJoint().getName();
      Vector3D forcePointOffset = new Vector3D(0, 0, pushHeightOffsetFromChest);
      pushRobotController = new PushRobotControllerSCS2(time, robot, jointBeforeSpine, forcePointOffset, 0.005);
      
      
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

   public void setUsePerfectSensors(boolean usePerfectSensors)
   {
      this.usePerfectSensors = usePerfectSensors;
   }

   public void enableSpeedUp()
   {
      ((YoBoolean) simulationTestHelper.findVariable("speedUpTransferDynamicsFromError")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("speedUpSwingDynamicsFromError")).set(true);
   }
   
   private void walkForward()
   {
      walkForward(10);
   }

   private void walkForward(int steps)
   {
      double stepLength = 0.3;
      double stepWidth = 0.14;

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
         footstepData.setShouldCheckForReachability(true);
         footsteps.getFootstepDataList().add().set(footstepData);
      }

      footsteps.setAreFootstepsAdjustable(true);
      footsteps.setOffsetFootstepsWithExecutionError(true);
      simulationTestHelper.publishToController(footsteps);
   }

   private void testPush(Vector3D forceDirection,
                         double velocityChange,
                         double percentInState,
                         StateTransitionCondition condition,
                         double stateTime,
                         int stepsToTake)
   {
      double totalMass = getRobotModel().createFullRobotModel().getTotalMass();
      double pushMagnitude = velocityChange / pushDuration * totalMass;

      double delay = stateTime * percentInState;
      pushRobotController.applyForceDelayed(condition, delay, forceDirection, pushMagnitude, pushDuration);

      double simulationDuration = getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime() + swingTime
            + (swingTime + transferTime) * stepsToTake;
      assertTrue(simulationTestHelper.simulateNow(simulationDuration));
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
