package us.ihmc.avatar.pushRecovery;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

@Tag("video")
public abstract class AvatarPushRecoveryWalkingTrackTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;

   private static final double standingTimeDuration = 1.0;
   private static final double defaultWalkingTimeDuration = 30.0;
   private static final boolean useVelocityAndHeadingScript = true;

   private static final double pushDuration = 0.05;

   private SideDependentList<AtomicInteger> footstepsCompletedPerSide;
   private SideDependentList<StateTransitionCondition> swingStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> swingFinishConditions = new SideDependentList<>();

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
      footstepsCompletedPerSide = null;
      swingStartConditions = null;
      swingFinishConditions = null;

      if (avatarSimulation != null)
      {
         avatarSimulation.dispose();
         avatarSimulation = null;
      }

      if (robotVisualizer != null)
      {
         robotVisualizer.close();
         robotVisualizer = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Override
   public abstract DRCRobotModel getRobotModel();

   public abstract double getForwardPushDeltaV();

   public abstract double getOutwardPushDeltaV();

   public abstract double getBackwardPushDeltaV();

   public abstract double getInwardPushDeltaV();

   public abstract double getForwardPushInTransferDeltaV();

   public abstract double getOutwardPushInTransferDeltaV();

   public abstract double getBackwardPushInTransferDeltaV();

   public abstract double getInwardPushInTransferDeltaV();

   @Tag("humanoid-flat-ground")
   @Test
   public void testFlatGroundWalking()
   {
      DRCRobotModel robotModel = getRobotModel();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      //      simulationTestingParameters.setUsePefectSensors(getUsePerfectSensors());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             flatGround,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setDefaultHighLevelHumanoidControllerFactory(useVelocityAndHeadingScript, new HeadingAndVelocityEvaluationScriptParameters());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupStateMonitors();

      ((YoBoolean) simulationTestHelper.findVariable("controllerAllowStepAdjustment")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("controllerAllowCrossOverSteps")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("stepsAreAdjustableCSG")).set(true);
      ((YoBoolean) simulationTestHelper.findVariable("shiftUpcomingStepsWithTouchdownCSG")).set(true);

      PushRobotControllerSCS2 pushRobotController = new PushRobotControllerSCS2(simulationTestHelper.getSimulationConstructionSet().getTime(),
                                                                                simulationTestHelper.getRobot(),
                                                                                simulationTestHelper.getControllerFullRobotModel());
      simulationTestHelper.addYoGraphicDefinition(pushRobotController.getForceVizDefinition());

      setupCameraForUnitTest();
      simulateAndAssertGoodWalking(simulationTestHelper, pushRobotController);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void simulateAndAssertGoodWalking(SCS2AvatarTestingSimulation simulationTestHelper, PushRobotControllerSCS2 pushRobotController)
   {
      YoBoolean walk = (YoBoolean) simulationTestHelper.findVariable("walkCSG");
      YoDouble desiredXVelocity = (YoDouble) simulationTestHelper.findVariable("scriptDesiredVelocityRateLimitedX");
      YoDouble desiredYVelocity = (YoDouble) simulationTestHelper.findVariable("scriptDesiredVelocityRateLimitedY");
      YoDouble desiredTurningVelocity = (YoDouble) simulationTestHelper.findVariable("scriptDesiredTurningVelocityRateLimited");

      assertTrue(simulationTestHelper.simulateNow(standingTimeDuration));

      walk.set(true);

      Random random = new Random(1738L);

      double stepsToTakeWithEachFoot = 3;
      simulationTestHelper.addSimulationTerminalCondition(() ->
      {
         return footstepsCompletedPerSide.get(RobotSide.LEFT).get() >= stepsToTakeWithEachFoot
               && footstepsCompletedPerSide.get(RobotSide.RIGHT).get() >= stepsToTakeWithEachFoot;
      });

      while (simulationTestHelper.getSimulationTime() - standingTimeDuration < defaultWalkingTimeDuration)
      {
         for (RobotSide robotSide : RobotSide.values)
            footstepsCompletedPerSide.get(robotSide).set(0);

         boolean pushInSwing = RandomNumbers.nextBoolean(random, 0.75);
         if (pushInSwing)
         {
            Vector2D pushDirection = EuclidCoreRandomTools.nextVector2D(random);
            pushDirection.normalize();
            Vector3D forceDirection = new Vector3D(pushDirection);

            RobotSide swingSideForPush = RandomNumbers.nextBoolean(random, 0.5) ? RobotSide.LEFT : RobotSide.RIGHT;

            // queue push
            double totalMass = getRobotModel().createFullRobotModel().getTotalMass();
            double pushMagnitude = getPushDeltaVForMidSwing(pushDirection,
                                                            new Vector2D(desiredXVelocity.getDoubleValue(), desiredYVelocity.getDoubleValue()),
                                                            desiredTurningVelocity.getDoubleValue(),
                                                            swingSideForPush)
                  / pushDuration * totalMass;

            double percentOfState = 0.5;
            double delay = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime() * percentOfState;

            pushRobotController.applyForceDelayed(swingStartConditions.get(swingSideForPush), delay, forceDirection, pushMagnitude, pushDuration);
         }
         else
         {
            Vector2D pushDirection = EuclidCoreRandomTools.nextVector2D(random);
            pushDirection.normalize();
            Vector3D forceDirection = new Vector3D(pushDirection);

            RobotSide swingSideForPush = RandomNumbers.nextBoolean(random, 0.5) ? RobotSide.LEFT : RobotSide.RIGHT;

            // queue push
            double totalMass = getRobotModel().createFullRobotModel().getTotalMass();
            double pushMagnitude = getPushDeltaVForTransfer(pushDirection,
                                                            new Vector2D(desiredXVelocity.getDoubleValue(), desiredYVelocity.getDoubleValue()),
                                                            desiredTurningVelocity.getDoubleValue(),
                                                            swingSideForPush)
                  / pushDuration * totalMass;

            double percentOfState = 0.5;
            double delay = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime() * percentOfState;

            pushRobotController.applyForceDelayed(swingFinishConditions.get(swingSideForPush), delay, forceDirection, pushMagnitude, pushDuration);
         }

         LogTools.info("Resume simulation!");
         // Simulate until reaching the desired number of footstep, see terminal condition setup before this loop.
         boolean success = simulationTestHelper.simulateNow();
         LogTools.info("Simulation has stopped: success={}", success);
         assertTrue(success);
      }
   }

   private void setupStateMonitors()
   {
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
         final YoEnum<FootControlModule.ConstraintType> footConstraintType = (YoEnum<FootControlModule.ConstraintType>) simulationTestHelper.findVariable(sidePrefix
               + "FootControlModule", footPrefix + "CurrentState");
         @SuppressWarnings("unchecked")
         final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable("WalkingHighLevelHumanoidController",
                                                                                                                    "walkingCurrentState");
         swingStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         swingFinishConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }
   }

   private double getPushDeltaVForMidSwing(Vector2DReadOnly pushDirection, Vector2DReadOnly desiredVelocity, double desiredTurningVelocity, RobotSide pushSide)
   {
      // TODO scale this based on yaw velocity, too
      Vector2D direction = new Vector2D(pushDirection);
      Vector2D velocityDirection = new Vector2D(desiredVelocity);
      direction.normalize();
      if (velocityDirection.lengthSquared() > 0.0)
         velocityDirection.normalize();

      double xMax;
      double yMax;
      if (direction.getX() > 0.0)
         xMax = getForwardPushDeltaV();
      else
         xMax = getBackwardPushDeltaV();
      if (pushSide.negateIfRightSide(direction.getY()) > 0.0)
         yMax = getOutwardPushDeltaV();
      else
         yMax = getInwardPushDeltaV();

      yMax = InterpolationTools.linearInterpolate(yMax, 0.0, MathTools.clamp(pushSide.negateIfLeftSide(desiredTurningVelocity) / 0.5, 0.0, 1.0));
      double magnitudeAlongEllipse = Math.sqrt(1.0 / (MathTools.square(direction.getX() / xMax) + MathTools.square(direction.getY() / yMax)));

      double dotValue = pushDirection.dot(desiredVelocity);
      double magnitudeScaler = InterpolationTools.linearInterpolate(1.5, 0.5, (dotValue + 1.0) / 2.0);

      return magnitudeScaler * magnitudeAlongEllipse;
   }

   private double getPushDeltaVForTransfer(Vector2DReadOnly pushDirection, Vector2DReadOnly desiredVelocity, double desiredTurningVelocity, RobotSide pushSide)
   {
      // TODO scale this based on yaw velocity, too
      Vector2D direction = new Vector2D(pushDirection);
      Vector2D velocityDirection = new Vector2D(desiredVelocity);
      direction.normalize();
      if (velocityDirection.lengthSquared() > 0.0)
         velocityDirection.normalize();

      double xMax;
      double yMax;
      if (direction.getX() > 0.0)
         xMax = getForwardPushInTransferDeltaV();
      else
         xMax = getBackwardPushInTransferDeltaV();
      if (pushSide.negateIfRightSide(direction.getY()) > 0.0)
         yMax = getOutwardPushInTransferDeltaV();
      else
         yMax = getInwardPushInTransferDeltaV();

      yMax = InterpolationTools.linearInterpolate(yMax, 0.0, Math.min(Math.abs(desiredTurningVelocity) / 0.5, 1.0));
      double magnitudeAlongEllipse = Math.sqrt(1.0 / (MathTools.square(direction.getX() / xMax) + MathTools.square(direction.getY() / yMax)));

      double dotValue = pushDirection.dot(desiredVelocity);
      double magnitudeScaler = InterpolationTools.linearInterpolate(1.5, 0.5, (dotValue + 1.0) / 2.0);

      return magnitudeScaler * magnitudeAlongEllipse;
   }

   private AvatarSimulation avatarSimulation;
   private RobotVisualizer robotVisualizer;

   private void setupCameraForUnitTest()
   {
      simulationTestHelper.setCameraFocusPosition(0.6, 0.4, 1.1);
      simulationTestHelper.setCameraPosition(-0.15, 10.0, 3.0);
   }

   public SimulationTestingParameters getSimulationTestingParameters()
   {
      return simulationTestingParameters;
   }

   private static class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final YoEnum<FootControlModule.ConstraintType> footConstraintType;

      public SingleSupportStartCondition(YoEnum<FootControlModule.ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean testCondition(double time)
      {
         return footConstraintType.getEnumValue() == FootControlModule.ConstraintType.SWING;
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
