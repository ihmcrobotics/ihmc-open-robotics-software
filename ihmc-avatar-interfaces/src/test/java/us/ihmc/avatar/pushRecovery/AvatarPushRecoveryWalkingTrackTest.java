package us.ihmc.avatar.pushRecovery;

import controller_msgs.msg.dds.FootstepStatusMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationDoneListener;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.Random;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

@Tag("video")
public abstract class AvatarPushRecoveryWalkingTrackTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private BlockingSimulationRunner blockingSimulationRunner;
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static final double standingTimeDuration = 1.0;
   private static final double defaultWalkingTimeDuration = 30.0;
   private static final boolean useVelocityAndHeadingScript = true;
   private static final boolean cheatWithGroundHeightAtForFootstep = false;

   private static final double pushDuration = 0.05;

   private SimulationConstructionSet scs;
   private SideDependentList<AtomicInteger> footstepsCompletedPerSide;
   private SideDependentList<StateTransitionCondition> swingStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> swingFinishConditions = new SideDependentList<>();
   private final AtomicBoolean simulationCrashed = new AtomicBoolean(false);

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      simulationCrashed.set(false);
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
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }
      scs = null;
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
   public void testFlatGroundWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      DRCRobotModel robotModel = getRobotModel();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      //      simulationTestingParameters.setUsePefectSensors(getUsePerfectSensors());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.setAddFootstepMessageGenerator(true);
      drcSimulationTestHelper.setUseHeadingAndVelocityScript(useVelocityAndHeadingScript);
      drcSimulationTestHelper.setCheatWithGroundHeightAtFootstep(cheatWithGroundHeightAtForFootstep);
      drcSimulationTestHelper.createSimulation(robotModel.getSimpleRobotName() + "FlatGroundWalking");

      scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.addSimulateDoneListener(new DoneListener());

      setupStateMonitors();

      ((YoBoolean) scs.findVariable("controllerAllowStepAdjustment")).set(true);
      ((YoBoolean) scs.findVariable("controllerAllowCrossOverSteps")).set(true);
      ((YoBoolean) scs.findVariable("stepsAreAdjustableCSG")).set(true);
      ((YoBoolean) scs.findVariable("shiftUpcomingStepsWithTouchdownCSG")).set(true);

      PushRobotController pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(),
                                                                        drcSimulationTestHelper.getControllerFullRobotModel());

      setupCameraForUnitTest(scs);
      simulateAndAssertGoodWalking(drcSimulationTestHelper, pushRobotController);

      if (simulationTestingParameters.getCheckNothingChangedInSimulation())
         drcSimulationTestHelper.checkNothingChanged();

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void simulateAndAssertGoodWalking(DRCSimulationTestHelper drcSimulationTestHelper, PushRobotController pushRobotController)
         throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      YoBoolean walk = (YoBoolean) scs.findVariable("walkCSG");
      YoDouble desiredXVelocity = (YoDouble) scs.findVariable("scriptDesiredVelocityRateLimitedX");
      YoDouble desiredYVelocity = (YoDouble) scs.findVariable("scriptDesiredVelocityRateLimitedY");
      YoDouble desiredTurningVelocity = (YoDouble) scs.findVariable("scriptDesiredTurningVelocityRateLimited");

      drcSimulationTestHelper.simulateAndBlock(standingTimeDuration);

      walk.set(true);

      Random random = new Random(1738L);

      while (!simulationCrashed.get() && scs.getTime() - standingTimeDuration < defaultWalkingTimeDuration)
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
                                                            swingSideForPush) / pushDuration * totalMass;

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
                                                            swingSideForPush) / pushDuration * totalMass;

            double percentOfState = 0.5;
            double delay = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime() * percentOfState;

            pushRobotController.applyForceDelayed(swingFinishConditions.get(swingSideForPush), delay, forceDirection, pushMagnitude, pushDuration);
         }

         scs.simulate();

         double stepsToTakeWithEachFoot = 3;
         while (!simulationCrashed.get() && footstepsCompletedPerSide.get(RobotSide.LEFT).get() < stepsToTakeWithEachFoot && footstepsCompletedPerSide.get(RobotSide.RIGHT).get() < stepsToTakeWithEachFoot)
         {
            Thread.yield();
         }

         scs.stop();
      }
   }

   private void stop()
   {
      scs.stop();
   }

   private void setupStateMonitors()
   {
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
         @SuppressWarnings("unchecked") final YoEnum<FootControlModule.ConstraintType> footConstraintType = (YoEnum<FootControlModule.ConstraintType>) scs.findVariable(
               sidePrefix + "FootControlModule", footPrefix + "CurrentState");
         @SuppressWarnings("unchecked") final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.findVariable(
               "WalkingHighLevelHumanoidController",
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

   private void setupCameraForUnitTest(SimulationConstructionSet scs)
   {
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");
      cameraConfiguration.setCameraFix(0.6, 0.4, 1.1);
      cameraConfiguration.setCameraPosition(-0.15, 10.0, 3.0);
      cameraConfiguration.setCameraTracking(true, true, true, false);
      cameraConfiguration.setCameraDolly(true, true, true, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
   }

   public SimulationTestingParameters getSimulationTestingParameters()
   {
      return simulationTestingParameters;
   }

   private class DoneListener implements SimulationDoneListener
   {

      @Override
      public void simulationDone()
      {
//         simulationCrashed.set(true);
      }

      @Override
      public void simulationDoneWithException(Throwable throwable)
      {
         simulationCrashed.set(true);
      }
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
