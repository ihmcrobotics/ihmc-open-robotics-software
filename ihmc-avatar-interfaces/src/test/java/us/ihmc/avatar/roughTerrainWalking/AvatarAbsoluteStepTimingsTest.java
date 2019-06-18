package us.ihmc.avatar.roughTerrainWalking;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.SmoothCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.ExecutionTiming;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

@Tag("humanoid-rough-terrain")
public abstract class AvatarAbsoluteStepTimingsTest implements MultiRobotTestInterface
{
   protected final static SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static final double swingStartTimeEpsilon = 0.005;

   @Test
   public void testTakingStepsWithAbsoluteTimings() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();
      CommonAvatarEnvironmentInterface environment = new TestingEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.createSimulation(className);
      ThreadTools.sleep(1000);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.setCameraPosition(8.0, -8.0, 5.0);
      scs.setCameraFix(1.5, 0.0, 0.8);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      Random random = new Random(59249625689L);

      double swingStartInterval = 1.125;
      int steps = 20;

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepWidth = (walkingControllerParameters.getSteppingParameters().getMinStepWidth()
            + walkingControllerParameters.getSteppingParameters().getMaxStepWidth()) / 2.0;
      double stepLength = walkingControllerParameters.getSteppingParameters().getDefaultStepLength() / 2.0;
      double defaultSwingTime = walkingControllerParameters.getDefaultSwingTime();
      double defaultTransferTime = walkingControllerParameters.getDefaultTransferTime();

      FootstepDataListMessage footstepMessage1 = new FootstepDataListMessage();
      footstepMessage1.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS.toByte());
      footstepMessage1.getQueueingProperties().setMessageId(1);

      FootstepDataListMessage footstepMessage2 = new FootstepDataListMessage();
      footstepMessage2.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS.toByte());
      footstepMessage2.getQueueingProperties().setExecutionMode(ExecutionMode.QUEUE.toByte());
      footstepMessage2.getQueueingProperties().setPreviousMessageId(1);
      footstepMessage2.getQueueingProperties().setMessageId(2);

      double takeOffTime = 0.0;
      double previousSwingTime = 0.0;
      double timeToSendSecondMessage = scs.getTime();

      for (int stepIndex = 0; stepIndex < steps; stepIndex++)
      {
         RobotSide side = stepIndex % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double y = side == RobotSide.LEFT ? stepWidth / 2.0 : -stepWidth / 2.0;
         Point3D location = new Point3D(stepIndex * stepLength, y, 0.0);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(side, location, orientation);
         footstepData.setSwingHeight(0.04);
         double transferTime = defaultTransferTime + random.nextDouble() * 0.5;
         double swingTime = defaultSwingTime + random.nextDouble() * 0.5 - 0.2;
         double touchdownTime = transferTime * random.nextDouble() * 0.3;

         if (stepIndex == 0)
         {
            footstepData.setTransferDuration(swingStartInterval);
         }
         else if (stepIndex == 10)
         {
            footstepData.setTransferDuration(2.0);
         }
         else
         {
            footstepData.setTransferDuration(transferTime);
         }

         footstepData.setSwingDuration(swingTime);
         footstepData.setTouchdownDuration(touchdownTime);

         takeOffTime += previousSwingTime + footstepData.getTransferDuration();
         LogTools.info(stepIndex + ": " + takeOffTime);

         previousSwingTime = footstepData.getSwingDuration();

         if (stepIndex == 7)
         {
            timeToSendSecondMessage += takeOffTime;
         }

         if (stepIndex < 10)
         {
            footstepMessage1.getFootstepDataList().add().set(footstepData);
         }
         else
         {
            footstepMessage2.getFootstepDataList().add().set(footstepData);
         }
      }

      YoVariable<?> yoTime = drcSimulationTestHelper.getSimulationConstructionSet().getVariable("t");
      TimingChecker timingChecker1 = new TimingChecker(scs, footstepMessage1, footstepMessage2);
      yoTime.addVariableChangedListener(timingChecker1);

      drcSimulationTestHelper.publishToController(footstepMessage1);

      boolean hasMessageBeenSent = false;
      while (!timingChecker1.isDone())
      {
         if (scs.getTime() > timeToSendSecondMessage && !hasMessageBeenSent)
         {
            drcSimulationTestHelper.publishToController(footstepMessage2);
            hasMessageBeenSent = true;
         }
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.2));
      }
   }

   private class TimingChecker implements VariableChangedListener
   {
      private static final String failMessage = "Swing did not start at expected time.";

      private int stepCount = 0;
      private double expectedStartTimeOfNextStep = 0.0;
      private WalkingStateEnum previousWalkingState = WalkingStateEnum.STANDING;

      private final SimulationConstructionSet scs;
      private final FootstepDataListMessage footstepMessage1;
      private final FootstepDataListMessage footstepMessage2;

      private boolean isDone = false;

      public TimingChecker(SimulationConstructionSet scs, FootstepDataListMessage footstepMessage1, FootstepDataListMessage footstepMessage2)
      {
         this.scs = scs;
         this.footstepMessage1 = footstepMessage1;
         this.footstepMessage2 = footstepMessage2;
      }

      @Override
      public void notifyOfVariableChange(YoVariable<?> v)
      {
         if (isDone)
         {
            return;
         }

         double time = v.getValueAsDouble();
         WalkingStateEnum walkingState = getWalkingState(scs);

         if (previousWalkingState.isDoubleSupport() && walkingState.isSingleSupport())
         {
            if (stepCount == 0)
            {
               expectedStartTimeOfNextStep = time;
            }

            // added this to allow the test to keep going with printouts if you comment out the assert
            boolean success = MathTools.epsilonEquals(expectedStartTimeOfNextStep, time, swingStartTimeEpsilon);
            if (!success)
            {
               LogTools.error(stepCount + " expected: " + expectedStartTimeOfNextStep + " but was: " + time);
            }

            assertEquals(failMessage, expectedStartTimeOfNextStep, time, swingStartTimeEpsilon);

            if (stepCount > footstepMessage1.getFootstepDataList().size() + footstepMessage2.getFootstepDataList().size() - 2)
            {
               isDone = true;
               return;
            }

            if (stepCount < footstepMessage1.getFootstepDataList().size())
            {
               double swingTime = footstepMessage1.getFootstepDataList().get(stepCount).getSwingDuration();

               double transferTime = Double.NaN;
               if (stepCount == footstepMessage1.getFootstepDataList().size() - 1)
               {
                  transferTime = footstepMessage2.getFootstepDataList().get(0).getTransferDuration();
               }
               else
               {
                  transferTime = footstepMessage1.getFootstepDataList().get(stepCount + 1).getTransferDuration();
               }

               expectedStartTimeOfNextStep += swingTime + transferTime;
            }
            else
            {
               double swingTime = footstepMessage2.getFootstepDataList().get(stepCount - footstepMessage1.getFootstepDataList().size()).getSwingDuration();
               double transferTime = footstepMessage2.getFootstepDataList().get(stepCount + 1 - footstepMessage1.getFootstepDataList().size())
                                                     .getTransferDuration();
               expectedStartTimeOfNextStep += swingTime + transferTime;
            }

            stepCount++;
         }

         previousWalkingState = walkingState;
      }

      public boolean isDone()
      {
         return isDone;
      }

   }

   @Test
   public void testMinimumTransferTimeIsRespected() throws SimulationExceededMaximumTimeException
   {
      String className = getClass().getSimpleName();
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.createSimulation(className);
      ThreadTools.sleep(1000);
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.setCameraPosition(8.0, -8.0, 5.0);
      scs.setCameraFix(1.5, 0.0, 0.8);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      FootstepDataListMessage footsteps = HumanoidMessageTools.createFootstepDataListMessage(0.6, 0.3, 0.1);
      footsteps.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS.toByte());
      double minimumTransferTime = getRobotModel().getWalkingControllerParameters().getMinimumTransferTime();

      // add very fast footstep:
      {
         RobotSide side = RobotSide.LEFT;
         double y = side == RobotSide.LEFT ? 0.15 : -0.15;
         Point3D location = new Point3D(0.0, y, 0.0);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(side, location, orientation);
         footstepData.setTransferDuration(minimumTransferTime / 2.0);
         footsteps.getFootstepDataList().add().set(footstepData);
      }

      drcSimulationTestHelper.publishToController(footsteps);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(minimumTransferTime / 2.0));
      checkTransferTimes(scs, minimumTransferTime);
   }

   @SuppressWarnings("unchecked")
   public void testPausingWalkDuringLongTransfers() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());
      ThreadTools.sleep(1000);
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25));

      double finalTransferDuration = 0.5;
      double swingDuration = 0.6;
      double defaultInitialTransferDuration = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double minTransferDuration = robotModel.getWalkingControllerParameters().getMinimumTransferTime();

      FootstepDataListMessage message = new FootstepDataListMessage();
      message.setFinalTransferDuration(finalTransferDuration);
      message.setExecutionTiming(ExecutionTiming.CONTROL_ABSOLUTE_TIMINGS.toByte());

      int steps = 10;
      Random random = new Random(149L);

      List<Double> stepTime = new ArrayList<>();
      List<Boolean> expectPause = new ArrayList<>();

      for (int i = 0; i < steps; i++)
      {
         // If the transfer time is larger then the final transfer and initial transfer duration the robot will pause:
         double transferSwitchDuration = finalTransferDuration + defaultInitialTransferDuration;
         double transferDuration = transferSwitchDuration + 2.0 * (transferSwitchDuration - minTransferDuration) * (random.nextDouble() - 0.5);

         RobotSide side = RobotSide.generateRandomRobotSide(random);
         ReferenceFrame soleFrame = drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(side);
         FramePose3D footstepPose = new FramePose3D(soleFrame);
         footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

         FootstepDataMessage footstepMessage = message.getFootstepDataList().add();
         footstepMessage.setRobotSide(side.toByte());
         footstepMessage.setTransferDuration(transferDuration);
         footstepMessage.setSwingDuration(swingDuration);
         footstepMessage.getLocation().set(footstepPose.getPosition());
         footstepMessage.getOrientation().set(footstepPose.getOrientation());

         expectPause.add(transferDuration > transferSwitchDuration);
         stepTime.add(transferDuration + swingDuration);
      }

      drcSimulationTestHelper.publishToController(message);
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1));
      YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) drcSimulationTestHelper.getYoVariable("WalkingCurrentState");
      for (int i = 0; i < steps; i++)
      {
         Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(stepTime.get(i)));
         if (i + 1 < steps)
         {
            boolean isPaused = WalkingStateEnum.TO_STANDING == walkingState.getEnumValue();
            Assert.assertEquals(expectPause.get(i + 1), isPaused);
         }
      }
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(finalTransferDuration));
   }

   private void checkTransferTimes(SimulationConstructionSet scs, double minimumTransferTime)
   {
      YoDouble firstTransferTime = getDoubleYoVariable(scs, "icpPlannerTransferDuration0", SmoothCMPBasedICPPlanner.class.getSimpleName());
      assertTrue("Executing transfer that is faster then allowed.", firstTransferTime.getDoubleValue() >= minimumTransferTime);
   }

   private static YoDouble getDoubleYoVariable(SimulationConstructionSet scs, String name, String namespace)
   {
      return getYoVariable(scs, name, namespace, YoDouble.class);
   }

   @SuppressWarnings("unchecked")
   private static WalkingStateEnum getWalkingState(SimulationConstructionSet scs)
   {
      return (WalkingStateEnum) getYoVariable(scs, "WalkingCurrentState", WalkingHighLevelHumanoidController.class.getSimpleName(),
                                              YoEnum.class).getEnumValue();
   }

   private static <T extends YoVariable<T>> T getYoVariable(SimulationConstructionSet scs, String name, String namespace, Class<T> clazz)
   {
      YoVariable<?> uncheckedVariable = scs.getVariable(namespace, name);
      if (uncheckedVariable == null)
         throw new RuntimeException("Could not find yo variable: " + namespace + "/" + name + ".");
      if (!clazz.isInstance(uncheckedVariable))
         throw new RuntimeException("YoVariable " + name + " is not of type " + clazz.getSimpleName());
      return clazz.cast(uncheckedVariable);
   }

   public class TestingEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrain;
      private final Random random = new Random(19389481L);

      public TestingEnvironment()
      {
         SteppingParameters steppingParameters = getRobotModel().getWalkingControllerParameters().getSteppingParameters();
         double flatArea = steppingParameters.getDefaultStepLength() * 0.5;
         double maxElevation = steppingParameters.getDefaultSwingHeightFromStanceFoot() * 0.25;

         terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
         terrain.addBox(-0.5 - flatArea / 2.0, -1.0, flatArea / 2.0, 1.0, -0.01, 0.0);

         for (int i = 0; i < 50; i++)
         {
            double xStart = flatArea + i * flatArea - flatArea / 2.0;
            double height = maxElevation * 2.0 * (random.nextDouble() - 0.5);
            double length = flatArea;
            terrain.addBox(xStart, -1.0, xStart + length, 1.0, height - 0.01, height);
         }
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return terrain;
      }

      @Override
      public List<? extends Robot> getEnvironmentRobots()
      {
         return null;
      }

      @Override
      public void createAndSetContactControllerToARobot()
      {
      }

      @Override
      public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
      {
      }

      @Override
      public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
      {
      }

   }

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

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }
}
