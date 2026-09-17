package us.ihmc.avatar.roughTerrainWalking;

import static org.junit.jupiter.api.Assertions.*;

import controller_msgs.FootstepDataListMessage;
import controller_msgs.FootstepDataMessage;
import gnu.trove.list.array.TDoubleArrayList;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
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
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableHolder;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public abstract class AvatarAbsoluteStepTimingsTest implements MultiRobotTestInterface
{
   protected final static SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;

   private static final double swingStartTimeEpsilon = 0.0075;

   @Test
   public void testTakingStepsWithAbsoluteTimings()
   {
      TestingEnvironment environment = new TestingEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, environment, simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.setCameraPosition(8.0, -8.0, 5.0);
      simulationTestHelper.setCameraFocusPosition(1.5, 0.0, 0.8);
      assertTrue(simulationTestHelper.simulateNow(0.25));
      Random random = new Random(59249625689L);

      double swingStartInterval = 1.125;
      int steps = 20;

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParametersForStepGeneration();
      double stepWidth = (steppingParameters.getMinStepWidth() + steppingParameters.getMaxStepWidth()) / 2.0;
      double stepLength = steppingParameters.getDefaultStepLength() / 2.0;
      stepLength = Math.max(steppingParameters.getActualFootLength() * 1.2, stepLength);
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
      double timeToSendSecondMessage = simulationTestHelper.getSimulationTime();

      for (int stepIndex = 0; stepIndex < steps; stepIndex++)
      {
         RobotSide side = stepIndex % 2 == 0 ? RobotSide.LEFT : RobotSide.RIGHT;
         double y = side == RobotSide.LEFT ? stepWidth / 2.0 : -stepWidth / 2.0;
         Point3D location = new Point3D(stepIndex * stepLength, y, environment.getHeight(stepIndex * stepLength));
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(side, location, orientation);
         footstepData.setSwingHeight(0.12);
         footstepData.getCustomWaypointProportions().add(0.05);
         footstepData.getCustomWaypointProportions().add(0.95);
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
      footstepMessage1.setOffsetFootstepsHeightWithExecutionError(true);
      footstepMessage2.setOffsetFootstepsHeightWithExecutionError(true);

      YoVariable yoTime = simulationTestHelper.getSimulationConstructionSet().getTime();
      TimingChecker timingChecker1 = new TimingChecker(simulationTestHelper, footstepMessage1, footstepMessage2);
      yoTime.addListener(timingChecker1);

      simulationTestHelper.publishToController(footstepMessage1);

      boolean hasMessageBeenSent = false;
      while (!timingChecker1.isDone())
      {
         if (simulationTestHelper.getSimulationTime() > timeToSendSecondMessage && !hasMessageBeenSent)
         {
            simulationTestHelper.publishToController(footstepMessage2);
            hasMessageBeenSent = true;
         }
         boolean success = simulationTestHelper.simulateNow(0.2);
         if (!success)
            timingChecker1.setEnable(false);
         assertTrue(success);
      }
   }

   private static class TimingChecker implements YoVariableChangedListener
   {
      private static final String failMessage = "Swing did not start at expected time.";

      private int stepCount = 0;
      private double expectedStartTimeOfNextStep = 0.0;
      private WalkingStateEnum previousWalkingState = WalkingStateEnum.STANDING;

      private final YoVariableHolder yoVariableHolder;
      private final FootstepDataListMessage footstepMessage1;
      private final FootstepDataListMessage footstepMessage2;

      private boolean isDone = false;
      private boolean enable = true;

      public TimingChecker(YoVariableHolder yoVariableHolder, FootstepDataListMessage footstepMessage1, FootstepDataListMessage footstepMessage2)
      {
         this.yoVariableHolder = yoVariableHolder;
         this.footstepMessage1 = footstepMessage1;
         this.footstepMessage2 = footstepMessage2;
      }

      public void setEnable(boolean enable)
      {
         this.enable = enable;
      }

      @Override
      public void changed(YoVariable v)
      {
         if (!enable)
         {
            return;
         }
         if (isDone)
         {
            return;
         }

         double time = v.getValueAsDouble();
         WalkingStateEnum walkingState = getWalkingState(yoVariableHolder);

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

            assertEquals(expectedStartTimeOfNextStep, time, swingStartTimeEpsilon, failMessage);

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
   public void testMinimumTransferTimeIsRespected()
   {
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCRobotModel robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel, environment, simulationTestingParameters);
      simulationTestHelper.start();
      simulationTestHelper.setCameraPosition(8.0, -8.0, 5.0);
      simulationTestHelper.setCameraFocusPosition(1.5, 0.0, 0.8);
      assertTrue(simulationTestHelper.simulateNow(0.25));

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

      simulationTestHelper.publishToController(footsteps);
      assertTrue(simulationTestHelper.simulateNow(minimumTransferTime / 2.0));
      checkTransferTimes(simulationTestHelper, minimumTransferTime);
   }

   @SuppressWarnings("unchecked")
   public void testPausingWalkDuringLongTransfers()
   {
      DRCRobotModel robotModel = getRobotModel();
      simulationTestHelper = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulation(robotModel,
                                                                                            new FlatGroundEnvironment(),
                                                                                            simulationTestingParameters);
      simulationTestHelper.start();
      assertTrue(simulationTestHelper.simulateNow(0.25));

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
         ReferenceFrame soleFrame = simulationTestHelper.getControllerFullRobotModel().getSoleFrame(side);
         FramePose3D footstepPose = new FramePose3D(soleFrame);
         footstepPose.changeFrame(ReferenceFrame.getWorldFrame());

         FootstepDataMessage footstepMessage = message.getFootstepDataList().add();
         footstepMessage.setRobotSide(side.toByte());
         footstepMessage.setTransferDuration(transferDuration);
         footstepMessage.setSwingDuration(swingDuration);
         footstepMessage.getLocation().getPoint().set(footstepPose.getPosition());
         footstepMessage.getOrientation().set(footstepPose.getOrientation());

         expectPause.add(transferDuration > transferSwitchDuration);
         stepTime.add(transferDuration + swingDuration);
      }

      simulationTestHelper.publishToController(message);
      assertTrue(simulationTestHelper.simulateNow(0.1));
      YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) simulationTestHelper.findVariable("WalkingCurrentState");
      for (int i = 0; i < steps; i++)
      {
         assertTrue(simulationTestHelper.simulateNow(stepTime.get(i)));
         if (i + 1 < steps)
         {
            boolean isPaused = WalkingStateEnum.TO_STANDING == walkingState.getEnumValue();
            assertEquals(expectPause.get(i + 1), isPaused);
         }
      }
      assertTrue(simulationTestHelper.simulateNow(finalTransferDuration));
   }

   private void checkTransferTimes(YoVariableHolder yoVariableHolder, double minimumTransferTime)
   {
      YoDouble firstTransferTime = getDoubleYoVariable(yoVariableHolder, "transferTime0", BalanceManager.class.getSimpleName());
      assertTrue(firstTransferTime.getDoubleValue() >= minimumTransferTime, "Executing transfer that is faster then allowed.");
   }

   private static YoDouble getDoubleYoVariable(YoVariableHolder yoVariableHolder, String name, String namespace)
   {
      return getYoVariable(yoVariableHolder, name, namespace, YoDouble.class);
   }

   private static WalkingStateEnum getWalkingState(YoVariableHolder yoVariableHolder)
   {
      return (WalkingStateEnum) getYoVariable(yoVariableHolder,
                                              "WalkingCurrentState",
                                              WalkingHighLevelHumanoidController.class.getSimpleName(),
                                              YoEnum.class).getEnumValue();
   }

   private static <T extends YoVariable> T getYoVariable(YoVariableHolder yoVariableHolder, String name, String namespace, Class<T> clazz)
   {
      YoVariable uncheckedVariable = yoVariableHolder.findVariable(namespace, name);
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

      private final TDoubleArrayList xStarts = new TDoubleArrayList();
      private final TDoubleArrayList xEnds = new TDoubleArrayList();
      private final TDoubleArrayList heights = new TDoubleArrayList();

      public TestingEnvironment()
      {
         SteppingParameters steppingParameters = getRobotModel().getWalkingControllerParameters().getSteppingParameters();
         double flatArea = steppingParameters.getDefaultStepLength() * 0.5;
         flatArea = Math.max(steppingParameters.getActualFootLength() * 1.2, flatArea);
         double maxElevation = getRobotModel().getWalkingControllerParameters().getSwingTrajectoryParameters().getDefaultSwingHeight() * 0.25;

         terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
         terrain.addBox(-0.5 - flatArea / 2.0, -1.0, flatArea / 2.0, 1.0, -0.01, 0.0);
         xStarts.add(-0.5 - flatArea / 2.0);
         xEnds.add(flatArea / 2.0);
         heights.add(0.0);

         for (int i = 0; i < 50; i++)
         {
            double xStart = flatArea + i * flatArea - flatArea / 2.0;
            double height = maxElevation * 2.0 * (random.nextDouble() - 0.5);
            double length = flatArea;
            terrain.addBox(xStart, -1.0, xStart + length, 1.0, height - 0.01, height);

            xStarts.add(xStart);
            xEnds.add(xStart + length);
            heights.add(height);
         }
      }

      public double getHeight(double x)
      {
         for (int i = 0; i < xStarts.size(); i++)
         {
            if (x >= xStarts.get(i) && x < xEnds.get(i))
               return heights.get(i);
         }

         return 0.0;
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
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
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

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }
}
