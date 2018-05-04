package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Before;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.SmallStepDownEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class AvatarLeapOfFaithTest implements MultiRobotTestInterface
{
   private static final Random random = new Random(100L);
   private final YoVariableRegistry registry = new YoVariableRegistry("PointyRocksTest");
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
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

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   public void testUnknownStepDownTwoFeetOnEachStep(double stepDownHeight) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 1.0;
      double transferTime = 0.2;

      int numberOfStepsDown = 5;
      double stepLength = 0.35;

      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(numberOfStepsDown, stepLength, stepDownHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("AvatarLeapOfFaithTest");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      PelvisHeightTrajectoryMessage pelvisHeight = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, 0.8);
      drcSimulationTestHelper.send(pelvisHeight);

      double executionDuration = 0.0;

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      message.setAreFootstepsAdjustable(true);

      // take care of steps down
      for (int stepNumber = 0; stepNumber < numberOfStepsDown; stepNumber++)
      {
         // step forward
         RobotSide robotSide = RobotSide.LEFT;
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1) *  stepLength, 0.0, -stepNumber * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;

         // step in place
         robotSide = robotSide.getOppositeSide();
         stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1) * stepLength, 0.0, -(stepNumber + 1) * stepDownHeight);
         footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;
      }

      int numberOfClosingSteps = 3;
      RobotSide robotSide = RobotSide.LEFT;
      for (int stepNumber = 0; stepNumber < numberOfClosingSteps; stepNumber++)
      {
         // step forward
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1 + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (numberOfClosingSteps + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.getFootstepDataList().add().set(footstepData);
      executionDuration += transferTime + swingTime;


      message.setOffsetFootstepsWithExecutionError(true);
      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * (executionDuration + transferTime));

      assertTrue("Robot had an exception, probably fell.", success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testUnknownStepDownOneFootOnEachStepLong(double stepLength, double stairLength, double stepDownHeight) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 1.0;
      double transferTime = 0.2;

      int numberOfStepsDown = 5;

      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(numberOfStepsDown, stairLength, stepDownHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      PelvisHeightTrajectoryMessage pelvisHeight = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, 0.8);
      drcSimulationTestHelper.send(pelvisHeight);

      double executionDuration = 0.0;

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      message.setAreFootstepsAdjustable(true);

      RobotSide robotSide = RobotSide.LEFT;
      // take care of steps down
      for (int stepNumber = 0; stepNumber < numberOfStepsDown; stepNumber++)
      {
         // step forward
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1) *  stepLength, 0.0, -stepNumber * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      int numberOfClosingSteps = 3;
      for (int stepNumber = 0; stepNumber < numberOfClosingSteps; stepNumber++)
      {
         // step forward
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1 + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (numberOfClosingSteps + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.getFootstepDataList().add().set(footstepData);
      executionDuration += transferTime + swingTime;


      message.setOffsetFootstepsWithExecutionError(true);
      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * (executionDuration + transferTime));

      assertTrue("Robot had an exception, probably fell.", success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testUnknownStepDownOneFootOnEachStep(double stepLength, double stairLength, double stepDownHeight) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 1.0;
      double transferTime = 0.2;

      int numberOfStepsDown = 5;

      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(numberOfStepsDown, stairLength, stepDownHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      PelvisHeightTrajectoryMessage pelvisHeight = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, 0.8);
      drcSimulationTestHelper.send(pelvisHeight);

      double executionDuration = 0.0;

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);

      message.setAreFootstepsAdjustable(true);

      RobotSide robotSide = RobotSide.LEFT;
      // take care of steps down
      for (int stepNumber = 0; stepNumber < numberOfStepsDown; stepNumber++)
      {
         // step forward
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1) *  stepLength, 0.0, -stepNumber * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      int numberOfClosingSteps = 3;
      for (int stepNumber = 0; stepNumber < numberOfClosingSteps; stepNumber++)
      {
         // step forward
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1 + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (numberOfClosingSteps + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.getFootstepDataList().add().set(footstepData);
      executionDuration += transferTime + swingTime;


      message.setOffsetFootstepsWithExecutionError(true);
      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * (executionDuration + transferTime));

      assertTrue("Robot had an exception, probably fell.", success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   public void testUnknownStepDownOneFootOnEachStepWithUncertainty(double stepDownHeight, double stepLength, double stairLength) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 1.0;
      double transferTime = 0.2;

      int numberOfStepsDown = 5;

      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(numberOfStepsDown, stairLength, stepDownHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      PelvisHeightTrajectoryMessage pelvisHeight = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, 0.8);
      drcSimulationTestHelper.send(pelvisHeight);

      double executionDuration = 0.0;

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      message.setAreFootstepsAdjustable(true);

      RobotSide robotSide = RobotSide.LEFT;
      // take care of steps down
      for (int stepNumber = 0; stepNumber < numberOfStepsDown; stepNumber++)
      {
         // step forward
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1) *  stepLength, 0.0, -stepNumber * stepDownHeight + RandomNumbers.nextDouble(random, 0.6 * stepDownHeight));
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      int numberOfClosingSteps = 3;
      for (int stepNumber = 0; stepNumber < numberOfClosingSteps; stepNumber++)
      {
         // step forward
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1 + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      // step square
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), (numberOfClosingSteps + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.getFootstepDataList().add().set(footstepData);
      executionDuration += transferTime + swingTime;


      message.setOffsetFootstepsWithExecutionError(true);
      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * (executionDuration + transferTime));

      assertTrue("Robot had an exception, probably fell.", success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testRandomHeightField(double maxStepHeight, double minStepHeight, double maxStepIncrease) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      Random random = new Random(10);

      double swingTime = 1.0;
      double transferTime = 0.2;

      int numberOfSteps = 16;

      double minStepLength = 0.3;
      double maxStepLength = 0.6;
      double endingStepLength = 0.3;
      double stepFraction = 0.8;

      ArrayList<Double> stepHeights = new ArrayList<>();
      ArrayList<Double> stepLengths = new ArrayList<>();
      ArrayList<Double> stairLengths = new ArrayList<>();

      double previousStepHeight = 0.0;
      boolean didDrop = false;
      for (int i = 0; i < numberOfSteps; i++)
      {
         double maxHeight = Math.min(previousStepHeight + maxStepIncrease, maxStepHeight);
         double stairLength = RandomNumbers.nextDouble(random, minStepLength, maxStepLength);
         double stepHeight, stepLength;
         if (didDrop)
         {
            stepHeight = Math.min(0.0, maxHeight);
            stepLength = stairLength;
         }
         else
         {
            stepHeight = RandomNumbers.nextDouble(random, minStepHeight, maxHeight);
            stepLength = stepFraction * stairLength;
         }

         previousStepHeight = stepHeight;

         stepHeights.add(stepHeight);
         stepLengths.add(stepLength);
         stairLengths.add(stairLength);

         if (didDrop)
            didDrop = false;
         else
            didDrop = true;
      }

      double starterLength = 0.35;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(stepHeights, stairLengths, starterLength, 0.0, 0.0);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      PelvisHeightTrajectoryMessage pelvisHeight = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, 0.8);
      drcSimulationTestHelper.send(pelvisHeight);

      double executionDuration = 0.0;
      double distanceTraveled = 0.5 * starterLength;

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      message.setAreFootstepsAdjustable(true);
      message.setOffsetFootstepsWithExecutionError(true);
      RobotSide robotSide = RobotSide.LEFT;
      // take care of random steps
      for (int stepNumber = 0; stepNumber < numberOfSteps; stepNumber++)
      {
         // step forward
         double stepLength = stepLengths.get(stepNumber);
         distanceTraveled += stepLength;

         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, 0.0);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);

         executionDuration += transferTime + swingTime;
         robotSide = robotSide.getOppositeSide();
      }

      int numberOfClosingSteps = 3;
      for (int stepNumber = 0; stepNumber < numberOfClosingSteps; stepNumber++)
      {
         // step forward
         distanceTraveled += endingStepLength;
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * endingStepLength, 0.0, 0.0);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * endingStepLength, 0.0, 0.0);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.getFootstepDataList().add().set(footstepData);
      executionDuration += transferTime + swingTime;


      //message.setOffsetFootstepsWithExecutionError(true);
      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * (executionDuration + transferTime));

      assertTrue("Robot had an exception, probably fell.", success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }




   public void testDropOffsWhileWalking(double stepDownHeight) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 1.0;
      double transferTime = 0.2;

      double stepLength = 0.4;
      double dropHeight = -stepDownHeight;

      int numberOfDrops = 4;
      int stepsBeforeDrop = 2;

      ArrayList<Double> stepHeights = new ArrayList<>();
      ArrayList<Double> stepLengths = new ArrayList<>();

      double currentHeight = 0.0;

      for (int i = 0; i < numberOfDrops; i++)
      {
         for (int j = 0; j < stepsBeforeDrop; j++)
         {
            stepHeights.add(currentHeight);
            stepLengths.add(stepLength);
         }

         currentHeight += dropHeight;

         stepHeights.add(currentHeight);
         stepLengths.add(stepLength);
      }


      double starterLength = 0.35;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(stepHeights, stepLengths, starterLength, 0.0, currentHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double executionDuration = 0.0;
      double distanceTraveled = 0.5 * starterLength;

      FootstepDataListMessage message = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);
      message.setAreFootstepsAdjustable(true);
      RobotSide robotSide = RobotSide.LEFT;

      int numberOfSteps = stepLengths.size();

      // take care of falling steps
      double stepHeight = 0.0;
      for (int stepNumber = 0; stepNumber < numberOfSteps; stepNumber++)
      {
         // step forward
         distanceTraveled += stepLength;

         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, stepHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);

         stepHeight = stepHeights.get(stepNumber);

         executionDuration += transferTime + swingTime;
         robotSide = robotSide.getOppositeSide();
      }

      int numberOfClosingSteps = 3;
      for (int stepNumber = 0; stepNumber < numberOfClosingSteps; stepNumber++)
      {
         // step forward
         distanceTraveled += stepLength;
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, stepHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, stepHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.getFootstepDataList().add().set(footstepData);
      executionDuration += transferTime + swingTime;

      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * (executionDuration + transferTime));

      assertTrue("Robot had an exception, probably fell.", success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   private FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, FramePoint3D placeToStep)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();

      FramePoint3D placeToStepInWorld = new FramePoint3D(placeToStep);
      placeToStepInWorld.changeFrame(worldFrame);

      footstepData.getLocation().set(placeToStepInWorld);
      footstepData.getOrientation().set(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide.toByte());

      return footstepData;
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
