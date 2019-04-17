package us.ihmc.avatar.straightLegWalking;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CinderBlockFieldEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.SmallStepDownEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.StairsUpAndDownEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class AvatarStraightLegWalkingTest implements MultiRobotTestInterface
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testForwardWalking() throws SimulationExceededMaximumTimeException
   {
//      simulationTestingParameters.setKeepSCSUp(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");

      setupCamera();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      ThreadTools.sleep(1000);

      double transferDuration = 0.2;
      double swingDuration = 0.7;
      double length = 0.6;
      int numberOfSteps = 6;
      List<FootstepDataMessage> footsteps = getFlatGroundFootsteps(numberOfSteps, length, transferDuration, swingDuration);
      FootstepDataListMessage footstepListMessage = HumanoidMessageTools
            .createFootstepDataListMessage(footsteps, swingDuration, transferDuration, ExecutionMode.OVERRIDE);
      footstepListMessage.setAreFootstepsAdjustable(true);
      footstepListMessage.setOffsetFootstepsWithExecutionError(true);

      drcSimulationTestHelper.publishToController(footstepListMessage);

      double timeOverrunFactor = 1.1;

      double simulationTime = timeOverrunFactor * numberOfSteps * (transferDuration + swingDuration) + 3.0;

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);

      assertReachedGoal(footstepListMessage);
   }

   private ArrayList<FootstepDataMessage> getFlatGroundFootsteps(int numberOfSteps, double length, double transferDuration, double swingDuration)
   {
      ArrayList<FootstepDataMessage> footsteps = new ArrayList<>();
      RobotSide robotSide = RobotSide.RIGHT;
      double width = 0.22;
      double xLocation = 0.0;
      for (int i = 0; i < numberOfSteps; i++)
      {
         xLocation += length;
         FootstepDataMessage dataMessage = HumanoidMessageTools
               .createFootstepDataMessage(robotSide, new Point3D(xLocation, robotSide.negateIfRightSide(width / 2.0), 0.0), new Quaternion());
         dataMessage.setSwingDuration(swingDuration);
         if (i > 0)
            dataMessage.setTransferDuration(transferDuration);

         footsteps.add(dataMessage);

         robotSide = robotSide.getOppositeSide();
      }

      FootstepDataMessage dataMessage = HumanoidMessageTools
            .createFootstepDataMessage(robotSide, new Point3D(xLocation, robotSide.negateIfRightSide(width / 2.0), 0.0), new Quaternion());
      dataMessage.setSwingDuration(swingDuration);
      dataMessage.setTransferDuration(transferDuration);

      footsteps.add(dataMessage);

      return footsteps;
   }


   @Test
   public void testWalkingOverCinderBlockField() throws Exception
   {
      CinderBlockFieldEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldEnvironment();
      FootstepDataListMessage footsteps = generateFootstepsForCinderBlockField(cinderBlockFieldEnvironment.getCinderBlockPoses());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), cinderBlockFieldEnvironment);
      drcSimulationTestHelper.createSimulation("EndToEndCinderBlockFieldTest");

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      drcSimulationTestHelper.publishToController(footsteps);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      int numberOfSteps = footsteps.getFootstepDataList().size();
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(numberOfSteps * stepTime + 2.0 * initialFinalTransfer + 3.0);

      assertReachedGoal(footsteps);
      assertTrue(success);

   }

   @Test
   public void testWalkingOverStairs() throws Exception
   {
      StairsUpAndDownEnvironment stairsEnvironment = new StairsUpAndDownEnvironment();
      FootstepDataListMessage footsteps = generateFootstepsForStairs(stairsEnvironment.getStairPoses());
      //footsteps.setDefaultTransferDuration(0.5);
      //footsteps.setDefaultSwingDuration(1.0);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), stairsEnvironment);
      drcSimulationTestHelper.createSimulation("EndToEndCinderBlockFieldTest");

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);
      assertTrue(success);

      drcSimulationTestHelper.publishToController(footsteps);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(footsteps.getFootstepDataList().size() * stepTime + 2.0 * initialFinalTransfer + 1.0);

      assertReachedGoal(footsteps);
   }

   @Test
   public void testSlowerWalking() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");

      setupCamera();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      ThreadTools.sleep(1000);
      int numberOfSteps = 6;
      double stepLength = 0.35;
      double transferDuration = 0.75;
      double swingDuration = 1.25;

      List<FootstepDataMessage> footsteps = getFlatGroundFootsteps(numberOfSteps, stepLength, transferDuration, swingDuration);
      FootstepDataListMessage footstepListMessage = HumanoidMessageTools
            .createFootstepDataListMessage(footsteps, swingDuration, transferDuration, ExecutionMode.OVERRIDE);
      footstepListMessage.setAreFootstepsAdjustable(true);
      footstepListMessage.setOffsetFootstepsWithExecutionError(true);

      drcSimulationTestHelper.publishToController(footstepListMessage);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(numberOfSteps * (transferDuration + swingDuration) + 4.0);

      assertReachedGoal(footstepListMessage);
   }

   @Test
   public void testDropOffsWhileWalking() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.08;
      double stepLength = 0.35;
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
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      setupCamera();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double distanceTraveled = 0.5 * starterLength;

      FootstepDataListMessage message = new FootstepDataListMessage();
      RobotSide robotSide = RobotSide.LEFT;

      int numberOfSteps = stepLengths.size();
      double instep = 0.03;

      // take care of falling steps
      double stepHeight = 0.0;
      for (int stepNumber = 0; stepNumber < numberOfSteps; stepNumber++)
      {
         // step forward
         distanceTraveled += stepLength;
         instep = -instep;

         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, instep, stepHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);

         stepHeight = stepHeights.get(stepNumber);

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

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, stepHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.getFootstepDataList().add().set(footstepData);
      //message.setOffsetFootstepsWithExecutionError(true);

      drcSimulationTestHelper.publishToController(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * message.getFootstepDataList().size() * 2.0);

      assertReachedGoal(message);
   }

   @Test
   public void testSteppingDown() throws SimulationExceededMaximumTimeException
   {
      double stepDownHeight = 0.2;
      runSteppingDown(stepDownHeight, 0.30, 1);
   }

   @Test
   public void testSteppingDownEveryTime() throws Exception
   {
      double stepLength = 0.35;
      double stepDownHeight = 0.15;
      runSteppingDown(stepDownHeight, stepLength, 0);
   }


   private void runSteppingDown(double stepDownHeight, double stepLength, int stepsBeforeDrop) throws SimulationExceededMaximumTimeException
   {
      double dropHeight = -stepDownHeight;

      int numberOfDrops = 4;

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
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      setupCamera();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double distanceTraveled = 0.5 * starterLength;

      FootstepDataListMessage message = new FootstepDataListMessage();
      RobotSide robotSide = RobotSide.LEFT;

      int numberOfSteps = stepLengths.size();
      double instep = 0.03;

      // take care of falling steps
      double stepHeight = 0.0;
      for (int stepNumber = 0; stepNumber < numberOfSteps; stepNumber++)
      {
         // step forward
         distanceTraveled += stepLength;
         instep = -instep;
         stepHeight = stepHeights.get(stepNumber);

         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, instep, stepHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.getFootstepDataList().add().set(footstepData);

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

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, stepHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.getFootstepDataList().add().set(footstepData);
      //message.setOffsetFootstepsWithExecutionError(true);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepDuration = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double addedTime = walkingControllerParameters.getDefaultInitialTransferTime() - walkingControllerParameters.getDefaultTransferTime() + walkingControllerParameters.getDefaultFinalTransferTime();

      drcSimulationTestHelper.publishToController(message);

      double timeOverrunFactor = 1.03;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * message.getFootstepDataList().size() * stepDuration + addedTime);

      assertTrue(success);
      assertReachedGoal(message);
   }

   @Test
   public void testRandomHeightField() throws Exception
   {
      double maxStepIncrease = 0.07;
      double maxStepHeight = 0.04;
      double minStepHeight = -0.12;
      runRandomHeightField(maxStepHeight, minStepHeight, maxStepIncrease);
   }

   private void runRandomHeightField(double maxStepHeight, double minStepHeight, double maxStepIncrease) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      Random random = new Random(10);

      int numberOfSteps = 16;

      double minStepLength = 0.3;
      double maxStepLength = 0.6;
      double endingStepLength = 0.3;

      ArrayList<Double> stepHeights = new ArrayList<>();
      ArrayList<Double> stepLengths = new ArrayList<>();

      double previousStepHeight = 0.0;
      boolean didDrop = false;
      for (int i = 0; i < numberOfSteps; i++)
      {
         double maxHeight = Math.min(previousStepHeight + maxStepIncrease, maxStepHeight);
         double stepLength = RandomNumbers.nextDouble(random, minStepLength, maxStepLength);
         double stepHeight;
         if (didDrop)
            stepHeight = Math.min(0.0, maxHeight);
         else
            stepHeight = RandomNumbers.nextDouble(random, minStepHeight, maxHeight);

         previousStepHeight = stepHeight;

         stepHeights.add(stepHeight);
         stepLengths.add(stepLength);

         if (didDrop)
            didDrop = false;
         else
            didDrop = true;
      }

      double starterLength = 0.35;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(stepHeights, stepLengths, starterLength, 0.0, 0.0);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), stepDownEnvironment);
      drcSimulationTestHelper.createSimulation("HumanoidPointyRocksTest");

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      setupCamera();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double distanceTraveled = 0.5 * starterLength;

      FootstepDataListMessage message = new FootstepDataListMessage();
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

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * endingStepLength, 0.0, 0.0);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.getFootstepDataList().add().set(footstepData);

      //message.setOffsetFootstepsWithExecutionError(true);
      drcSimulationTestHelper.publishToController(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * message.getFootstepDataList().size() * 2.0);

      assertTrue(success);
      assertReachedGoal(message);
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

   private static FootstepDataListMessage generateFootstepsForCinderBlockField(List<List<FramePose3D>> cinderBlockPoses)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      footsteps.setOffsetFootstepsHeightWithExecutionError(true);

      int numberOfColumns = cinderBlockPoses.get(0).size();

      int indexForLeftSide = (numberOfColumns - 1) / 2;
      int indexForRightSide = indexForLeftSide + 1;
      SideDependentList<List<FramePose3D>> columns = extractColumns(cinderBlockPoses, indexForLeftSide, indexForRightSide);

      for (int row = 0; row < cinderBlockPoses.size(); row++)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FramePose3D cinderBlockPose = columns.get(robotSide).get(row);
            Point3D location = new Point3D();
            Quaternion orientation = new Quaternion();
            cinderBlockPose.get(location, orientation);
            location.setZ(location.getZ() + 0.02);
            FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
            footsteps.getFootstepDataList().add().set(footstep);
         }
      }

      return footsteps;
   }

   private static FootstepDataListMessage generateFootstepsForStairs(List<List<FramePose3D>> stepPoses)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      double stepWidth = 0.275;
      int numberOfStartingSteps = 3;
      double firstStepPosition = stepPoses.get(0).get(0).getX();
      double startingLength = (firstStepPosition - 0.35) / numberOfStartingSteps;

      // approach the stairs
      RobotSide robotSide = RobotSide.LEFT;
      for (int stepIndex = 0; stepIndex < numberOfStartingSteps; stepIndex++)
      {
         double yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

         FramePose3D stepPose = new FramePose3D();
         stepPose.setX(startingLength * (stepIndex + 1));
         stepPose.setY(yPosition);

         Point3D location = new Point3D();
         Quaternion orientation = new Quaternion();
         stepPose.get(location, orientation);
         FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
         footsteps.getFootstepDataList().add().set(footstep);

         robotSide = robotSide.getOppositeSide();
      }

      // closing step
      double yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

      FramePose3D stepPose = new FramePose3D();
      stepPose.setX(startingLength * numberOfStartingSteps);
      stepPose.setY(yPosition);

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.get(location, orientation);
      FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(robotSide, location, orientation);
      footsteps.getFootstepDataList().add().set(footstep);

      // ascend the stairs
      List<FramePose3D> stepUpPoses = stepPoses.get(0);
      for (int row = 0; row < stepUpPoses.size(); row++)
      {
         for (RobotSide stepSide : RobotSide.values)
         {
            yPosition = stepSide.negateIfRightSide(stepWidth / 2.0);
            FramePose3D stairPose = stepUpPoses.get(row);
            stairPose.setY(yPosition);

            Point3D stairLocation = new Point3D();
            Quaternion stairOrientation = new Quaternion();
            stairPose.get(stairLocation, stairOrientation);

            FootstepDataMessage stepFootstep = HumanoidMessageTools.createFootstepDataMessage(stepSide, stairLocation, stairOrientation);
            footsteps.getFootstepDataList().add().set(stepFootstep);
         }
      }

      FramePose3D lastStepGoingUpStairs = stepPoses.get(0).get(stepPoses.get(0).size() - 1);
      double topOfStairs = lastStepGoingUpStairs.getX();
      double startOfAscent = stepPoses.get(1).get(0).getX() - 0.35;
      double platformWidth = startOfAscent - topOfStairs;

      int numberOfPlatformSteps = 1;
      double platformStepLength = (platformWidth) / numberOfPlatformSteps;

      // approach the top of the stairs
      robotSide = RobotSide.LEFT;
      for (int stepIndex = 0; stepIndex < numberOfPlatformSteps; stepIndex++)
      {
         yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

         FramePose3D landingPose = new FramePose3D();
         landingPose.setX(platformStepLength * (stepIndex + 1) + topOfStairs);
         landingPose.setY(yPosition);
         landingPose.setZ(lastStepGoingUpStairs.getZ());

         Point3D stepLocation = new Point3D();
         Quaternion stepOrientation = new Quaternion();
         landingPose.get(stepLocation, stepOrientation);
         FootstepDataMessage stepMessage = HumanoidMessageTools.createFootstepDataMessage(robotSide, stepLocation, stepOrientation);
         footsteps.getFootstepDataList().add().set(stepMessage);

         robotSide = robotSide.getOppositeSide();
      }

      // closing step
      yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

      FramePose3D landingPose = new FramePose3D();
      double forwardLocation = footsteps.getFootstepDataList().get(footsteps.getFootstepDataList().size() - 1).getLocation().getX();
      landingPose.setX(forwardLocation);
      landingPose.setY(yPosition);
      landingPose.setZ(lastStepGoingUpStairs.getZ());

      Point3D stepLocation = new Point3D();
      Quaternion stepOrientation = new Quaternion();
      landingPose.get(stepLocation, stepOrientation);
      FootstepDataMessage landingMessage = HumanoidMessageTools.createFootstepDataMessage(robotSide, stepLocation, stepOrientation);
      footsteps.getFootstepDataList().add().set(landingMessage);

      // descend the stairs
      List<FramePose3D> stepDownPoses = stepPoses.get(1);
      for (int row = 0; row < stepDownPoses.size(); row++)
      {
         for (RobotSide stepSide : RobotSide.values)
         {
            yPosition = stepSide.negateIfRightSide(stepWidth / 2.0);
            FramePose3D stairPose = stepDownPoses.get(row);
            stairPose.setY(yPosition);

            Point3D stairLocation = new Point3D();
            Quaternion stairOrientation = new Quaternion();
            stairPose.get(stairLocation, stairOrientation);

            FootstepDataMessage stepFootstep = HumanoidMessageTools.createFootstepDataMessage(stepSide, stairLocation, stairOrientation);
            footsteps.getFootstepDataList().add().set(stepFootstep);
         }
      }

      // exit steps
      double stepLength = 0.35;
      int numberOfExitSteps = 2;
      double stepPosition = footsteps.getFootstepDataList().get(footsteps.getFootstepDataList().size() - 1).getLocation().getX();

      robotSide = RobotSide.LEFT;
      for (int stepIndex = 0; stepIndex < numberOfExitSteps; stepIndex++)
      {
         stepPosition += stepLength;
         yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

         FramePose3D exitPose = new FramePose3D();
         exitPose.setX(stepPosition);
         exitPose.setY(yPosition);
         exitPose.setZ(0.0);

         Point3D exitLocation = new Point3D();
         Quaternion exitOrientation = new Quaternion();
         exitPose.get(exitLocation, exitOrientation);
         FootstepDataMessage exitFootstep = HumanoidMessageTools.createFootstepDataMessage(robotSide, exitLocation, exitOrientation);
         footsteps.getFootstepDataList().add().set(exitFootstep);

         robotSide = robotSide.getOppositeSide();
      }

      // closing footstep
      yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

      FramePose3D exitPose = new FramePose3D();
      double exitLocation = footsteps.getFootstepDataList().get(footsteps.getFootstepDataList().size() - 1).getLocation().getX();
      exitPose.setX(exitLocation);
      exitPose.setY(yPosition);

      Point3D exitPosition = new Point3D();
      Quaternion exitOrientation = new Quaternion();
      exitPose.get(exitPosition, exitOrientation);
      FootstepDataMessage exitFootstep = HumanoidMessageTools.createFootstepDataMessage(robotSide, exitPosition, exitOrientation);
      footsteps.getFootstepDataList().add().set(exitFootstep);

      return footsteps;
   }

   private static SideDependentList<List<FramePose3D>> extractColumns(List<List<FramePose3D>> cinderBlockPoses, int indexForLeftSide, int indexForRightSide)
   {
      SideDependentList<Integer> columnIndices = new SideDependentList<Integer>(indexForLeftSide, indexForRightSide);
      SideDependentList<List<FramePose3D>> sideDependentColumns = new SideDependentList<List<FramePose3D>>(new ArrayList<FramePose3D>(),
                                                                                                           new ArrayList<FramePose3D>());

      for (RobotSide robotSide : RobotSide.values)
      {
         int column = columnIndices.get(robotSide);

         for (int row = 0; row < cinderBlockPoses.size(); row++)
            sideDependentColumns.get(robotSide).add(cinderBlockPoses.get(row).get(column));
      }

      return sideDependentColumns;
   }

   private void setupCamera()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void assertReachedGoal(FootstepDataListMessage footsteps)
   {
      int numberOfSteps = footsteps.getFootstepDataList().size();
      Point3D lastStep = footsteps.getFootstepDataList().get(numberOfSteps - 1).getLocation();
      Point3D nextToLastStep = footsteps.getFootstepDataList().get(numberOfSteps - 2).getLocation();

      Point3D midStance = new Point3D();
      midStance.interpolate(lastStep, nextToLastStep, 0.5);

      Point3D midpoint = new Point3D(midStance);
      midpoint.addZ(1.0);

      Point3D bounds = new Point3D(0.25, 0.25, 0.3);

      BoundingBox3D boundingBox3D = BoundingBox3D.createUsingCenterAndPlusMinusVector(midpoint, bounds);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox3D);
   }
}
