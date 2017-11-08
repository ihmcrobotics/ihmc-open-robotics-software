package us.ihmc.commonWalkingControlModules;

import static org.junit.Assert.*;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePose;
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
import us.ihmc.commons.thread.ThreadTools;

public abstract class AvatarStraightLegWalkingTest implements MultiRobotTestInterface
{
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   private static final String forwardFastScript = "scripts/ExerciseAndJUnitScripts/stepAdjustment_forwardWalkingFast.xml";
   private static final String yawScript = "scripts/ExerciseAndJUnitScripts/icpOptimizationPushTestScript.xml";
   private static final String slowStepScript = "scripts/ExerciseAndJUnitScripts/icpOptimizationPushTestScriptSlow.xml";

   private static double simulationTime = 10.0;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @ContinuousIntegrationTest(estimatedDuration =  20.0)
   @Test(timeout = 120000)
   public void testForwardWalking() throws SimulationExceededMaximumTimeException
   {
      setupTest(getLongScript());

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);

      assertTrue(success);
   }

   private void setupTest(String scriptName) throws SimulationExceededMaximumTimeException
   {
      this.setupTest(scriptName, ReferenceFrame.getWorldFrame());
   }

   private void setupTest(String scriptName, ReferenceFrame yawReferenceFrame) throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");

      if (scriptName != null && !scriptName.isEmpty())
      {
         drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.001);
         InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
         if (yawReferenceFrame != null)
         {
            drcSimulationTestHelper.loadScriptFile(scriptInputStream, yawReferenceFrame);
         }
         else
         {
            drcSimulationTestHelper.loadScriptFile(scriptInputStream, ReferenceFrame.getWorldFrame());
         }
      }

      setupCamera();
      ThreadTools.sleep(1000);
   }

   @ContinuousIntegrationTest(estimatedDuration = 167.7)
   @Test(timeout = 840000)
   public void testWalkingOverCinderBlockField() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      CinderBlockFieldEnvironment cinderBlockFieldEnvironment = new CinderBlockFieldEnvironment();
      FootstepDataListMessage footsteps = generateFootstepsForCinderBlockField(cinderBlockFieldEnvironment.getCinderBlockPoses());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(cinderBlockFieldEnvironment);
      drcSimulationTestHelper.createSimulation("EndToEndCinderBlockFieldTest");

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      drcSimulationTestHelper.send(footsteps);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(footsteps.size() * stepTime + 2.0 * initialFinalTransfer + 1.0);
      assertTrue(success);

   }

   @ContinuousIntegrationTest(estimatedDuration = 167.7)
   @Test(timeout = 840000)
   public void testWalkingOverStairs() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      StairsUpAndDownEnvironment stairsEnvironment = new StairsUpAndDownEnvironment();
      FootstepDataListMessage footsteps = generateFootstepsForStairs(stairsEnvironment.getStairPoses());
      //footsteps.setDefaultTransferDuration(0.5);
      //footsteps.setDefaultSwingDuration(1.0);

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stairsEnvironment);
      drcSimulationTestHelper.createSimulation("EndToEndCinderBlockFieldTest");

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);
      assertTrue(success);

      drcSimulationTestHelper.send(footsteps);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double stepTime = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
      double initialFinalTransfer = walkingControllerParameters.getDefaultInitialTransferTime();

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(footsteps.size() * stepTime + 2.0 * initialFinalTransfer + 1.0);
      //success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10 * stepTime + 2.0 * initialFinalTransfer + 1.0);
      assertTrue(success);

   }

   public void testSlowerWalking() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.001);

      int numberOfSteps = 6;
      double stepLength = 0.35;
      double transferDuration = 0.75;
      double swingDuration = 1.25;

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      setupCamera();

      ThreadTools.sleep(1000);

      FootstepDataListMessage message = new FootstepDataListMessage();
      RobotSide robotSide = RobotSide.LEFT;

      double distanceTraveled = 0.5 * stepLength;
      double stepHeight = 0.95;

      // take care of falling steps
      for (int stepNumber = 0; stepNumber < numberOfSteps; stepNumber++)
      {
         // step forward
         distanceTraveled += stepLength;

         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, stepHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.add(footstepData);

         robotSide = robotSide.getOppositeSide();
      }

      // closing step
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, stepHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.add(footstepData);

      message.setDefaultTransferDuration(transferDuration);
      message.setDefaultSwingDuration(swingDuration);

      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * message.footstepDataList.size() * 2.0);

      assertTrue(success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testDropOffsWhileWalking(double stepDownHeight) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

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
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
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
         message.add(footstepData);

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
         message.add(footstepData);

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, stepHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.add(footstepData);
      //message.setOffsetFootstepsWithExecutionError(true);

      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * message.footstepDataList.size() * 2.0);

      assertTrue(success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testSteppingDown(double stepDownHeight, double stepLength, int stepsBeforeDrop) throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

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
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
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
         message.add(footstepData);


         robotSide = robotSide.getOppositeSide();
      }

      int numberOfClosingSteps = 3;
      for (int stepNumber = 0; stepNumber < numberOfClosingSteps; stepNumber++)
      {
         // step forward
         distanceTraveled += stepLength;
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, stepHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.add(footstepData);

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * stepLength, 0.0, stepHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.add(footstepData);
      //message.setOffsetFootstepsWithExecutionError(true);

      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * message.footstepDataList.size() * 2.0);

      assertTrue(success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testRandomHeightField(double maxStepHeight, double minStepHeight, double maxStepIncrease) throws SimulationExceededMaximumTimeException
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
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(stepDownEnvironment);
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
         message.add(footstepData);

         robotSide = robotSide.getOppositeSide();
      }

      int numberOfClosingSteps = 3;
      for (int stepNumber = 0; stepNumber < numberOfClosingSteps; stepNumber++)
      {
         // step forward
         distanceTraveled += endingStepLength;
         FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * endingStepLength, 0.0, 0.0);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.add(footstepData);

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint3D stepLocation = new FramePoint3D(fullRobotModel.getSoleFrame(robotSide), distanceTraveled - 0.5 * endingStepLength, 0.0, 0.0);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.add(footstepData);

      //message.setOffsetFootstepsWithExecutionError(true);
      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * message.footstepDataList.size() * 2.0);

      assertTrue(success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }


   private FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, FramePoint3D placeToStep)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();

      FramePoint3D placeToStepInWorld = new FramePoint3D(placeToStep);
      placeToStepInWorld.changeFrame(worldFrame);

      footstepData.setLocation(placeToStepInWorld);
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide);

      return footstepData;
   }

   private static FootstepDataListMessage generateFootstepsForCinderBlockField(List<List<FramePose>> cinderBlockPoses)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      int numberOfColumns = cinderBlockPoses.get(0).size();

      int indexForLeftSide = (numberOfColumns - 1) / 2;
      int indexForRightSide = indexForLeftSide + 1;
      SideDependentList<List<FramePose>> columns = extractColumns(cinderBlockPoses, indexForLeftSide, indexForRightSide);

      for (int row = 0; row < cinderBlockPoses.size(); row++)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            FramePose cinderBlockPose = columns.get(robotSide).get(row);
            Point3D location = new Point3D();
            Quaternion orientation = new Quaternion();
            cinderBlockPose.getPose(location, orientation);
            location.setZ(location.getZ() + 0.02);
            FootstepDataMessage footstep = new FootstepDataMessage(robotSide, location, orientation);
            footsteps.add(footstep);
         }
      }

      return footsteps;
   }

   private static FootstepDataListMessage generateFootstepsForStairs(List<List<FramePose>> stepPoses)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      double stepWidth = 0.275;
      int numberOfStartingSteps = 3;
      double firstStepPosition = stepPoses.get(0).get(0).getX();
      double startingLength = (firstStepPosition - 0.35)  / numberOfStartingSteps;

      // approach the stairs
      RobotSide robotSide = RobotSide.LEFT;
      for (int stepIndex = 0; stepIndex < numberOfStartingSteps; stepIndex++)
      {
            double yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

            FramePose stepPose = new FramePose();
            stepPose.setX(startingLength * (stepIndex + 1));
            stepPose.setY(yPosition);

            Point3D location = new Point3D();
            Quaternion orientation = new Quaternion();
            stepPose.getPose(location, orientation);
            FootstepDataMessage footstep = new FootstepDataMessage(robotSide, location, orientation);
            footsteps.add(footstep);

         robotSide = robotSide.getOppositeSide();
      }

      // closing step
      double yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

      FramePose stepPose = new FramePose();
      stepPose.setX(startingLength * numberOfStartingSteps);
      stepPose.setY(yPosition);

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      stepPose.getPose(location, orientation);
      FootstepDataMessage footstep = new FootstepDataMessage(robotSide, location, orientation);
      footsteps.add(footstep);

      // ascend the stairs
      List<FramePose> stepUpPoses = stepPoses.get(0);
      for (int row = 0; row < stepUpPoses.size(); row++)
      {
         for (RobotSide stepSide : RobotSide.values)
         {
            yPosition = stepSide.negateIfRightSide(stepWidth / 2.0);
            FramePose stairPose = stepUpPoses.get(row);
            stairPose.setY(yPosition);

            Point3D stairLocation = new Point3D();
            Quaternion stairOrientation = new Quaternion();
            stairPose.getPose(stairLocation, stairOrientation);

            FootstepDataMessage stepFootstep = new FootstepDataMessage(stepSide, stairLocation, stairOrientation);
            footsteps.add(stepFootstep);
         }
      }

      FramePose lastStepGoingUpStairs = stepPoses.get(0).get(stepPoses.get(0).size() - 1);
      double topOfStairs = lastStepGoingUpStairs.getX();
      double startOfAscent = stepPoses.get(1).get(0).getX() - 0.35;
      double platformWidth = startOfAscent - topOfStairs;

      int numberOfPlatformSteps = 1;
      double platformStepLength = (platformWidth)  / numberOfPlatformSteps;

      // approach the top of the stairs
      robotSide = RobotSide.LEFT;
      for (int stepIndex = 0; stepIndex < numberOfPlatformSteps; stepIndex++)
      {
         yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

         FramePose landingPose = new FramePose();
         landingPose.setX(platformStepLength * (stepIndex + 1) + topOfStairs);
         landingPose.setY(yPosition);
         landingPose.setZ(lastStepGoingUpStairs.getZ());

         Point3D stepLocation = new Point3D();
         Quaternion stepOrientation = new Quaternion();
         landingPose.getPose(stepLocation, stepOrientation);
         FootstepDataMessage stepMessage = new FootstepDataMessage(robotSide, stepLocation, stepOrientation);
         footsteps.add(stepMessage);

         robotSide = robotSide.getOppositeSide();
      }


      // closing step
      yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

      FramePose landingPose = new FramePose();
      double forwardLocation = footsteps.get(footsteps.size() - 1).getLocation().getX();
      landingPose.setX(forwardLocation);
      landingPose.setY(yPosition);
      landingPose.setZ(lastStepGoingUpStairs.getZ());

      Point3D stepLocation = new Point3D();
      Quaternion stepOrientation = new Quaternion();
      landingPose.getPose(stepLocation, stepOrientation);
      FootstepDataMessage landingMessage = new FootstepDataMessage(robotSide, stepLocation, stepOrientation);
      footsteps.add(landingMessage);

      // descend the stairs
      List<FramePose> stepDownPoses = stepPoses.get(1);
      for (int row = 0; row < stepDownPoses.size(); row++)
      {
         for (RobotSide stepSide : RobotSide.values)
         {
            yPosition = stepSide.negateIfRightSide(stepWidth / 2.0);
            FramePose stairPose = stepDownPoses.get(row);
            stairPose.setY(yPosition);

            Point3D stairLocation = new Point3D();
            Quaternion stairOrientation = new Quaternion();
            stairPose.getPose(stairLocation, stairOrientation);

            FootstepDataMessage stepFootstep = new FootstepDataMessage(stepSide, stairLocation, stairOrientation);
            footsteps.add(stepFootstep);
         }
      }

      // exit steps
      double stepLength = 0.35;
      int numberOfExitSteps = 2;
      double stepPosition = footsteps.get(footsteps.size() - 1).getLocation().getX();

      robotSide = RobotSide.LEFT;
      for (int stepIndex = 0; stepIndex < numberOfExitSteps; stepIndex++)
      {
         stepPosition += stepLength;
         yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

         FramePose exitPose = new FramePose();
         exitPose.setX(stepPosition);
         exitPose.setY(yPosition);
         exitPose.setZ(0.0);

         Point3D exitLocation = new Point3D();
         Quaternion exitOrientation = new Quaternion();
         exitPose.getPose(exitLocation, exitOrientation);
         FootstepDataMessage exitFootstep = new FootstepDataMessage(robotSide, exitLocation, exitOrientation);
         footsteps.add(exitFootstep);

         robotSide = robotSide.getOppositeSide();
      }

      // closing footstep
      yPosition = robotSide.negateIfRightSide(stepWidth / 2.0);

      FramePose exitPose = new FramePose();
      double exitLocation = footsteps.get(footsteps.size() - 1).getLocation().getX();
      exitPose.setX(exitLocation);
      exitPose.setY(yPosition);

      Point3D exitPosition = new Point3D();
      Quaternion exitOrientation = new Quaternion();
      exitPose.getPose(exitPosition, exitOrientation);
      FootstepDataMessage exitFootstep = new FootstepDataMessage(robotSide, exitPosition, exitOrientation);
      footsteps.add(exitFootstep);

      return footsteps;
   }

   private static SideDependentList<List<FramePose>> extractColumns(List<List<FramePose>> cinderBlockPoses, int indexForLeftSide, int indexForRightSide)
   {
      SideDependentList<Integer> columnIndices = new SideDependentList<Integer>(indexForLeftSide, indexForRightSide);
      SideDependentList<List<FramePose>> sideDependentColumns = new SideDependentList<List<FramePose>>(new ArrayList<FramePose>(), new ArrayList<FramePose>());

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

   public String getLongScript()
   {
      return forwardFastScript;
   }
}
