package us.ihmc.avatar.obstacleCourseTests;

import org.junit.After;
import org.junit.Before;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.SmallStepDownEnvironment;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;

import static org.junit.Assert.assertTrue;

public abstract class AvatarLeapOfFaithTest implements MultiRobotTestInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry("PointyRocksTest");
   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

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

   public void testUnknownStepDownTwoFeetOnEachStep() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 1.5;
      double transferTime = 0.7;

      int numberOfStepsDown = 2;
      double stepLength = 0.35;
      double stepDownHeight = 0.05;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(numberOfStepsDown, stepLength, stepDownHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(stepDownEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double ankleDamping = 1.0;
      HumanoidFloatingRootJointRobot simulatedRobot = drcSimulationTestHelper.getAvatarSimulation().getHumanoidFloatingRootJointRobot();
      for (RobotSide robotSide : RobotSide.values)
      {
         String firstAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getName();
         String secondAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getName();
         simulatedRobot.getOneDegreeOfFreedomJoint(firstAnkleName).setDamping(ankleDamping);
         simulatedRobot.getOneDegreeOfFreedomJoint(secondAnkleName).setDamping(ankleDamping);
      }

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double executionDuration = 0.0;

      FootstepDataListMessage message = new FootstepDataListMessage(swingTime, transferTime);

      // take care of steps down
      for (int stepNumber = 0; stepNumber < numberOfStepsDown; stepNumber++)
      {
         // step forward
         RobotSide robotSide = RobotSide.LEFT;
         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1) *  stepLength, 0.0, -stepNumber * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.add(footstepData);
         executionDuration += transferTime + swingTime;

         // step in place
         robotSide = robotSide.getOppositeSide();
         stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1) * stepLength, 0.0, -(stepNumber + 1) * stepDownHeight);
         footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.add(footstepData);
         executionDuration += transferTime + swingTime;
      }

      int numberOfClosingSteps = 3;
      RobotSide robotSide = RobotSide.LEFT;
      for (int stepNumber = 0; stepNumber < numberOfClosingSteps; stepNumber++)
      {
         // step forward
         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1 + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.add(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide), (numberOfClosingSteps + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.add(footstepData);
      executionDuration += transferTime + swingTime;


      message.setOffsetFootstepsWithExecutionError(true);
      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * (executionDuration + transferTime));

      assertTrue(success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public void testUnknownStepDownOneFootOnEachStep() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      double swingTime = 1.5;
      double transferTime = 0.7;

      int numberOfStepsDown = 2;
      double stepLength = 0.35;
      double stepDownHeight = 0.05;

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      SmallStepDownEnvironment stepDownEnvironment = new SmallStepDownEnvironment(numberOfStepsDown, stepLength, stepDownHeight);
      drcSimulationTestHelper = new DRCSimulationTestHelper(stepDownEnvironment, "HumanoidPointyRocksTest", selectedLocation, simulationTestingParameters, getRobotModel());

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double ankleDamping = 1.0;
      HumanoidFloatingRootJointRobot simulatedRobot = drcSimulationTestHelper.getAvatarSimulation().getHumanoidFloatingRootJointRobot();
      for (RobotSide robotSide : RobotSide.values)
      {
         String firstAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getName();
         String secondAnkleName = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getName();
         simulatedRobot.getOneDegreeOfFreedomJoint(firstAnkleName).setDamping(ankleDamping);
         simulatedRobot.getOneDegreeOfFreedomJoint(secondAnkleName).setDamping(ankleDamping);
      }

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      double executionDuration = 0.0;

      FootstepDataListMessage message = new FootstepDataListMessage(swingTime, transferTime);

      RobotSide robotSide = RobotSide.LEFT;
      // take care of steps down
      for (int stepNumber = 0; stepNumber < numberOfStepsDown; stepNumber++)
      {
         // step forward
         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1) *  stepLength, 0.0, -stepNumber * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.add(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      int numberOfClosingSteps = 3;
      for (int stepNumber = 0; stepNumber < numberOfClosingSteps; stepNumber++)
      {
         // step forward
         FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide), (stepNumber + 1 + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
         FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
         message.add(footstepData);
         executionDuration += transferTime + swingTime;

         robotSide = robotSide.getOppositeSide();
      }

      // step forward
      FramePoint stepLocation = new FramePoint(fullRobotModel.getSoleFrame(robotSide), (numberOfClosingSteps + numberOfStepsDown) *  stepLength, 0.0, -numberOfStepsDown * stepDownHeight);
      FootstepDataMessage footstepData = createFootstepDataMessage(robotSide, stepLocation);
      message.add(footstepData);
      executionDuration += transferTime + swingTime;


      message.setOffsetFootstepsWithExecutionError(true);
      drcSimulationTestHelper.send(message);

      double timeOverrunFactor = 1.2;
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(timeOverrunFactor * (executionDuration + transferTime));

      assertTrue(success);
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private FootstepDataMessage createFootstepDataMessage(RobotSide robotSide, FramePoint placeToStep)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();

      FramePoint placeToStepInWorld = new FramePoint(placeToStep);
      placeToStepInWorld.changeFrame(worldFrame);

      footstepData.setLocation(placeToStepInWorld.getPointCopy());
      footstepData.setOrientation(new Quaternion(0.0, 0.0, 0.0, 1.0));
      footstepData.setRobotSide(robotSide);

      return footstepData;
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
