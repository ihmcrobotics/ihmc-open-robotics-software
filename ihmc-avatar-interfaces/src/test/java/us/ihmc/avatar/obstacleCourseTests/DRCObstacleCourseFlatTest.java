package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.IOException;
import java.io.InputStream;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.data.SpaceData3D;
import us.ihmc.commonWalkingControlModules.controllerCore.data.Type;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameSE3TrajectoryPointList;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationDoneCriterion;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.tools.YoGeometryNameTools;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class DRCObstacleCourseFlatTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Disabled
   @Test
   public void testForMemoryLeaks() throws Exception
   {
      for (int i = 0; i < 10; i++)
      {
         showMemoryUsageBeforeTest();
         testStandingForACoupleSeconds();
         destroySimulationAndRecycleMemory();
      }
   }

   @Test
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation("DRCStandingTest");

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      ThreadTools.sleep(2000);

      // drcSimulationTestHelper.createVideo(getSimpleRobotName(), simulationConstructionSet, 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-8.956281888358388E-4, -3.722237566790175E-7, 0.8882009563211146);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation("DRCStandingTest");

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      YoDouble offsetHeightAboveGround = (YoDouble) drcSimulationTestHelper.getSimulationConstructionSet().findVariable("HeightOffsetHandler",
                                                                                                                       "offsetHeightAboveGround");
      offsetHeightAboveGround.set(0.15);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      offsetHeightAboveGround.set(0.30);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      offsetHeightAboveGround.set(0.50);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      assertTrue(success);

      ThreadTools.sleep(2000);

      assertTrue(success);

      Point3D center = new Point3D(-8.956281888358388E-4, -3.722237566790175E-7, 0.8882009563211146);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testSimpleScripts() throws SimulationExceededMaximumTimeException, IOException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      String name = "DRCSimpleScriptsTest";

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(name);
      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1); //1.0);

      FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      ReferenceFrame leftSoleFrame = controllerFullRobotModel.getSoleFrame(RobotSide.LEFT);
      ReferenceFrame rightSoleFrame = controllerFullRobotModel.getSoleFrame(RobotSide.RIGHT);

      FramePoint3D leftSole = new FramePoint3D(leftSoleFrame);
      leftSole.changeFrame(ReferenceFrame.getWorldFrame());
      System.out.println("leftSole = " + leftSole);

      String scriptName = "scripts/ExerciseAndJUnitScripts/SimpleSingleStepScript.xml";
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, leftSoleFrame);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0);

      scriptName = "scripts/ExerciseAndJUnitScripts/SimpleSingleHandTrajectoryScript.xml";
      scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, leftSoleFrame);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      scriptName = "scripts/ExerciseAndJUnitScripts/SimpleSingleFootTrajectoryScript.xml";
      scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, rightSoleFrame);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      scriptName = "scripts/ExerciseAndJUnitScripts/SimpleSinglePelvisHeightScript.xml";
      scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, leftSoleFrame);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.24, 0.18, 0.8358344340816537);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testACoupleStepsUsingQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String name = "DRCQueuedControllerCommandTest";

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(name);
      ConcurrentLinkedQueue<Command<?, ?>> queuedControllerCommands = drcSimulationTestHelper.getQueuedControllerCommands();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      //	      YoBoolean walk = (YoBoolean) robot.getVariable("walkCSG");
      //	      walk.set(true);

      FootstepDataListCommand footstepList = new FootstepDataListCommand();
      FootstepDataCommand footstepCommand = new FootstepDataCommand();

      Point3D position = new Point3D(0.0, 0.2, 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3D(0.3, -0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3D(0.8, 0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3D(0.8, -0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      queuedControllerCommands.add(footstepList);

      double perStepDuration = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime()
            + getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();
      double simulationDuration = footstepList.getNumberOfFootsteps() * perStepDuration
            + getRobotModel().getWalkingControllerParameters().getDefaultFinalTransferTime()
            + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime();

      footstepList = new FootstepDataListCommand();
      footstepCommand = new FootstepDataCommand();

      position = new Point3D(1.0, 0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3D(1.3, -0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3D(1.8, 0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3D(1.8, -0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      queuedControllerCommands.add(footstepList);

      perStepDuration += getRobotModel().getWalkingControllerParameters().getDefaultSwingTime()
            + getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();
      simulationDuration += footstepList.getNumberOfFootsteps() * perStepDuration
            + getRobotModel().getWalkingControllerParameters().getDefaultFinalTransferTime()
            + getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime();

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationDuration + 1.5);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.8, 0.0, 0.78);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testACoupleQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String name = "DRCQueuedControllerCommandTest";

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(name);
      ConcurrentLinkedQueue<Command<?, ?>> queuedControllerCommands = drcSimulationTestHelper.getQueuedControllerCommands();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25);

      //      YoBoolean walk = (YoBoolean) robot.getVariable("walkCSG");
      //      walk.set(true);

      FootstepDataListCommand footstepList = new FootstepDataListCommand();
      FootstepDataCommand footstepCommand = new FootstepDataCommand();

      Point3D position = new Point3D(0.3, 0.2, 0.0);
      Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3D(0.3, -0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      queuedControllerCommands.add(footstepList);

      // Some chest motions. These will continue during the steps to come afterwards:
      ChestTrajectoryCommand chestCommand = new ChestTrajectoryCommand();
      FrameSO3TrajectoryPointList chestTrajectoryPointList = new FrameSO3TrajectoryPointList();
      chestTrajectoryPointList.addTrajectoryPoint(0.0, new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D());
      chestTrajectoryPointList.addTrajectoryPoint(1.0, new Quaternion(0.2, 0.0, 0.0, 1.0), new Vector3D());
      chestTrajectoryPointList.addTrajectoryPoint(2.0, new Quaternion(-0.2, 0.0, 0.0, 1.0), new Vector3D());
      chestTrajectoryPointList.addTrajectoryPoint(3.0, new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D());
      chestCommand.getSO3Trajectory().setTrajectoryPointList(chestTrajectoryPointList);
      chestCommand.getSO3Trajectory().setTrajectoryFrame(ReferenceFrame.getWorldFrame());
      queuedControllerCommands.add(chestCommand);

      // Some more steps:
      footstepList = new FootstepDataListCommand();
      footstepCommand = new FootstepDataCommand();

      position = new Point3D(0.65, 0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3D(0.65, -0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3D(1.1, 0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.LEFT);
      footstepList.addFootstep(footstepCommand);

      position = new Point3D(1.1, -0.2, 0.0);
      orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      footstepCommand.setPose(position, orientation);
      footstepCommand.setRobotSide(RobotSide.RIGHT);
      footstepList.addFootstep(footstepCommand);

      queuedControllerCommands.add(footstepList);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(9.0);

      FootTrajectoryCommand footTrajectoryCommand = new FootTrajectoryCommand();

      FrameSE3TrajectoryPointList footPointList = new FrameSE3TrajectoryPointList();
      footPointList.addTrajectoryPoint(0.2, new Point3D(1.1, -0.2, 0.25), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
      footPointList.addTrajectoryPoint(0.5, new Point3D(1.1, -0.2, 0.35), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
      footPointList.addTrajectoryPoint(1.0, new Point3D(1.1, -0.2, 0.25), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
      footPointList.addTrajectoryPoint(2.0, new Point3D(1.1, -0.2, 0.35), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());

      footTrajectoryCommand.getSE3Trajectory().setTrajectoryPointList(footPointList);
      footTrajectoryCommand.setRobotSide(RobotSide.RIGHT);
      footTrajectoryCommand.getSE3Trajectory().setTrajectoryFrame(ReferenceFrame.getWorldFrame());
      queuedControllerCommands.add(footTrajectoryCommand);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.1, 0.22, 0.78);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testACoupleMoreQueuedControllerCommands() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String name = "DRCQueuedControllerCommandTest";

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(name);
      ConcurrentLinkedQueue<Command<?, ?>> queuedControllerCommands = drcSimulationTestHelper.getQueuedControllerCommands();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25);

      //      YoBoolean walk = (YoBoolean) robot.getVariable("walkCSG");
      //      walk.set(true);

      FootTrajectoryCommand footTrajectoryCommand = new FootTrajectoryCommand();

      FrameSE3TrajectoryPointList footPointList = new FrameSE3TrajectoryPointList();
      footPointList.addTrajectoryPoint(1.0, new Point3D(0.0, -0.2, 0.25), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
      footPointList.addTrajectoryPoint(2.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
      footPointList.addTrajectoryPoint(3.0, new Point3D(0.0, -0.2, 0.25), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
      footPointList.addTrajectoryPoint(4.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());

      footTrajectoryCommand.getSE3Trajectory().setTrajectoryPointList(footPointList);
      footTrajectoryCommand.setRobotSide(RobotSide.RIGHT);
      queuedControllerCommands.add(footTrajectoryCommand);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);

      footTrajectoryCommand = new FootTrajectoryCommand();

      footPointList = new FrameSE3TrajectoryPointList();
      footPointList.addTrajectoryPoint(1.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.1, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
      footPointList.addTrajectoryPoint(2.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.0, 0.1, 0.0, 1.0), new Vector3D(), new Vector3D());
      footPointList.addTrajectoryPoint(3.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.0, 0.0, 0.1, 1.0), new Vector3D(), new Vector3D());
      footPointList.addTrajectoryPoint(4.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.0, 0.1, 0.0, 1.0), new Vector3D(), new Vector3D());

      footTrajectoryCommand.getSE3Trajectory().setTrajectoryPointList(footPointList);
      footTrajectoryCommand.setRobotSide(RobotSide.RIGHT);
      queuedControllerCommands.add(footTrajectoryCommand);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-0.1, 0.18, 0.78);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void loadScriptFileInLeftSoleFrame(String scriptName)
   {
      FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      ReferenceFrame leftSoleFrame = controllerFullRobotModel.getSoleFrame(RobotSide.LEFT);
      InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
      drcSimulationTestHelper.loadScriptFile(scriptInputStream, leftSoleFrame);
   }

   public void testLongStepsMaxHeightPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation("DRCLongStepsMaxHeightPauseAndRestartTest");
      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      String scriptName = "scripts/ExerciseAndJUnitScripts/LongStepsMaxHeightPauseAndRestart_LeftFootTest.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      success = success & drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(2.36472504931194, 0.012458249442189283, 0.7892313252995141);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testStandingOnUnevenTerrainForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.TOP_OF_SLOPES;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCStandingTest");

      Point3D cameraFix = new Point3D(3.25, 3.25, 1.02);
      Point3D cameraPosition = new Point3D(6.35, 0.18, 0.97);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   // @Test
   // public void testMemoryStuff()
   // {
   // for (int i=0; i<3; i++)
   // {
   // System.gc();
   // System.runFinalization();
   // ThreadTools.sleep(1000);
   //
   // System.out.println("Sleeping Forever");
   // ThreadTools.sleepForever();
   // }
   // }

   // Added for fixing DRC-866. Does not work for fast walking
   @Test
   public void testRotatedStepInTheAir() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation("DRCRotatedStepsInTheAirTest");

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      FootstepDataListMessage footstepDataList = createFootstepsForRotatedStepInTheAir(scriptedFootstepGenerator);
      drcSimulationTestHelper.publishToController(footstepDataList);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();

      double stepDuration = walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultInitialTransferTime()
            + walkingControllerParameters.getDefaultFinalTransferTime();

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(stepDuration + 1.5);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingUpToRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation("DRCWalkingUpToRampShortStepsTest");

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingUpToRampShortSteps(scriptedFootstepGenerator);
      drcSimulationTestHelper.publishToController(footstepDataList);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(15.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(3.281440097950577, 0.08837997229569997, 0.7855496116044516);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testRepeatedWalking() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, new FlatGroundEnvironment());
      drcSimulationTestHelper.createSimulation("DRCWalkingOccasionallyStraightKneesTest");

      Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(-7.0, 0.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);

      ThreadTools.sleep(1000);
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));

      SideDependentList<MovingReferenceFrame> soleFrames = drcSimulationTestHelper.getReferenceFrames().getSoleFrames();
      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      for (RobotSide side : RobotSide.values)
      {
         FootstepDataMessage footstep = footsteps.getFootstepDataList().add();
         FramePose3D pose = new FramePose3D(soleFrames.get(side));
         pose.changeFrame(ReferenceFrame.getWorldFrame());
         footstep.getLocation().set(pose.getPosition());
         footstep.getOrientation().set(pose.getOrientation());
         footstep.setRobotSide(side.toByte());
      }

      WalkingControllerParameters parameters = robotModel.getWalkingControllerParameters();
      double initialTransferDuration = parameters.getDefaultInitialTransferTime();
      double swingDuration = parameters.getDefaultSwingTime();
      double transferDuration = parameters.getDefaultTransferTime();
      double finalTransferDuration = parameters.getDefaultFinalTransferTime();
      double duration = initialTransferDuration + transferDuration + 2.0 * swingDuration + finalTransferDuration;

      drcSimulationTestHelper.publishToController(footsteps);
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(duration - 0.7 * finalTransferDuration));

      double vy_bef = drcSimulationTestHelper.getYoVariable("desiredICPVelocityY").getValueAsDouble();

      drcSimulationTestHelper.publishToController(footsteps);
      Assert.assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(transferDuration / 10.0));

      double vy_aft = drcSimulationTestHelper.getYoVariable("desiredICPVelocityY").getValueAsDouble();
      Assert.assertEquals(Math.signum(vy_bef), Math.signum(vy_aft));

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.createSimulation("DRCWalkingOccasionallyStraightKneesTest");

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingOnFlatLongSteps(scriptedFootstepGenerator);

      // FootstepDataList footstepDataList = createFootstepsForTwoLongFlatSteps(scriptedFootstepGenerator);
      drcSimulationTestHelper.publishToController(footstepDataList);
      //      drcSimulationTestHelper.send(new ComHeightPacket(0.08, 1.0));

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(8.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(3.1200570722246437, 0.017275273114368033, 0.8697236867426688);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testTurningInPlaceAndPassingPI() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCTurningInPlaceAndPassingPITest");

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForTurningInPlaceAndPassingPI();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForTurningInPlaceAndPassingPI(scriptedFootstepGenerator);
      drcSimulationTestHelper.publishToController(footstepDataList);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      final YoDouble pelvisOrientationError = getPelvisOrientationErrorVariableName(simulationConstructionSet, fullRobotModel);

      SimulationDoneCriterion checkPelvisOrientationError = new SimulationDoneCriterion()
      {
         @Override
         public boolean isSimulationDone()
         {
            double errorMag = Math.abs(pelvisOrientationError.getDoubleValue());
            boolean largeError = errorMag > 0.15;
            if (largeError)
               PrintTools.error(DRCObstacleCourseFlatTest.class, "Large pelvis orientation error, stopping sim. Error magnitude: " + errorMag);
            return largeError;
         }
      };

      simulationConstructionSet.setSimulateDoneCriterion(checkPelvisOrientationError);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(15.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.125, 0.03, 0.78);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupCameraForWalkingUpToRamp()
   {
      Point3D cameraFix = new Point3D(1.8375, -0.16, 0.89);
      Point3D cameraPosition = new Point3D(1.10, 8.30, 1.37);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private void setupCameraForTurningInPlaceAndPassingPI()
   {
      Point3D cameraFix = new Point3D(0.036, 0.0, 0.89);
      Point3D cameraPosition = new Point3D(-7, -0.3575, 1.276);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForRotatedStepInTheAir(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][] {{{0.4, 0.10, 0.10}, {0.4, 0.0, 0.0, 0.8}},
            {{0.48, -0.10329823409587219, 0.08400000000000005}, {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}}};

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataListMessage createFootstepsForWalkingUpToRampShortSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][] {
            {{0.2148448504580547, -0.09930268518393547, 0.08399999999999999},
                  {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{0.4481532647842352, 0.10329823409587219, 0.08400000000000005},
                  {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{0.6834821762408051, -0.09551979778612019, 0.08399999999999999},
                  {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{0.9167977582017036, 0.10565710343022289, 0.08400000000000005},
                  {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{1.1521266696582735, -0.09316092845176947, 0.08399999999999999},
                  {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{1.385442251619172, 0.1080159727645736, 0.08400000000000005},
                  {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{1.620771163075742, -0.09080205911741877, 0.08399999999999999},
                  {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{1.8540867450366407, 0.11037484209892431, 0.08400000000000005},
                  {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{2.0894156564932107, -0.08844318978306806, 0.08399999999999999},
                  {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{2.322731238454109, 0.11273371143327501, 0.08400000000000005},
                  {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{2.558060149910679, -0.08608432044871735, 0.08398952447640476},
                  {-5.047008501650524E-21, 4.53358964226292E-22, 0.0025166698394258787, 0.9999968331814453}},
            {{2.7913757318715775, 0.11509258076762573, 0.08400000000000005},
                  {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{3.0267046433281477, -0.08372545111436663, 0.08398952447640476},
                  {-6.38257081820882E-21, -2.5377866560433405E-20, 0.0025166698394258787, 0.9999968331814453}},
            {{3.260020225289046, 0.11745145010197644, 0.08400000000000005},
                  {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}},
            {{3.2610268900368817, -0.08254601644719128, 0.08398952447640476},
                  {3.49577202412201E-21, 2.923107094657073E-20, 0.0025166698394258787, 0.9999968331814453}}};

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataListMessage createFootstepsForWalkingOnFlatLongSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][] {
            {{0.5909646234016005, 0.10243127081250579, 0.08400000000000002},
                  {3.5805394102331502E-22, -1.0841962601668662E-19, 0.003302464707320093, 0.99999454684856}},
            {{1.212701966120992, -0.09394691394679651, 0.084}, {1.0806157207566333E-19, 1.0877767995770995E-19, 0.0033024647073200924, 0.99999454684856}},
            {{1.8317941784239657, 0.11014657591704705, 0.08619322927296164},
                  {8.190550851520344E-19, 1.5693991726842814E-18, 0.003302464707320093, 0.99999454684856}},
            {{2.4535283480857237, -0.08575120920059497, 0.08069788195751608},
                  {-2.202407644730947E-19, -8.117149793610565E-19, 0.0033024647073200924, 0.99999454684856}},
            {{3.073148474156348, 0.11833676240086898, 0.08590468550531082},
                  {4.322378465953267E-5, 0.003142233766871708, 0.0033022799833692306, 0.9999896096688056}},
            {{3.0729346702590505, -0.0816428320664241, 0.0812390388356},
                  {-8.243740658642556E-5, -0.005993134849034999, 0.003301792738040525, 0.999976586577641}}};

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataListMessage createFootstepsForTurningInPlaceAndPassingPI(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][] {
            {{0.053884346896697966, 0.19273164589134978, 0.08574185103923426},
                  {-6.938862977443471E-11, -8.7126898825953E-11, 0.9990480941331229, 0.04362230632342559}},
            {{0.05388201845443364, -0.20574623329424319, 0.08574185073944539},
                  {1.6604742582112774E-10, 1.4170466407843545E-10, 0.9990483490180827, -0.04361646849807009}},
            {{0.0017235494647287533, 0.19045456181341558, 0.08574185040535603},
                  {-5.0383363690493444E-11, -1.0843741493223105E-10, 0.9961949527487116, -0.0871528319562377}},
            {{0.10485496441611886, -0.19444611557725083, 0.08574185102571344},
                  {1.5201027889830733E-10, 1.4860298371617872E-10, 0.9848082603764649, -0.1736453002366632}},
            {{-0.04807055917333275, 0.17475485972777594, 0.08574185070322422},
                  {-3.05160242173266E-11, -1.2789253687750615E-10, 0.976296639469044, -0.21643676157587363}},
            {{0.15116636401480588, -0.17033827066486662, 0.08574185038049925},
                  {1.3537219389951473E-10, 1.5295866511692108E-10, 0.9537178292633579, -0.3007030131960579}},
            {{-0.09210459251806524, 0.14670244796138915, 0.08574185100111767},
                  {-1.0126547178247246E-11, -1.4515938198837407E-10, 0.9396936200915386, -0.3420173977435346}},
            {{0.18966017152321202, -0.13506560904726644, 0.0857418506668508},
                  {1.1641785319333712E-10, 1.5469718133894557E-10, 0.9063090217942931, -0.42261561378428936}},
            {{-0.12737770450507258, 0.10820905279560836, 0.08574185036731347},
                  {-1.0436197890210116E-11, 1.599425098341044E-10, -0.8870121823838428, 0.46174602142590504}},
            {{0.21771309767509972, -0.09103190305599193, 0.08574185095383173},
                  {-9.547157074708167E-11, -1.5378878590499154E-10, -0.8433930157263759, 0.5372971440683164}},
            {{-0.15148609051286105, 0.061897935068802395, 0.085741850664082},
                  {-3.082037679075772E-11, 1.7198897704022203E-10, -0.8191537200820054, 0.573574043063153}},
            {{0.23341338156809216, -0.0412379781596809, 0.08574185031046283},
                  {-7.289174317616828E-11, -1.5024902169235376E-10, -0.7660463210652019, 0.6427853716307409}},
            {{-0.16278680351003783, 0.010925120900156002, 0.08574185095977704},
                  {-5.067721042808702E-11, 1.8109266512133938E-10, -0.7372793107685568, 0.6755880534117237}},
            {{0.23569107567555475, 0.010922792383292988, 0.08574185059966628},
                  {-4.906471775149843E-11, -1.441384550795834E-10, -0.6755923617465286, 0.7372753629070672}},
            {{-0.1605097194824509, -0.0412356760683499, 0.08574185032282551},
                  {-6.966694301257708E-11, 1.87097807520974E-10, -0.6427898477118872, 0.766042565187163}},
            {{0.20979454839765582, 0.013396779318557463, 0.08574185088931394},
                  {-2.91671807071375E-11, -1.3694134194254838E-10, -0.5937694707170026, 0.804635206565342}},
            {{-0.0496373406094997, -0.06666317759167362, 0.08574185062507425},
                  {-7.826318574113734E-11, 1.8865011916447275E-10, -0.5937694705296589, 0.8046352067035897}}};

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private YoDouble getPelvisOrientationErrorVariableName(SimulationConstructionSet scs, FullHumanoidRobotModel fullRobotModel)
   {
      String pelvisName = fullRobotModel.getPelvis().getName();
      String namePrefix = pelvisName + Type.ERROR.getName() + SpaceData3D.ROTATION_VECTOR.getName();
      String varName = YoGeometryNameTools.createZName(namePrefix, "");
      return (YoDouble) scs.findVariable(FeedbackControllerToolbox.class.getSimpleName(), varName);
   }
}
