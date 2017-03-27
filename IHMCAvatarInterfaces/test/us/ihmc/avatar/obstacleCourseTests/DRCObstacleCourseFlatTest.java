package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.*;

import java.io.IOException;
import java.io.InputStream;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.avatar.testTools.ScriptedHandstepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage.FootstepOrigin;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointList;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPointList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationToolkit.controllers.OscillateFeetPerturber;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationDoneCriterion;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCObstacleCourseFlatTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   // Invoked manually to test memory & thread leaks
	@ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.MANUAL)
	@Test(timeout=300000)
   public void testForMemoryLeaks() throws Exception
   {
      for (int i = 0; i < 10; i++)
      {
         showMemoryUsageBeforeTest();
         testStandingForACoupleSeconds();
         destroySimulationAndRecycleMemory();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 15.3)
	@Test(timeout = 77000)
   public void testStandingForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCStandingTest", selectedLocation, simulationTestingParameters, getRobotModel());

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

	@ContinuousIntegrationTest(estimatedDuration = 19.0)
	@Test(timeout = 95000)
   public void testStandingTooHighToCheckIfSingularityStuffIsWorkingProperly() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCStandingTest", selectedLocation, simulationTestingParameters, getRobotModel());

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      DoubleYoVariable offsetHeightAboveGround = (DoubleYoVariable) drcSimulationTestHelper.getSimulationConstructionSet().getVariable("LookAheadCoMHeightTrajectoryGenerator", "offsetHeightAboveGround");
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

	@ContinuousIntegrationTest(estimatedDuration = 39.4)
	@Test(timeout = 200000)
   public void testSimpleFlatGroundScriptWithRandomFootSlip() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      String name = "DRCSimpleFlatGroundScriptTest";

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, name, selectedLocation, simulationTestingParameters, getRobotModel());
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      setupCameraForWalkingUpToRamp();
      SlipRandomOnNextStepPerturber slipRandomOnEachStepPerturber = new SlipRandomOnNextStepPerturber(robot, 1002L);
      slipRandomOnEachStepPerturber.setTranslationRangeToSlipNextStep(new double[]{0.01, 0.01, 0.0}, new double[]{0.06, 0.06, 0.005});
      slipRandomOnEachStepPerturber.setRotationRangeToSlipNextStep(new double[]{0.03, 0.0, 0.0}, new double[]{0.3, 0.0, 0.0});
      slipRandomOnEachStepPerturber.setSlipAfterStepTimeDeltaRange(0.005, 0.25);
      slipRandomOnEachStepPerturber.setSlipPercentSlipPerTickRange(0.005, 0.5);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.0);

      robot.setController(slipRandomOnEachStepPerturber, 10);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.5);

      String scriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";
      loadScriptFileInLeftSoleFrame(scriptName);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(16.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.2315617729419353, 0.14530717103231391, 0.8358344340816537);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	  @ContinuousIntegrationTest(estimatedDuration = 39.9)
	   @Test(timeout = 200000)
	   public void testSimpleScripts() throws SimulationExceededMaximumTimeException, IOException
	   {
	      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
	      String name = "DRCSimpleScriptsTest";

	      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
	      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

	      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, name, selectedLocation, simulationTestingParameters, getRobotModel());
	      setupCameraForWalkingUpToRamp();

	      ThreadTools.sleep(1000);
	      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1); //1.0);


	      FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
         ReferenceFrame leftSoleFrame = controllerFullRobotModel.getSoleFrame(RobotSide.LEFT);
         ReferenceFrame rightSoleFrame = controllerFullRobotModel.getSoleFrame(RobotSide.RIGHT);


	      FramePoint leftSole = new FramePoint(leftSoleFrame);
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


	  @ContinuousIntegrationTest(estimatedDuration = 29.1)
	   @Test(timeout = 150000)
	   public void testACoupleStepsUsingQueuedControllerCommands() throws SimulationExceededMaximumTimeException
	   {
	      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

	      String name = "DRCQueuedControllerCommandTest";

	      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
	      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

	      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, name, selectedLocation, simulationTestingParameters, getRobotModel());
	      ConcurrentLinkedQueue<Command<?, ?>> queuedControllerCommands = drcSimulationTestHelper.getQueuedControllerCommands();

	      setupCameraForWalkingUpToRamp();

	      ThreadTools.sleep(1000);
	      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

//	      BooleanYoVariable walk = (BooleanYoVariable) robot.getVariable("walk");
//	      walk.set(true);

	      FootstepDataListCommand footstepList = new FootstepDataListCommand();
         FootstepDataCommand footstepCommand = new FootstepDataCommand();

         Point3D position = new Point3D(0.0, 0.2, 0.0);
         Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footstepCommand.setPose(position, orientation);
         footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footstepCommand.setRobotSide(RobotSide.LEFT);
         footstepList.addFootstep(footstepCommand);

         position = new Point3D(0.3, -0.2, 0.0);
         orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footstepCommand.setPose(position, orientation);
         footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footstepCommand.setRobotSide(RobotSide.RIGHT);
         footstepList.addFootstep(footstepCommand);

         position = new Point3D(0.8, 0.2, 0.0);
         orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footstepCommand.setPose(position, orientation);
         footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footstepCommand.setRobotSide(RobotSide.LEFT);
         footstepList.addFootstep(footstepCommand);

         position = new Point3D(0.8, -0.2, 0.0);
         orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footstepCommand.setPose(position, orientation);
         footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footstepCommand.setRobotSide(RobotSide.RIGHT);
         footstepList.addFootstep(footstepCommand);

	      queuedControllerCommands.add(footstepList);



	      footstepList = new FootstepDataListCommand();
         footstepCommand = new FootstepDataCommand();

         position = new Point3D(1.0, 0.2, 0.0);
         orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footstepCommand.setPose(position, orientation);
         footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footstepCommand.setRobotSide(RobotSide.LEFT);
         footstepList.addFootstep(footstepCommand);

         position = new Point3D(1.3, -0.2, 0.0);
         orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footstepCommand.setPose(position, orientation);
         footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footstepCommand.setRobotSide(RobotSide.RIGHT);
         footstepList.addFootstep(footstepCommand);

         position = new Point3D(1.8, 0.2, 0.0);
         orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footstepCommand.setPose(position, orientation);
         footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footstepCommand.setRobotSide(RobotSide.LEFT);
         footstepList.addFootstep(footstepCommand);

         position = new Point3D(1.8, -0.2, 0.0);
         orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
         footstepCommand.setPose(position, orientation);
         footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
         footstepCommand.setRobotSide(RobotSide.RIGHT);
         footstepList.addFootstep(footstepCommand);


         queuedControllerCommands.add(footstepList);


	      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0);


	      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
	      drcSimulationTestHelper.checkNothingChanged();

	      assertTrue(success);

	      Point3D center = new Point3D(1.8, 0.0, 0.78);
	      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
	      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
	      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

	      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
	   }


     @ContinuousIntegrationTest(estimatedDuration = 31.9)
     @Test(timeout = 160000)
     public void testACoupleQueuedControllerCommands() throws SimulationExceededMaximumTimeException
     {
        BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

        String name = "DRCQueuedControllerCommandTest";

        FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
        DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

        drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, name, selectedLocation, simulationTestingParameters, getRobotModel());
        ConcurrentLinkedQueue<Command<?, ?>> queuedControllerCommands = drcSimulationTestHelper.getQueuedControllerCommands();

        setupCameraForWalkingUpToRamp();

        ThreadTools.sleep(1000);
        boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25);

//      BooleanYoVariable walk = (BooleanYoVariable) robot.getVariable("walk");
//      walk.set(true);

        FootstepDataListCommand footstepList = new FootstepDataListCommand();
        FootstepDataCommand footstepCommand = new FootstepDataCommand();

        Point3D position = new Point3D(0.3, 0.2, 0.0);
        Quaternion orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
        footstepCommand.setPose(position, orientation);
        footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
        footstepCommand.setRobotSide(RobotSide.LEFT);
        footstepList.addFootstep(footstepCommand);

        position = new Point3D(0.3, -0.2, 0.0);
        orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
        footstepCommand.setPose(position, orientation);
        footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
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
        chestCommand.setTrajectoryPointList(chestTrajectoryPointList);
        queuedControllerCommands.add(chestCommand);

        // Some more steps:
        footstepList = new FootstepDataListCommand();
        footstepCommand = new FootstepDataCommand();

        position = new Point3D(0.65, 0.2, 0.0);
        orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
        footstepCommand.setPose(position, orientation);
        footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
        footstepCommand.setRobotSide(RobotSide.LEFT);
        footstepList.addFootstep(footstepCommand);

        position = new Point3D(0.65, -0.2, 0.0);
        orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
        footstepCommand.setPose(position, orientation);
        footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
        footstepCommand.setRobotSide(RobotSide.RIGHT);
        footstepList.addFootstep(footstepCommand);

        position = new Point3D(1.1, 0.2, 0.0);
        orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
        footstepCommand.setPose(position, orientation);
        footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
        footstepCommand.setRobotSide(RobotSide.LEFT);
        footstepList.addFootstep(footstepCommand);

        position = new Point3D(1.1, -0.2, 0.0);
        orientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
        footstepCommand.setPose(position, orientation);
        footstepCommand.setOrigin(FootstepOrigin.AT_SOLE_FRAME);
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

        footTrajectoryCommand.setTrajectoryPointList(footPointList);
        footTrajectoryCommand.setRobotSide(RobotSide.RIGHT);
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


     @ContinuousIntegrationTest(estimatedDuration = 38.5)
     @Test(timeout = 190000)
     public void testACoupleMoreQueuedControllerCommands() throws SimulationExceededMaximumTimeException
     {
        BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

        String name = "DRCQueuedControllerCommandTest";

        FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
        DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

        drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, name, selectedLocation, simulationTestingParameters, getRobotModel());
        ConcurrentLinkedQueue<Command<?, ?>> queuedControllerCommands = drcSimulationTestHelper.getQueuedControllerCommands();

        setupCameraForWalkingUpToRamp();

        ThreadTools.sleep(1000);
        boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.25);

//      BooleanYoVariable walk = (BooleanYoVariable) robot.getVariable("walk");
//      walk.set(true);

        FootTrajectoryCommand footTrajectoryCommand = new FootTrajectoryCommand();

        FrameSE3TrajectoryPointList footPointList = new FrameSE3TrajectoryPointList();
        footPointList.addTrajectoryPoint(1.0, new Point3D(0.0, -0.2, 0.25), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
        footPointList.addTrajectoryPoint(2.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
        footPointList.addTrajectoryPoint(3.0, new Point3D(0.0, -0.2, 0.25), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
        footPointList.addTrajectoryPoint(4.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.0, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());

        footTrajectoryCommand.setTrajectoryPointList(footPointList);
        footTrajectoryCommand.setRobotSide(RobotSide.RIGHT);
        queuedControllerCommands.add(footTrajectoryCommand);

        success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);


        footTrajectoryCommand = new FootTrajectoryCommand();

        footPointList = new FrameSE3TrajectoryPointList();
        footPointList.addTrajectoryPoint(1.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.1, 0.0, 0.0, 1.0), new Vector3D(), new Vector3D());
        footPointList.addTrajectoryPoint(2.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.0, 0.1, 0.0, 1.0), new Vector3D(), new Vector3D());
        footPointList.addTrajectoryPoint(3.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.0, 0.0, 0.1, 1.0), new Vector3D(), new Vector3D());
        footPointList.addTrajectoryPoint(4.0, new Point3D(0.0, -0.2, 0.15), new Quaternion(0.0, 0.1, 0.0, 1.0), new Vector3D(), new Vector3D());

        footTrajectoryCommand.setTrajectoryPointList(footPointList);
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


	@ContinuousIntegrationTest(estimatedDuration = 52.3)
	@Test(timeout = 260000)
   public void testSimpleFlatGroundScriptWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCSimpleFlatGroundScriptTest", selectedLocation, simulationTestingParameters, getRobotModel());
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      setupCameraForWalkingUpToRamp();

      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = new OscillateFeetPerturber(robot, simulationConstructionSet.getDT() * (ticksPerPerturbation));
      oscillateFeetPerturber.setTranslationMagnitude(new double[]{0.008, 0.011, 0.004});
      oscillateFeetPerturber.setRotationMagnitudeYawPitchRoll(new double[]{0.012, 0.047, 0.009});

      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.LEFT, new double[]{1.0, 2.5, 3.3});
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.RIGHT, new double[]{2.0, 0.5, 1.3});

      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.LEFT, new double[]{5.0, 0.5, 0.3});
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.RIGHT, new double[]{0.2, 3.4, 1.11});

      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      // Load script file:
      String scriptName = "scripts/ExerciseAndJUnitScripts/SimpleFlatGroundScript.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(16.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.199355605426889, 0.15130115291430654, 0.8414863015120644);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.3, 0.5);
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

	@ContinuousIntegrationTest(estimatedDuration = 20.8)
	@Test(timeout = 100000)
   public void testStandingWithOscillatingFeet() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCSimpleFlatGroundScriptTest", selectedLocation, simulationTestingParameters, getRobotModel());
      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      setupCameraForWalkingUpToRamp();

      int ticksPerPerturbation = 10;
      OscillateFeetPerturber oscillateFeetPerturber = new OscillateFeetPerturber(robot, simulationConstructionSet.getDT() * (ticksPerPerturbation));
      oscillateFeetPerturber.setTranslationMagnitude(new double[]{0.01, 0.015, 0.005});
      oscillateFeetPerturber.setRotationMagnitudeYawPitchRoll(new double[]{0.017, 0.012, 0.011});

      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.LEFT, new double[]{5.0, 2.5, 3.3});
      oscillateFeetPerturber.setTranslationFrequencyHz(RobotSide.RIGHT, new double[]{2.0, 6.5, 1.3});

      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.LEFT, new double[]{5.0, 0.5, 7.3});
      oscillateFeetPerturber.setRotationFrequencyHzYawPitchRoll(RobotSide.RIGHT, new double[]{0.2, 3.4, 1.11});

      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(6.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.011508654344298094, -0.005208268357032689, 0.780662368979778);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 42.8)
	@Test(timeout = 210000)
   public void testLongStepsMaxHeightPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());


      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCLongStepsMaxHeightPauseAndRestartTest", selectedLocation, simulationTestingParameters, getRobotModel());
      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      String scriptName = "scripts/ExerciseAndJUnitScripts/LongStepsMaxHeightPauseAndRestart_LeftFootTest.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      success = success & drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(20.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(2.36472504931194, 0.012458249442189283, 0.7892313252995141);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 34.4)
	@Test(timeout = 170000)
   public void testSideStepsWithSlipping() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());


      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCLongStepsMaxHeightPauseAndRestartTest", selectedLocation, simulationTestingParameters, getRobotModel());

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();

      SlipOnNextStepPerturber slipOnEachStepPerturber = new SlipOnNextStepPerturber(robot, RobotSide.LEFT);
      slipOnEachStepPerturber.setAmountToSlipNextStep(getFootSlipVector());
      slipOnEachStepPerturber.setRotationToSlipNextStep(-0.15, 0.0, 0.0);
      slipOnEachStepPerturber.setSlipAfterStepTimeDelta(getFootSlipTimeDeltaAfterTouchdown());
      slipOnEachStepPerturber.setPercentToSlipPerTick(0.1);
      robot.setController(slipOnEachStepPerturber, 10);

      setupCameraForSideStepSlipping();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      String scriptName = "scripts/ExerciseAndJUnitScripts/LongSideStepsLeft.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      slipOnEachStepPerturber.setSlipNextStep(true);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(14.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.09590605437816137, 1.0379918543616593, 0.8383906558584916);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 33.1)
	@Test(timeout = 170000)
   public void testSideStepsWithRandomSlipping() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());


      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCSideStepsWithSlippingTest", selectedLocation, simulationTestingParameters, getRobotModel());

      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();

      SlipRandomOnNextStepPerturber slipRandomOnEachStepPerturber = new SlipRandomOnNextStepPerturber(robot, 1000L);
      slipRandomOnEachStepPerturber.setTranslationRangeToSlipNextStep(new double[]{0.0, 0.0, 0.0}, new double[]{0.04, 0.04, 0.01});
      slipRandomOnEachStepPerturber.setRotationRangeToSlipNextStep(new double[]{0.0, 0.0, 0.0}, new double[]{0.2, 0.05, 0.02});
      slipRandomOnEachStepPerturber.setSlipAfterStepTimeDeltaRange(0.01, 1.0);
      slipRandomOnEachStepPerturber.setSlipPercentSlipPerTickRange(0.02, 1.0);
      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.70);

      robot.setController(slipRandomOnEachStepPerturber, 10);

      setupCameraForSideStepSlipping();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      String scriptName = "scripts/ExerciseAndJUnitScripts/LongSideStepsLeft.xml";
      loadScriptFileInLeftSoleFrame(scriptName);

      slipRandomOnEachStepPerturber.setProbabilityOfSlip(0.5);
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(14.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(0.022704922237925088, 1.0831838988457891, 0.8389256934215261);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   // TODO re-enable that test when we have polygon to polygon contact model for SCS
	@ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testStandingOnUnevenTerrainForACoupleSeconds() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.TOP_OF_SLOPES;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCStandingTest", selectedLocation, simulationTestingParameters, getRobotModel());

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

   // @Test(timeout=300000)
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
	@ContinuousIntegrationTest(estimatedDuration = 50.0)
	@Test(timeout=300000)
   public void testRotatedStepInTheAir() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;
      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCRotatedStepsInTheAirTest", selectedLocation, simulationTestingParameters, getRobotModel());

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForRotatedStepInTheAir(scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(8.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 39.0)
	@Test(timeout = 190000)
   public void testWalkingUpToRampWithShortSteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingUpToRampShortStepsTest", selectedLocation, simulationTestingParameters, getRobotModel());

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingUpToRampShortSteps(scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);
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

	@ContinuousIntegrationTest(estimatedDuration = 36.5)
	@Test(timeout = 180000)
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCWalkingOccasionallyStraightKneesTest", selectedLocation, simulationTestingParameters, getRobotModel());

      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpToRamp();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForWalkingOnFlatLongSteps(scriptedFootstepGenerator);

      // FootstepDataList footstepDataList = createFootstepsForTwoLongFlatSteps(scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);
//      drcSimulationTestHelper.send(new ComHeightPacket(0.08, 1.0));


      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(12.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(3.1200570722246437, 0.017275273114368033, 0.8697236867426688);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 65.6)
	@Test(timeout = 330000)
   public void testTurningInPlaceAndPassingPI() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT_BUT_ALMOST_PI;

      drcSimulationTestHelper = new DRCSimulationTestHelper(flatGround, "DRCTurningInPlaceAndPassingPITest", selectedLocation, simulationTestingParameters, getRobotModel());

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForTurningInPlaceAndPassingPI();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForTurningInPlaceAndPassingPI(scriptedFootstepGenerator);
      drcSimulationTestHelper.send(footstepDataList);

      final DoubleYoVariable pelvisOrientationError = getPelvisOrientationErrorVariableName(simulationConstructionSet);

      SimulationDoneCriterion checkPelvisOrientationError = new SimulationDoneCriterion()
      {
         @Override
         public boolean isSimulationDone()
         {
            return (Math.abs(pelvisOrientationError.getDoubleValue()) > 0.1);
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

   private void setupCameraForSideStepSlipping()
   {
      Point3D cameraFix = new Point3D(2.0, 0.4, 0.75);
      Point3D cameraPosition = new Point3D(6.5, 0.4, 0.75);

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
      double[][][] footstepLocationsAndOrientations = new double[][][]
            {
            {
               {0.4, 0.10, 0.10},
               {0.4, 0.0, 0.0, 0.8}
            },
            {
               {0.48, -0.10329823409587219, 0.08400000000000005},
               {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
            }
            };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private Handstep createHandstepForTesting(ScriptedHandstepGenerator scriptedHandstepGenerator)
   {
      RobotSide robotSide = RobotSide.LEFT;
      Tuple3DBasics position = new Point3D(0.6, 0.3, 1.0);
      Vector3D surfaceNormal = new Vector3D(-1.0, 0.0, 0.0);
      double rotationAngleAboutNormal = 0.0;
      double swingTrajectoryTime = 1.0;
      return scriptedHandstepGenerator.createHandstep(robotSide, position, surfaceNormal, rotationAngleAboutNormal, swingTrajectoryTime);
   }

   private FootstepDataListMessage createFootstepsForWalkingUpToRampShortSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {0.2148448504580547, -0.09930268518393547, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {0.4481532647842352, 0.10329823409587219, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {0.6834821762408051, -0.09551979778612019, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {0.9167977582017036, 0.10565710343022289, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.1521266696582735, -0.09316092845176947, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.385442251619172, 0.1080159727645736, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.620771163075742, -0.09080205911741877, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {1.8540867450366407, 0.11037484209892431, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.0894156564932107, -0.08844318978306806, 0.08399999999999999},
            {3.405174677589428E-21, -6.767715309751755E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.322731238454109, 0.11273371143327501, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.558060149910679, -0.08608432044871735, 0.08398952447640476},
            {-5.047008501650524E-21, 4.53358964226292E-22, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {2.7913757318715775, 0.11509258076762573, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {3.0267046433281477, -0.08372545111436663, 0.08398952447640476},
            {-6.38257081820882E-21, -2.5377866560433405E-20, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {3.260020225289046, 0.11745145010197644, 0.08400000000000005},
            {-1.705361817083927E-23, 6.776242118837171E-21, 0.0025166698394258787, 0.9999968331814453}
         },
         {
            {3.2610268900368817, -0.08254601644719128, 0.08398952447640476},
            {3.49577202412201E-21, 2.923107094657073E-20, 0.0025166698394258787, 0.9999968331814453}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   private FootstepDataListMessage createFootstepsForWalkingOnFlatLongSteps(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {0.5909646234016005, 0.10243127081250579, 0.08400000000000002},
            {3.5805394102331502E-22, -1.0841962601668662E-19, 0.003302464707320093, 0.99999454684856}
         },
         {
            {1.212701966120992, -0.09394691394679651, 0.084}, {1.0806157207566333E-19, 1.0877767995770995E-19, 0.0033024647073200924, 0.99999454684856}
         },
         {
            {1.8317941784239657, 0.11014657591704705, 0.08619322927296164},
            {8.190550851520344E-19, 1.5693991726842814E-18, 0.003302464707320093, 0.99999454684856}
         },
         {
            {2.4535283480857237, -0.08575120920059497, 0.08069788195751608},
            {-2.202407644730947E-19, -8.117149793610565E-19, 0.0033024647073200924, 0.99999454684856}
         },
         {
            {3.073148474156348, 0.11833676240086898, 0.08590468550531082},
            {4.322378465953267E-5, 0.003142233766871708, 0.0033022799833692306, 0.9999896096688056}
         },
         {
            {3.0729346702590505, -0.0816428320664241, 0.0812390388356}, {-8.243740658642556E-5, -0.005993134849034999, 0.003301792738040525, 0.999976586577641}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.LEFT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }


   private FootstepDataListMessage createFootstepsForTurningInPlaceAndPassingPI(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][]
      {
         {
            {0.053884346896697966, 0.19273164589134978, 0.08574185103923426},
            {-6.938862977443471E-11, -8.7126898825953E-11, 0.9990480941331229, 0.04362230632342559}
         },
         {
            {0.05388201845443364, -0.20574623329424319, 0.08574185073944539},
            {1.6604742582112774E-10, 1.4170466407843545E-10, 0.9990483490180827, -0.04361646849807009}
         },
         {
            {0.0017235494647287533, 0.19045456181341558, 0.08574185040535603},
            {-5.0383363690493444E-11, -1.0843741493223105E-10, 0.9961949527487116, -0.0871528319562377}
         },
         {
            {0.10485496441611886, -0.19444611557725083, 0.08574185102571344},
            {1.5201027889830733E-10, 1.4860298371617872E-10, 0.9848082603764649, -0.1736453002366632}
         },
         {
            {-0.04807055917333275, 0.17475485972777594, 0.08574185070322422},
            {-3.05160242173266E-11, -1.2789253687750615E-10, 0.976296639469044, -0.21643676157587363}
         },
         {
            {0.15116636401480588, -0.17033827066486662, 0.08574185038049925},
            {1.3537219389951473E-10, 1.5295866511692108E-10, 0.9537178292633579, -0.3007030131960579}
         },
         {
            {-0.09210459251806524, 0.14670244796138915, 0.08574185100111767},
            {-1.0126547178247246E-11, -1.4515938198837407E-10, 0.9396936200915386, -0.3420173977435346}
         },
         {
            {0.18966017152321202, -0.13506560904726644, 0.0857418506668508},
            {1.1641785319333712E-10, 1.5469718133894557E-10, 0.9063090217942931, -0.42261561378428936}
         },
         {
            {-0.12737770450507258, 0.10820905279560836, 0.08574185036731347},
            {-1.0436197890210116E-11, 1.599425098341044E-10, -0.8870121823838428, 0.46174602142590504}
         },
         {
            {0.21771309767509972, -0.09103190305599193, 0.08574185095383173},
            {-9.547157074708167E-11, -1.5378878590499154E-10, -0.8433930157263759, 0.5372971440683164}
         },
         {
            {-0.15148609051286105, 0.061897935068802395, 0.085741850664082},
            {-3.082037679075772E-11, 1.7198897704022203E-10, -0.8191537200820054, 0.573574043063153}
         },
         {
            {0.23341338156809216, -0.0412379781596809, 0.08574185031046283},
            {-7.289174317616828E-11, -1.5024902169235376E-10, -0.7660463210652019, 0.6427853716307409}
         },
         {
            {-0.16278680351003783, 0.010925120900156002, 0.08574185095977704},
            {-5.067721042808702E-11, 1.8109266512133938E-10, -0.7372793107685568, 0.6755880534117237}
         },
         {
            {0.23569107567555475, 0.010922792383292988, 0.08574185059966628},
            {-4.906471775149843E-11, -1.441384550795834E-10, -0.6755923617465286, 0.7372753629070672}
         },
         {
            {-0.1605097194824509, -0.0412356760683499, 0.08574185032282551},
            {-6.966694301257708E-11, 1.87097807520974E-10, -0.6427898477118872, 0.766042565187163}
         },
         {
            {0.20979454839765582, 0.013396779318557463, 0.08574185088931394},
            {-2.91671807071375E-11, -1.3694134194254838E-10, -0.5937694707170026, 0.804635206565342}
         },
         {
            {-0.0496373406094997, -0.06666317759167362, 0.08574185062507425},
            {-7.826318574113734E-11, 1.8865011916447275E-10, -0.5937694705296589, 0.8046352067035897}
         }
      };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);

      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }

   protected abstract Vector3D getFootSlipVector();

   protected abstract double getFootSlipTimeDeltaAfterTouchdown();

   protected abstract DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs);
}
