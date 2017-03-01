package us.ihmc.avatar;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.visualizer.RobotVisualizer;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCPushRecoveryStandingTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();   
   private BlockingSimulationRunner blockingSimulationRunner;
   private DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack;

   private final static boolean VISUALIZE_FORCE = true;

   private PushRobotController pushRobotController;
   private RobotVisualizer robotVisualizer;

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
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }

      if (drcFlatGroundWalkingTrack != null)
      {
         drcFlatGroundWalkingTrack.destroySimulation();
         drcFlatGroundWalkingTrack = null;
      }

      if (robotVisualizer != null)
      {
         robotVisualizer.close();
         robotVisualizer = null;
      }

      pushRobotController = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
  
	@ContinuousIntegrationTest(estimatedDuration = 25.3)
	@Test(timeout = 130000)
   public void testPushForwardInDoubleSupport() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      double forceMagnitude = 400.0;
      double forceDuration = 0.15;
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("ComponentBasedFootstepDataMessageGenerator", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(3.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 5.0);

      assertRobotDidNotFall();

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 33.4)
	@Test(timeout = 170000)
   public void testPushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      double forceMagnitude = 400.0;
      double forceDuration = 0.15;
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("ComponentBasedFootstepDataMessageGenerator", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(3.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 4.0);

      assertRobotDidNotFall();

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      assertRobotDidNotFall();

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@Ignore("Needs to be improved")
	@ContinuousIntegrationTest(estimatedDuration = 49.1)
	@Test(timeout = 250000)
   public void testDoublePushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      //TODO: Increase the force magnitude and do more agressive pushes.
      double forceMagnitude = 400.0; //350.0;
      double forceDuration = 0.15;
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("ComponentBasedFootstepDataMessageGenerator", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(1.5);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 2.5);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 2.0);

      assertRobotDidNotFall();

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      assertRobotDidNotFall();

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 30.8)
	@Test(timeout = 150000)
   public void testPushBackwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      double forceMagnitude = -400.0;
      double forceDuration = 0.15;
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("ComponentBasedFootstepDataMessageGenerator", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(3.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 3.0);

      assertRobotDidNotFall();

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      assertRobotDidNotFall();

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@ContinuousIntegrationTest(estimatedDuration = 48.6)
	@Test(timeout = 240000)
   public void testPushBackwardForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      //TODO: Increase the force magnitude and do more agressive pushes.
      double forceMagnitude = -400.0; //-400.0;
      double forceDuration = 0.15;
      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("ComponentBasedFootstepDataMessageGenerator", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(1.5);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 4.0);

      forceMagnitude = 400.0;

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 3.0);

      assertRobotDidNotFall();

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      assertRobotDidNotFall();

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void setupTest(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException, InterruptedException
   {
      boolean runMultiThreaded = false;
      setupTrack(runMultiThreaded, robotModel);
      FullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      pushRobotController = new PushRobotController(drcFlatGroundWalkingTrack.getAvatarSimulation().getHumanoidFloatingRootJointRobot(), fullRobotModel);

      if (VISUALIZE_FORCE)
      {
         drcFlatGroundWalkingTrack.getSimulationConstructionSet().addYoGraphic(pushRobotController.getForceVisualizer());
      }

      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");

      // enable push recovery
      enable.set(true);
   }

   private void setupTrack(boolean runMultiThreaded, DRCRobotModel robotModel)
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, simulationTestingParameters);

      GroundProfile3D groundProfile = new FlatGroundProfile();

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setRunMultiThreaded(runMultiThreaded);
      
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, true, false,
            robotModel);
   }

   public void assertRobotDidNotFall()
   {
      Point3D center = new Point3D(0.0, 0.0, 0.8882009563211146);
      Vector3D plusMinusVector = new Vector3D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      DRCSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox, drcFlatGroundWalkingTrack.getAvatarSimulation().getHumanoidFloatingRootJointRobot());
   }
}
