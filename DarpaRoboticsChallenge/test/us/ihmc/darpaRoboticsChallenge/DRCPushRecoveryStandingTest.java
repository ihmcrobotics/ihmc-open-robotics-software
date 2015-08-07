package us.ihmc.darpaRoboticsChallenge;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCPushRobotController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;

public abstract class DRCPushRecoveryStandingTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();   
   private BlockingSimulationRunner blockingSimulationRunner;
   private DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack;

   private final static boolean VISUALIZE_FORCE = true;

   private DRCPushRobotController pushRobotController;
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
   
  
	@EstimatedDuration(duration = 30.6)
	@Test(timeout = 153027)
   public void TestPushForwardInDoubleSupport() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      double forceMagnitude = 400.0;
      double forceDuration = 0.15;
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(3.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 5.0);

      BambooTools.reportTestFinishedMessage();
   }

	@EstimatedDuration(duration = 40.2)
	@Test(timeout = 201110)
   public void TestPushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      double forceMagnitude = 400.0;
      double forceDuration = 0.15;
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(3.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 4.0);

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      BambooTools.reportTestFinishedMessage();
   }

	@EstimatedDuration(duration = 45.6)
	@Test(timeout = 227903)
   public void TestDoublePushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      //TODO: Increase the force magnitude and do more agressive pushes.
      double forceMagnitude = 400.0; //350.0;
      double forceDuration = 0.15;
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

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

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      BambooTools.reportTestFinishedMessage();
   }

	@EstimatedDuration(duration = 39.9)
	@Test(timeout = 199744)
   public void TestPushBackwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      double forceMagnitude = -400.0;
      double forceDuration = 0.15;
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(3.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 3.0);

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      BambooTools.reportTestFinishedMessage();
   }

	@EstimatedDuration(duration = 52.4)
	@Test(timeout = 262099)
   public void TestPushBackwardForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      //TODO: Increase the force magnitude and do more agressive pushes.
      double forceMagnitude = -400.0; //-400.0;
      double forceDuration = 0.15;
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

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

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      BambooTools.reportTestFinishedMessage();
   }

   private void setupTest(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException, InterruptedException
   {
      boolean runMultiThreaded = false;
      setupTrack(runMultiThreaded, robotModel);
      FullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      pushRobotController = new DRCPushRobotController(drcFlatGroundWalkingTrack.getDrcSimulation().getRobot(), fullRobotModel);

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
      
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, true, false,
            robotModel);
   }
}
