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
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.visualizer.RobotVisualizer;
import us.ihmc.yoUtilities.time.GlobalTimer;

public abstract class DRCPushRecoveryStandingTest implements MultiRobotTestInterface
{
   private final static boolean KEEP_SCS_UP = false;
   private static final boolean createMovie = BambooTools.doMovieCreation();
   private static final boolean SHOW_GUI = KEEP_SCS_UP || createMovie;
   private final static boolean VISUALIZE_FORCE = false;

   private DRCPushRobotController pushRobotController;
   private BlockingSimulationRunner blockingSimulationRunner;
   private DRCSimulationFactory drcSimulation;
   private RobotVisualizer robotVisualizer;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (KEEP_SCS_UP)
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be
      // recycled.
      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }

      if (drcSimulation != null)
      {
         drcSimulation.dispose();
         drcSimulation = null;
      }

      if (robotVisualizer != null)
      {
         robotVisualizer.close();
         robotVisualizer = null;
      }

      GlobalTimer.clearTimers();
      TimerTaskScheduler.cancelAndReset();
      AsyncContinuousExecutor.cancelAndReset();

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test(timeout=300000)
   public void TestPushForwardInDoubleSupport() throws SimulationExceededMaximumTimeException, InterruptedException
   {
      BambooTools.reportTestStartedMessage();

      DRCFlatGroundWalkingTrack track = setupTest(getRobotModel());
      SimulationConstructionSet scs = track.getSimulationConstructionSet();

      double forceMagnitude = 350.0;
      double forceDuration = 0.15;
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(6.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 5.0);

      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout=300000)
   public void TestPushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException
   {
      BambooTools.reportTestStartedMessage();

      DRCFlatGroundWalkingTrack track = setupTest(getRobotModel());
      SimulationConstructionSet scs = track.getSimulationConstructionSet();

      double forceMagnitude = 350.0;
      double forceDuration = 0.15;
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(6.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 2.0);

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout=300000)
   public void TestDoublePushForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException
   {
      BambooTools.reportTestStartedMessage();

      DRCFlatGroundWalkingTrack track = setupTest(getRobotModel());
      SimulationConstructionSet scs = track.getSimulationConstructionSet();

      double forceMagnitude = 350.0;
      double forceDuration = 0.15;
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(6.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 2.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 2.0);

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout=300000)
   public void TestPushBackwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException
   {
      BambooTools.reportTestStartedMessage();

      DRCFlatGroundWalkingTrack track = setupTest(getRobotModel());
      SimulationConstructionSet scs = track.getSimulationConstructionSet();

      double forceMagnitude = -450.0;
      double forceDuration = 0.15;
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(6.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 2.0);

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      BambooTools.reportTestFinishedMessage();
   }

   @Test(timeout=300000)
   public void TestPushBackwardForwardInDoubleSupportAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException
   {
      BambooTools.reportTestStartedMessage();

      DRCFlatGroundWalkingTrack track = setupTest(getRobotModel());
      SimulationConstructionSet scs = track.getSimulationConstructionSet();

      double forceMagnitude = -400.0;
      double forceDuration = 0.15;
      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(6.0);

      // push the robot
      pushRobotController.applyForce(forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 3.0);

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

   private DRCFlatGroundWalkingTrack setupTest(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException, InterruptedException
   {
      DRCSimulationFactory.RUN_MULTI_THREADED = false;
      DRCFlatGroundWalkingTrack track = setupTrack(robotModel);
      FullRobotModel fullRobotModel = robotModel.createFullRobotModel();
      pushRobotController = new DRCPushRobotController(track.getDrcSimulation().getRobot(), fullRobotModel);

      if (VISUALIZE_FORCE)
      {
         track.getSimulationConstructionSet().addYoGraphic(pushRobotController.getForceVisualizer());
      }

      SimulationConstructionSet scs = track.getSimulationConstructionSet();

      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");
      BooleanYoVariable enableDS = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecoveryFromDoubleSupport");
      BooleanYoVariable usePushRecoveryICPPlanner = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "usePushRecoveryICPPlanner");

      // enable push recovery
      enable.set(true);
      enableDS.set(true);

      // enable ICP push recovery planner and disable projection planner
      usePushRecoveryICPPlanner.set(true);

      return track;
   }

   private DRCFlatGroundWalkingTrack setupTrack(DRCRobotModel robotModel)
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);
      guiInitialSetup.setIsGuiShown(SHOW_GUI);

      GroundProfile3D groundProfile = new FlatGroundProfile();

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setDrawGroundProfile(true);

      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, true, false,
            robotModel);

      drcSimulation = drcFlatGroundWalkingTrack.getDrcSimulation();
      return drcFlatGroundWalkingTrack;
   }
}
