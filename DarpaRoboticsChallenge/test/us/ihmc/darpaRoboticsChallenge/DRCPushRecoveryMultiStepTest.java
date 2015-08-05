package us.ihmc.darpaRoboticsChallenge;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingState;
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
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

public abstract class DRCPushRecoveryMultiStepTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   
   private DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack;

   private final static boolean VISUALIZE_FORCE = false;
   private final static double PUSH_DELAY = 0.5;

   protected DRCPushRobotController pushRobotController;
   protected BlockingSimulationRunner blockingSimulationRunner;
   protected double forceMagnitude;
   protected double forceDuration;

   protected SideDependentList<StateTransitionCondition> doubleSupportStartConditions = new SideDependentList<>();
   StateTransitionCondition pushCondition = doubleSupportStartConditions.get(RobotSide.LEFT);


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
      if (drcFlatGroundWalkingTrack != null)
      {
         drcFlatGroundWalkingTrack.destroySimulation();
         drcFlatGroundWalkingTrack = null;
      }

      if (blockingSimulationRunner != null)
      {
         blockingSimulationRunner.destroySimulation();
         blockingSimulationRunner = null;
      }
     
         pushRobotController = null;

         doubleSupportStartConditions = null;

         pushCondition = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@EstimatedDuration(duration = 60.0)
	@Test(timeout = 300190)
   public void testMultiStepForwardAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      setForwardPushParameters();

      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(4.0);

      // push the robot
      pushRobotController.applyForceDelayed(pushCondition, PUSH_DELAY, forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 6.0);

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      createMovie(scs);

      BambooTools.reportTestFinishedMessage();
   }

	@EstimatedDuration(duration = 52.7)
	@Test(timeout = 263313)
   public void testMultiStepBackwardAndContinueWalking() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();

      setupTest(getRobotModel());
      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();

      setBackwardPushParameters();

      Vector3d forceDirection = new Vector3d(1.0, 0.0, 0.0);

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");

      // disable walking
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(4.0);

      // push the robot
      pushRobotController.applyForceDelayed(pushCondition, PUSH_DELAY, forceDirection, forceMagnitude, forceDuration);

      // simulate for a little while longer
      blockingSimulationRunner.simulateAndBlock(forceDuration + 5.0);

      //re-enable walking
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(6.0);

      createMovie(scs);

      BambooTools.reportTestFinishedMessage();
   }

   protected void setupTest(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException, InterruptedException
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

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = fullRobotModel.getFoot(robotSide).getName();
         scs.getVariable(prefix + "FootControlModule", prefix
               + "State");
         final EnumYoVariable<WalkingState> walkingState = (EnumYoVariable<WalkingState>) scs.getVariable("WalkingHighLevelHumanoidController", "walkingState");

         doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }
   }

   private void setupTrack(boolean runMultiThreaded, DRCRobotModel robotModel)
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, simulationTestingParameters);

      GroundProfile3D groundProfile = new FlatGroundProfile();

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      //      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setRunMultiThreaded(runMultiThreaded);
      
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, true, false,
            robotModel);

      drcFlatGroundWalkingTrack.getDrcSimulation();
   }

   private class DoubleSupportStartCondition implements StateTransitionCondition
   {
      private final EnumYoVariable<WalkingState> walkingState;
      private final RobotSide side;

      public DoubleSupportStartCondition(EnumYoVariable<WalkingState> walkingState, RobotSide side)
      {
         this.walkingState = walkingState;
         this.side = side;
      }

      @Override
      public boolean checkCondition()
      {
         if (side == RobotSide.LEFT)
         {
            return walkingState.getEnumValue() == WalkingState.DOUBLE_SUPPORT || walkingState.getEnumValue() == WalkingState.TRANSFER_TO_LEFT_SUPPORT;
         }
         else
         {
            return walkingState.getEnumValue() == WalkingState.DOUBLE_SUPPORT || walkingState.getEnumValue() == WalkingState.TRANSFER_TO_RIGHT_SUPPORT;
         }
      }
   }

   private void createMovie(SimulationConstructionSet scs)
   {
      if (simulationTestingParameters.getCreateSCSMovies())
      {
         BambooTools.createMovieAndDataWithDateTimeClassMethodAndShareOnSharedDriveIfAvailable(getSimpleRobotName(), scs, 1);
      }
   }

   protected abstract void setForwardPushParameters();

   protected abstract void setBackwardPushParameters();
}
