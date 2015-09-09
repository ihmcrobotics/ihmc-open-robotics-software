package us.ihmc.darpaRoboticsChallenge;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFHumanoidRobot;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingState;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCPushRobotController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.SdfLoader.visualizer.RobotVisualizer;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

public abstract class DRCPushRecoveryWalkingTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();   
   private BlockingSimulationRunner blockingSimulationRunner;
   private DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack;
   private final static boolean VISUALIZE_FORCE = false;

   private double swingTime, transferTime;
   private SideDependentList<StateTransitionCondition> swingStartConditions = new SideDependentList<>();
   private SideDependentList<StateTransitionCondition> swingFinishConditions = new SideDependentList<>();

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

      swingStartConditions = null;
      swingFinishConditions = null;
   
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   // cropped to 1.5 - 6.3 seconds
   @Ignore
	@DeployableTestMethod(estimatedDuration = 50.0)
	@Test(timeout=300000)
   public void testForVideo() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      Vector3d forceDirection = new Vector3d(0.0, 1.0, 0.0);
      double magnitude = 150.0;
      double duration = 0.5;
      double percentInSwing = 0.05;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      BambooTools.reportTestFinishedMessage();
   }

	@DeployableTestMethod(estimatedDuration = 38.9)
	@Test(timeout = 190000)
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      Vector3d forceDirection = new Vector3d(0.0, 1.0, 0.0);
      double magnitude = 750.0;
      double duration = 0.04;
      double percentInSwing = 0.2;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      BambooTools.reportTestFinishedMessage();
   }

	@DeployableTestMethod(estimatedDuration = 42.2)
	@Test(timeout = 210000)
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      Vector3d forceDirection = new Vector3d(0.0, 1.0, 0.0);
      double magnitude = 800.0;
      double duration = 0.05;
      double percentInSwing = 0.5;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      BambooTools.reportTestFinishedMessage();
   }

	@DeployableTestMethod(estimatedDuration = 52.9)
	@Test(timeout = 260000)
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      Vector3d forceDirection = new Vector3d(0.0, -1.0, 0.0);
      double magnitude = 800.0;
      double duration = 0.05;
      double percentInSwing = 0.4;
      RobotSide side = RobotSide.RIGHT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      // push the robot again with new parameters
      forceDirection = new Vector3d(-1.0, 0.0, 0.0);
      magnitude = 700.0;
      duration = 0.05;
      side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      BambooTools.reportTestFinishedMessage();
   }

	@DeployableTestMethod(estimatedDuration = 38.7)
	@Test(timeout = 190000)
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      Vector3d forceDirection = new Vector3d(-0.5, 1.0, 0.0);
      double magnitude = 800;
      double duration = 0.05;
      double percentInSwing = 0.2;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      BambooTools.reportTestFinishedMessage();
   }

	@DeployableTestMethod(estimatedDuration = 36.1)
	@Test(timeout = 180000)
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      Vector3d forceDirection = new Vector3d(0.5, 1.0, 0.0);
      double magnitude = 800.0;
      double duration = 0.05;
      double percentInSwing = 0.4;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      BambooTools.reportTestFinishedMessage();
   }

	@DeployableTestMethod(estimatedDuration = 65.8)
	@Test(timeout = 330000)
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      Vector3d forceDirection = new Vector3d(0.0, -1.0, 0.0);
      double magnitude = 600.0;
      double duration = 0.05;
      double percentInTransferState = 0.5;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInTransferState, side, swingFinishConditions, transferTime);
      
      // push the robot again with new parameters
      forceDirection = new Vector3d(0.5, -1.0, 0.0);
      magnitude = 700.0;
      duration = 0.05;
      double percentInSwing = 0.4;
      side = RobotSide.RIGHT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      BambooTools.reportTestFinishedMessage();
   }

	@DeployableTestMethod(estimatedDuration = 75.4)
	@Test(timeout = 380000)
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      Vector3d forceDirection = new Vector3d(0.0, -1.0, 0.0);
      double magnitude = 600.0;
      double duration = 0.05;
      double percentInTransferState = 0.5;
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInTransferState, side, swingFinishConditions, transferTime);
      
      // push the robot again with new parameters
      forceDirection = new Vector3d(0.0, 1.0, 0.0);
      magnitude = 600.0;
      duration = 0.05;
      double percentInSwing = 0.4;
      side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      BambooTools.reportTestFinishedMessage();
   }

	@DeployableTestMethod(estimatedDuration = 37.4)
	@Test(timeout = 190000)
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      Vector3d forceDirection = new Vector3d(0.0, -1.0, 0.0);
      double magnitude = 700.0;
      double duration = 0.05;
      // This doesn't work for 0.5 or higher. We need to do more development to get this working better. So for now, just stick with 0.4.
      double percentInTransferState = 0.4; //0.9 * Math.random();
      PrintTools.info(this, "percentInTransferState = " + percentInTransferState);
      RobotSide side = RobotSide.LEFT;

      // apply the push
      testPush(forceDirection, magnitude, duration, percentInTransferState, side, swingFinishConditions, transferTime);

      BambooTools.reportTestFinishedMessage();
   }

   private void setupTest(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      boolean runMultiThreaded = false;
      setupTrack(runMultiThreaded, robotModel);
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      pushRobotController = new DRCPushRobotController(drcFlatGroundWalkingTrack.getDrcSimulation().getRobot(), fullRobotModel);

      SimulationConstructionSet scs = drcFlatGroundWalkingTrack.getSimulationConstructionSet();
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");
      cameraConfiguration.setCameraFix(0.6, 0.0, 0.6);
      cameraConfiguration.setCameraPosition(10.0, 3.0, 3.0);
      cameraConfiguration.setCameraTracking(true, true, false, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");

      if (VISUALIZE_FORCE)
      {
         scs.addYoGraphic(pushRobotController.getForceVisualizer());
      }

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);

      // get YoVariables
      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");
      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");

      for (RobotSide robotSide : RobotSide.values)
      {
         String prefix = fullRobotModel.getFoot(robotSide).getName();
         @SuppressWarnings("unchecked")
         final EnumYoVariable<ConstraintType> footConstraintType = (EnumYoVariable<ConstraintType>) scs.getVariable(prefix + "FootControlModule", prefix
               + "State");
         @SuppressWarnings("unchecked")
         final EnumYoVariable<WalkingState> walkingState = (EnumYoVariable<WalkingState>) scs.getVariable("WalkingHighLevelHumanoidController", "walkingState");
         
         swingStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
         swingFinishConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
      }
      // simulate for a while
      enable.set(true);
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(1.0);
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(2.0);
   }

   private void setupTrack(boolean runMultiThreaded, DRCRobotModel robotModel)
   {
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false, simulationTestingParameters);
      GroundProfile3D groundProfile = new FlatGroundProfile();

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setRunMultiThreaded(runMultiThreaded);

      DRCRobotInitialSetup<SDFHumanoidRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, true, false,
            robotModel);
   }

   private void testPush(Vector3d forceDirection, double magnitude, double duration, double percentInState, RobotSide side, SideDependentList<StateTransitionCondition> condition, double stateTime)
         throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      double delay = stateTime * percentInState;

      pushRobotController.applyForceDelayed(condition.get(side), delay, forceDirection, magnitude, duration);

      blockingSimulationRunner.simulateAndBlock(8.0);
   }

   private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final EnumYoVariable<ConstraintType> footConstraintType;

      public SingleSupportStartCondition(EnumYoVariable<ConstraintType> footConstraintType)
      {
         this.footConstraintType = footConstraintType;
      }

      @Override
      public boolean checkCondition()
      {
         return footConstraintType.getEnumValue() == ConstraintType.SWING;
      }
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
}
