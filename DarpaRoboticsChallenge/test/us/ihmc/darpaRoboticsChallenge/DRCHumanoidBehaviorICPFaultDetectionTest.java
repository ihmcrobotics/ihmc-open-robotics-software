package us.ihmc.darpaRoboticsChallenge;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingState;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCPushRobotController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.environment.DRCDemo01NavigationEnvironment;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.SdfLoader.visualizer.RobotVisualizer;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

public abstract class DRCHumanoidBehaviorICPFaultDetectionTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();   
   private BlockingSimulationRunner blockingSimulationRunner;

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

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   private final static boolean VISUALIZE_FORCE = true;

   private double swingTime, transferTime;
   private final SideDependentList<StateTransitionCondition> swingStartConditions = new SideDependentList<>();
   private final SideDependentList<StateTransitionCondition> swingFinishConditions = new SideDependentList<>();

   private DRCPushRobotController pushRobotController;
   private DRCSimulationFactory drcSimulation;
   private RobotVisualizer robotVisualizer;
   private SimulationConstructionSet scs;
   
   private  BooleanYoVariable enablePushing;


   @After
   public void tearDown()
   {
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
   }

   // cropped to 1.5 - 6.3 seconds
   @Ignore

	@EstimatedDuration
	@Test(timeout=300000)
   public void TestForVideo() throws SimulationExceededMaximumTimeException, InterruptedException
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

   @Ignore

	@EstimatedDuration
	@Test(timeout=300000)
   public void TestPushLeftEarlySwing() throws SimulationExceededMaximumTimeException, InterruptedException
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

   @Ignore

	@EstimatedDuration
	@Test(timeout=300000)
   public void TestPushRightLateSwing() throws SimulationExceededMaximumTimeException, InterruptedException
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

   @Ignore

	@EstimatedDuration
	@Test(timeout=300000)
   public void TestPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException, InterruptedException
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

   @Ignore

	@EstimatedDuration
	@Test(timeout=300000)
   public void TestPushTowardsTheBack() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {

      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      double duration = 0.05;
      double percentInSwing = 0.2;
      RobotSide side = RobotSide.LEFT;
      double magnitude = 800;
      Vector3d forceDirection = new Vector3d(-1.0, 0.0, 0.0);

      // apply the push
      ThreadTools.sleep(25000);

      for (int i = 0; i < 100; i++)
      {
         ThreadTools.sleep(5000);
         if(enablePushing.getBooleanValue() == true){
            
         if (i % 2 == 0)
         {
            magnitude = 400;
            forceDirection = new Vector3d(-1.0, 0.0, 0.0);
            System.out.println("DRCHumanoidBehaviorICPFaultDetectionTest: Pushing with " + magnitude + " N of force from the front");
         }
         else
         {
            magnitude = 800;
            forceDirection = new Vector3d(0.0, 1.0, 0.0);
            System.out.println("DRCHumanoidBehaviorICPFaultDetectionTest: Pushing with " + magnitude + " N of force from the right");
         }
         testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);
         }
      }

      blockingSimulationRunner.simulateAndBlock(100.0);
      BambooTools.reportTestFinishedMessage();

   }

   @Ignore

	@EstimatedDuration
	@Test(timeout=300000)
   public void TestPushTowardsTheFront() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      BambooTools.reportTestStartedMessage();
      setupTest(getRobotModel());

      // setup all parameters
      Vector3d forceDirection = new Vector3d(0.5, 1.0, 0.0);
      double magnitude = 8000.0;
      double duration = 0.05;
      //      double percentInSwing = 0.4;
      //      RobotSide side = RobotSide.LEFT;

      // apply the push
      //      testPush(forceDirection, magnitude, duration, percentInSwing, side, swingStartConditions, swingTime);

      blockingSimulationRunner.simulateAndBlock(8.0);
      pushRobotController.applyForce(forceDirection, magnitude, duration);
      BambooTools.reportTestFinishedMessage();
   }

   private void setupTest(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException, InterruptedException
   {
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, new DRCDemo01NavigationEnvironment());
      simulationStarter.setRunMultiThreaded(false);

      boolean automaticallyStartSimulation = true;
      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.enableUiModule(automaticallyStartSimulation);
      networkProcessorParameters.enableLocalControllerCommunicator(true);
      simulationStarter.startSimulation(networkProcessorParameters, automaticallyStartSimulation);
      
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      pushRobotController = new DRCPushRobotController(simulationStarter.getSDFRobot(), fullRobotModel);

      SimulationConstructionSet scs = simulationStarter.getSimulationConstructionSet();
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");
      cameraConfiguration.setCameraFix(0.6, 0.0, 0.6);
      cameraConfiguration.setCameraPosition(10.0, 3.0, 3.0);
      cameraConfiguration.setCameraTracking(true, true, false, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
      enablePushing = new BooleanYoVariable("enablePushing", scs.getRootRegistry());
      enablePushing.set(false);

      if (VISUALIZE_FORCE)
      {
         scs.addYoGraphic(pushRobotController.getForceVisualizer());
      }

      blockingSimulationRunner = new BlockingSimulationRunner(scs, 60.0);

      // get YoVariables
      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");
      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");

      for (RobotSide robotSide : RobotSide.values)
      {
         //         System.out.println("DRCHumanoidBehaviorICPFaultDetectionTest: made it to the for loop " + robotSide.toString());
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
      //      enable.set(true);
      //      enableDS.set(true);
      //      walk.set(false);
      //      blockingSimulationRunner.simulateAndBlock(1.0);
      //      System.out.println("DRCHumanoidBehaviorICPFaultDetectionTest: Made it after the first simulate and block");
      //      walk.set(true);
      //      blockingSimulationRunner.simulateAndBlock(2.0);
   }

   //   private DRCFlatGroundWalkingTrack setupTrack(DRCRobotModel robotModel)
   //   {
   //      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);
   //      guiInitialSetup.setIsGuiShown(SHOW_GUI);
   //      GroundProfile3D groundProfile = new FlatGroundProfile();
   //
   //      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
   //      scsInitialSetup.setInitializeEstimatorToActual(true);
   //      scsInitialSetup.setDrawGroundProfile(true);
   //
   //      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);
   //
   //      DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, true, false,
   //            robotModel);
   //
   //      drcSimulation = drcFlatGroundWalkingTrack.getDrcSimulation();
   //      return drcFlatGroundWalkingTrack;
   //   }

   private void testPush(Vector3d forceDirection, double magnitude, double duration, double percentInState, RobotSide side,
         SideDependentList<StateTransitionCondition> condition, double stateTime) throws SimulationExceededMaximumTimeException, InterruptedException
   {
      System.out.println("DRCHumanoidBehaviorICPFaultDetectionFault: testPush called.");

      double delay = stateTime * percentInState;

      pushRobotController.applyForceDelayed(null, 800, forceDirection, magnitude, duration);

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
