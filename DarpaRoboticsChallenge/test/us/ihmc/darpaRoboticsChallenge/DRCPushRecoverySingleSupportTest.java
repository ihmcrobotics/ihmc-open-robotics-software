package us.ihmc.darpaRoboticsChallenge;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.bambooTools.BambooTools;
import us.ihmc.commonWalkingControlModules.automaticSimulationRunner.AutomaticSimulationRunner;
import us.ihmc.commonWalkingControlModules.visualizer.RobotVisualizer;
import us.ihmc.darpaRoboticsChallenge.controllers.DRCPushRobotController;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile;
import us.ihmc.graphics3DAdapter.camera.CameraConfiguration;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.AsyncContinuousExecutor;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.TimerTaskScheduler;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.time.GlobalTimer;
import com.yobotics.simulationconstructionset.util.FlatGroundProfile;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import com.yobotics.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public abstract class DRCPushRecoverySingleSupportTest implements MultiRobotTestInterface 
{
	private final static boolean SHOW_GUI = false;
	private final static boolean VISUALIZE_FORCE = false;
	
	private double swingTime;
	private SingleSupportStartCondition swingStartConditionRight;
	private SingleSupportStartCondition swingStartConditionLeft;

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
		if (SHOW_GUI)
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
	
	// cropped to 1.5 - 6.3 seconds
	@Ignore
	@Test
   public void TestForVideo() throws SimulationExceededMaximumTimeException,InterruptedException 
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
      testPush(forceDirection, magnitude, duration, percentInSwing, side);
      
      BambooTools.reportTestFinishedMessage();
   }

	@Test
	public void TestPushLeftEarlySwing() throws SimulationExceededMaximumTimeException,InterruptedException 
	{
		BambooTools.reportTestStartedMessage();
		setupTest(getRobotModel());
		
		// setup all parameters
      Vector3d forceDirection = new Vector3d(0.0, 1.0, 0.0);
      double magnitude = 800.0;
      double duration = 0.04;
		double percentInSwing = 0.1;
		RobotSide side = RobotSide.LEFT;
		
		// apply the push
		testPush(forceDirection, magnitude, duration, percentInSwing, side);
		
		BambooTools.reportTestFinishedMessage();
	}
	
	@Test
   public void TestPushRightLateSwing() throws SimulationExceededMaximumTimeException,InterruptedException 
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
      testPush(forceDirection, magnitude, duration, percentInSwing, side);
      
      BambooTools.reportTestFinishedMessage();
   }
	
	@Test
	public void TestPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException,InterruptedException 
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
		testPush(forceDirection, magnitude, duration, percentInSwing, side);
		
     	// push the robot again with new parameters
	   forceDirection = new Vector3d(0.0, 1.0, 0.0);
		magnitude = 800.0;
		duration = 0.05;
		side = RobotSide.LEFT;
		
		// apply the push
		testPush(forceDirection, magnitude, duration, percentInSwing, side);
		
		BambooTools.reportTestFinishedMessage();
	}
	
	@Test
	public void TestPushTowardsTheBack() throws SimulationExceededMaximumTimeException,InterruptedException 
	{
		BambooTools.reportTestStartedMessage();
		setupTest(getRobotModel());

		// setup all parameters
		Vector3d forceDirection = new Vector3d(-0.5, 1.0, 0.0);
		double magnitude = 900;
		double duration = 0.05;
		double percentInSwing = 0.2;
      RobotSide side = RobotSide.LEFT;
		
	   // apply the push
		testPush(forceDirection, magnitude, duration, percentInSwing, side);
		
		BambooTools.reportTestFinishedMessage();
	}
	
	@Test
	public void TestPushTowardsTheFront() throws SimulationExceededMaximumTimeException,InterruptedException 
	{
		BambooTools.reportTestStartedMessage();
		setupTest(getRobotModel());

	   // setup all parameters
      Vector3d forceDirection = new Vector3d(1.0, 1.0, 0.0);
      double magnitude = 800.0;
      double duration = 0.05;
      double percentInSwing = 0.4;
      RobotSide side = RobotSide.LEFT;
      
      // apply the push
      testPush(forceDirection, magnitude, duration, percentInSwing, side);
		
		BambooTools.reportTestFinishedMessage();
	}

	private void setupTest(DRCRobotModel robotModel) throws SimulationExceededMaximumTimeException, InterruptedException {
		DRCFlatGroundWalkingTrack track = setupTrack(robotModel);
		FullRobotModel fullRobotModel = robotModel.createFullRobotModel();
		swingTime = robotModel.getWalkingControlParameters().getDefaultSwingTime();
		pushRobotController = new DRCPushRobotController(track.getDrcSimulation().getRobot(), fullRobotModel);
		
      SimulationConstructionSet scs = track.getSimulationConstructionSet();
      CameraConfiguration cameraConfiguration = new CameraConfiguration("testCamera");
      cameraConfiguration.setCameraFix(0.6, 0.0, 0.6);
      cameraConfiguration.setCameraPosition(10.0, 3.0, 3.0);
      cameraConfiguration.setCameraTracking(true, true, false, false);
      scs.setupCamera(cameraConfiguration);
      scs.selectCamera("testCamera");
      
      if(VISUALIZE_FORCE)
      {
         scs.addDynamicGraphicObject(pushRobotController.getForceVisualizer());
      }
      
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 1000.0);
      
      // get YoVariables
      BooleanYoVariable walk = (BooleanYoVariable) scs.getVariable("DesiredFootstepCalculatorFootstepProviderWrapper", "walk");
      BooleanYoVariable enable = (BooleanYoVariable) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");
      String prefixLeft = fullRobotModel.getFoot(RobotSide.LEFT).getName();
      String prefixRight = fullRobotModel.getFoot(RobotSide.RIGHT).getName();
      final BooleanYoVariable rightFootInSwing = (BooleanYoVariable) scs.getVariable(
            prefixRight + "FootControlModule", prefixRight + "IsUnconstrained");
      final BooleanYoVariable leftFootInSwing = (BooleanYoVariable) scs.getVariable(
            prefixLeft + "FootControlModule", prefixLeft + "IsUnconstrained");
      swingStartConditionRight = new SingleSupportStartCondition(rightFootInSwing);
      swingStartConditionLeft = new SingleSupportStartCondition(leftFootInSwing);
      
      // simulate for a while
      enable.set(true);
      walk.set(false);
      blockingSimulationRunner.simulateAndBlock(1.0);
      walk.set(true);
      blockingSimulationRunner.simulateAndBlock(2.0);
	}

	private DRCFlatGroundWalkingTrack setupTrack(DRCRobotModel robotModel) {
		DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);
		guiInitialSetup.setIsGuiShown(SHOW_GUI);
		GroundProfile groundProfile = new FlatGroundProfile();

		DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(
				groundProfile, robotModel.getSimulateDT(), false);
		scsInitialSetup.setInitializeEstimatorToActual(true);
		scsInitialSetup.setDrawGroundProfile(true);

		DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel
				.getDefaultRobotInitialSetup(0.0, 0.0);

		DRCFlatGroundWalkingTrack drcFlatGroundWalkingTrack = new DRCFlatGroundWalkingTrack(
				robotInitialSetup, guiInitialSetup, scsInitialSetup, true,
				false, robotModel);

		drcSimulation = drcFlatGroundWalkingTrack.getDrcSimulation();
		return drcFlatGroundWalkingTrack;
	}
	
  private void testPush(Vector3d forceDirection, double magnitude, double duration, double percentInSwing,
         RobotSide side) throws SimulationExceededMaximumTimeException, InterruptedException
   {
      double delay = swingTime * percentInSwing;
      
      if(side == RobotSide.LEFT)
      {
         pushRobotController.applyForceDelayed(swingStartConditionLeft, delay, forceDirection, magnitude, duration);
      }
      else
      {
         pushRobotController.applyForceDelayed(swingStartConditionRight, delay, forceDirection, magnitude, duration);
      }
      
      blockingSimulationRunner.simulateAndBlock(8.0);
   }
	
	private class SingleSupportStartCondition implements StateTransitionCondition
   {
      private final BooleanYoVariable footInSwing;
      private boolean isInSwing;
      
      public SingleSupportStartCondition(BooleanYoVariable footInSwing)
      {
         this.footInSwing = footInSwing;
      }

      @Override
      public boolean checkCondition()
      {
         if(isInSwing)
         {
            if(!footInSwing.getBooleanValue())
            {
               isInSwing = false;
            }
            
            return false;
         }
         
         if(footInSwing.getBooleanValue())
         {
            isInSwing = true;
            return true;
         }
         
         return false;
      }
   }
}
