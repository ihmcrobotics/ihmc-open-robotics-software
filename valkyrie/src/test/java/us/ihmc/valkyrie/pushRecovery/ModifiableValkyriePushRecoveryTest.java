package us.ihmc.valkyrie.pushRecovery;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

import java.io.File;
import java.io.InputStream;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.testsupport.ValkyrieTestExporter;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

public class ModifiableValkyriePushRecoveryTest extends ValkyriePushRecoveryTest
{
	DRCRobotModel robotModel;

	public ModifiableValkyriePushRecoveryTest(DRCRobotModel robot) {
		super();
		robotModel = robot;
	}

	@Override
	protected DRCRobotModel getRobotModel()
	{
		return robotModel;
	}
	
	protected boolean runSim(DRCSimulationTestHelper helper, double simulationTime)
	{
		boolean testSucceeded = true;
		try {
    		testSucceeded = helper.simulateAndBlockAndCatchExceptions(simulationTime);
		} catch (SimulationExceededMaximumTimeException e) {
			testSucceeded = false;
		}	
		
		return testSucceeded;
	}

	public File PushICPOptimiWhileInSwing(Vector3D forceDirection, double magnitude, double duration, File outputDir) throws SimulationExceededMaximumTimeException
	{
		String dataNameSuffix = "PushICPOptimiWhileInSwing";
		String info = "PushICPOptimiWhileInSwing";
		boolean testSucceeded = true;

		setupTest(getScriptFilePath(), true, false);
		if (!runSim(drcSimulationTestHelper, 1.0)) {
			return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, false);	
		}

		// push timing:
		StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
		double delay = 0.5 * swingTime;

		// push parameters: values left for reference
		//      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
		//      double magnitude = 600.0 * getForceScale();
		//      double duration = 0.1;
		forceDirection.normalize();
		pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
		
		testSucceeded = runSim(drcSimulationTestHelper, simulationTime);		
		return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, testSucceeded);
	}

	public File PushWhileInSwing(Vector3D forceDirection, double magnitude, double duration, File outputDir) throws SimulationExceededMaximumTimeException
	{
		String dataNameSuffix = "PushWhileInSwing";
		String info = "PushWhileInSwing";
		boolean testSucceeded = true;

		setupTest(getScriptFilePath(), true, false);
		if (!runSim(drcSimulationTestHelper, 1.0)) {
			return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, false);	
		}		
		
		// push timing:
		StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
		double delayMultiplier = getPushWhileInSwingDelayMultiplier();
		double delay = delayMultiplier * swingTime;

		// push parameters: values left for reference
		//      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
		//      double magnitude = 550.0 * getForceScale();
		//      double duration = 0.1;
		forceDirection.normalize();
		pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
		
        testSucceeded = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime);
		
		return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, testSucceeded);	
	}

	public File RecoveringWithSwingSpeedUpWhileInSwing(Vector3D forceDirection, double magnitude, double duration, File outputDir) throws SimulationExceededMaximumTimeException
	{
		String dataNameSuffix = "RecoveringWithSwingSpeedUpWhileInSwing";
		String info = "RecoveringWithSwingSpeedUpWhileInSwing";
		boolean testSucceeded = true;

		setupTest(getScriptFilePath(), false, false);
		if (!runSim(drcSimulationTestHelper, 1.0)) {
			return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, false);	
		}		

		// push timing:
		StateTransitionCondition pushCondition = singleSupportStartConditions.get(RobotSide.RIGHT);
		double delay = 0.25 * swingTime;

		// push parameters: values left for reference
		//      Vector3D forceDirection = new Vector3D(0.0, -1.0, 0.0);
		//      double magnitude = 450.0 * getForceScale();
		//      double duration = 0.1;
		forceDirection.normalize();
		pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
   		testSucceeded = runSim(drcSimulationTestHelper, simulationTime);
		
		return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, testSucceeded);
	}

	public File PushWhileInTransfer(Vector3D forceDirection, double magnitude, double duration, File outputDir) throws SimulationExceededMaximumTimeException
	{
		setupTest(getScriptFilePath(), true, false);
		String dataNameSuffix = "PushWhileInTransfer";
		String info = "PushWhileInTransfer";
		boolean testSucceeded = false;

		if (!runSim(drcSimulationTestHelper, 1.0)) {
			return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, false);	
		}		

		// push timing:
		StateTransitionCondition pushCondition = doubleSupportStartConditions.get(RobotSide.RIGHT);
		double delay = 0.5 * transferTime;

		// push parameters: values left for reference
		//      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
		//      double magnitude = 450.0 * getForceScale();
		//      double duration = 0.1;
		forceDirection.normalize();
		pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
   		testSucceeded = runSim(drcSimulationTestHelper, simulationTime);
		
		return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, testSucceeded);
	}

	public File PushWhileStanding(Vector3D forceDirection, double magnitude, double duration, File outputDir) throws SimulationExceededMaximumTimeException
	{
		String dataNameSuffix = "PushWhileStanding";
		String info = "PushWhileStanding";
		boolean testSucceeded = true;

		setupTest(null, true, false);
		if (!runSim(drcSimulationTestHelper, 1.0)) {
			return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, false);	
		}		

		// push timing:
		StateTransitionCondition pushCondition = null;
		double delay = 1.0;

		// push parameters: values left for reference
		//      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
		//      double magnitude = 350.0 * getForceScale();
		//      double duration = 0.15;
		forceDirection.normalize();
		pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
   		testSucceeded = runSim(drcSimulationTestHelper, simulationTime);
		
		return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, testSucceeded);	
	}

	public File PushWhileStandingRecoveringAfterControllerFailureKickedIn(Vector3D forceDirection, double magnitude, double duration, File outputDir) throws SimulationExceededMaximumTimeException
	{
		String dataNameSuffix = "PushWhileStandingRecoveringAfterControllerFailureKickedIn";
		String info = "PushWhileStandingRecoveringAfterControllerFailureKickedIn";
		boolean testSucceeded = true;

		setupTest(null, false, true);
		if (!runSim(drcSimulationTestHelper, 1.0)) {
			return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, false);	
		}		

		// push timing:
		StateTransitionCondition pushCondition = null;
		double delay = 1.0;

		// push parameters: values left for reference
		//      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
		//      double magnitude = 350.0 * getForceScale();
		//      double duration = 0.15;
		forceDirection.normalize();
		pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
   		testSucceeded = runSim(drcSimulationTestHelper, simulationTime);
		
		return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, testSucceeded);	
	}

	public File LongForwardPushWhileStanding(Vector3D forceDirection, double magnitude, double duration, File outputDir) throws SimulationExceededMaximumTimeException
	{
		String dataNameSuffix = "LongForwardPushWhileStanding";
		String info = "LongForwardPushWhileStanding";
		boolean testSucceeded = true;

		setupTest(null, true, false);
		if (!runSim(drcSimulationTestHelper, 1.0)) {
			return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, false);	
		}		
		
		// push timing:
		StateTransitionCondition pushCondition = null;
		double delay = 0.0;

		// push parameters: values left for reference
		//      Vector3D forceDirection = new Vector3D(1.0, 0.0, 0.0);
		//      double magnitude = 100.0 * getForceScale();
		//      double duration = 1.0;
		forceDirection.normalize();
		pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
    	testSucceeded = runSim(drcSimulationTestHelper, duration + 2.0);

    	return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, testSucceeded);	
	}

	public File LongBackwardPushWhileStanding(Vector3D forceDirection, double magnitude, double duration, File outputDir) throws SimulationExceededMaximumTimeException
	{
		String dataNameSuffix = "LongBackwardPushWhileStanding";
		String info = "LongBackwardPushWhileStanding";
		boolean testSucceeded = true;

		setupTest(null, true, false);
		
		if (!runSim(drcSimulationTestHelper, 1.0)) {
			return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, false);	
		}		

		// push timing:
		StateTransitionCondition pushCondition = null;
		double delay = 0.0;

		// push parameters: values left for reference
		//      Vector3D forceDirection = new Vector3D(-1.0, 0.0, 0.0);
		//      double magnitude = 100.0 * getForceScale();
		//      double duration = 1.0;
		forceDirection.normalize();      
		pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
   		testSucceeded = runSim(drcSimulationTestHelper, duration + 2.0);
		
		return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, testSucceeded);	
	}

	public File RecoveryWhileInFlamingoStance(Vector3D forceDirection, double magnitude, double duration, File outputDir) throws SimulationExceededMaximumTimeException
	{
		String dataNameSuffix = "RecoveryWhileInFlamingoStance";
		String info = "RecoveryWhileInFlamingoStance";

		setupTest(null, false, false);
		if (!runSim(drcSimulationTestHelper, 1.0)) {
			return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, false);	
		}		
		RobotSide footSide = RobotSide.LEFT;
		FramePose3D footPose = new FramePose3D(
				drcSimulationTestHelper.getAvatarSimulation().getControllerFullRobotModel().getEndEffectorFrame(footSide, LimbName.LEG));
		footPose.changeFrame(ReferenceFrame.getWorldFrame());
		footPose.prependTranslation(0.0, 0.0, 0.2);
		Point3D desiredFootPosition = new Point3D();
		Quaternion desiredFootOrientation = new Quaternion();
		footPose.get(desiredFootPosition, desiredFootOrientation);
		FootTrajectoryMessage footPosePacket = HumanoidMessageTools.createFootTrajectoryMessage(footSide, 0.6, desiredFootPosition, desiredFootOrientation);
		drcSimulationTestHelper.publishToController(footPosePacket);
		if (!runSim(drcSimulationTestHelper, 2.0)) {
			return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, false);	
		}		

		// push timing:
		StateTransitionCondition pushCondition = null;
		double delay = 0.0;

		// push parameters: values left for reference
		//      Vector3D forceDirection = new Vector3D(0.0, 1.0, 0.0);
		//      double magnitude = 180.0 * getForceScale();
		//      double duration = 0.2;
		forceDirection.normalize();      
		pushRobotController.applyForceDelayed(pushCondition, delay, forceDirection, magnitude, duration);
		boolean testSucceeded = runSim(drcSimulationTestHelper, 1.5);
		return ValkyrieTestExporter.exportSimData(drcSimulationTestHelper.getSimulationConstructionSet(), outputDir, dataNameSuffix, info, testSucceeded);	
	}

	private void setupTest(String scriptName, boolean enablePushRecoveryControlModule, boolean enablePushRecoveryOnFailure) throws SimulationExceededMaximumTimeException
	{
		FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
		drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
		drcSimulationTestHelper.setTestEnvironment(flatGround);
		drcSimulationTestHelper.createSimulation("DRCSimpleFlatGroundScriptTest");
		FullHumanoidRobotModel fullRobotModel = getRobotModel().createFullRobotModel();
		//      pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel);
		pushRobotController = new PushRobotController(drcSimulationTestHelper.getRobot(), fullRobotModel.getChest().getParentJoint().getName(), new Vector3D(0, 0, getPushPositionZHeightInChestFrame()));
		SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
		scs.addYoGraphic(pushRobotController.getForceVisualizer());

		if (scriptName != null && !scriptName.isEmpty())
		{
			assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.001));
			InputStream scriptInputStream = getClass().getClassLoader().getResourceAsStream(scriptName);
			drcSimulationTestHelper.loadScriptFile(scriptInputStream, ReferenceFrame.getWorldFrame());
		}

		// get rid of this once push recovery is enabled by default
		YoBoolean enable = (YoBoolean) scs.getVariable("PushRecoveryControlModule", "enablePushRecovery");
		enable.set(enablePushRecoveryControlModule);
		YoBoolean enableOnFailure = (YoBoolean) scs.getVariable(WalkingHighLevelHumanoidController.class.getSimpleName(),
				"enablePushRecoveryOnFailure");
		enableOnFailure.set(enablePushRecoveryOnFailure);

		for (RobotSide robotSide : RobotSide.values)
		{
			String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
			String footPrefix = sidePrefix + "Foot";
			@SuppressWarnings("unchecked")
			final YoEnum<ConstraintType> footConstraintType = (YoEnum<ConstraintType>) scs.getVariable(sidePrefix + "FootControlModule",
					footPrefix + "CurrentState");
			@SuppressWarnings("unchecked")
			final YoEnum<WalkingStateEnum> walkingState = (YoEnum<WalkingStateEnum>) scs.getVariable("WalkingHighLevelHumanoidController",
					"walkingCurrentState");
			singleSupportStartConditions.put(robotSide, new SingleSupportStartCondition(footConstraintType));
			doubleSupportStartConditions.put(robotSide, new DoubleSupportStartCondition(walkingState, robotSide));
		}

		setupCamera(scs);
		swingTime = getRobotModel().getWalkingControllerParameters().getDefaultSwingTime();
		transferTime = getRobotModel().getWalkingControllerParameters().getDefaultTransferTime();
		ThreadTools.sleep(1000);
	}

	private void setupCamera(SimulationConstructionSet scs)
	{
		Point3D cameraFix = new Point3D(0.0, 0.0, 0.89);
		Point3D cameraPosition = new Point3D(10.0, 2.0, 1.37);
		drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
	}

	private class SingleSupportStartCondition implements StateTransitionCondition
	{
		private final YoEnum<ConstraintType> footConstraintType;

		public SingleSupportStartCondition(YoEnum<ConstraintType> footConstraintType)
		{
			this.footConstraintType = footConstraintType;
		}

		@Override
		public boolean testCondition(double time)
		{
			return footConstraintType.getEnumValue() == ConstraintType.SWING;
		}
	}

	private class DoubleSupportStartCondition implements StateTransitionCondition
	{
		private final YoEnum<WalkingStateEnum> walkingState;

		private final RobotSide side;

		public DoubleSupportStartCondition(YoEnum<WalkingStateEnum> walkingState, RobotSide side)
		{
			this.walkingState = walkingState;
			this.side = side;
		}

		@Override
		public boolean testCondition(double time)
		{
			if (side == RobotSide.LEFT)
			{
				return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_LEFT_SUPPORT);
			}
			else
			{
				return (walkingState.getEnumValue() == WalkingStateEnum.TO_STANDING) || (walkingState.getEnumValue() == WalkingStateEnum.TO_WALKING_RIGHT_SUPPORT);
			}
		}
	} 
}
