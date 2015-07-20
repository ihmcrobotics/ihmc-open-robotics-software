package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.LinkedHashMap;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TaskspaceToJointspaceCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataReadOnly;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.controllers.YoPIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoUtilities.math.filters.RateLimitedYoVariable;
import us.ihmc.yoUtilities.math.trajectories.PoseTrajectoryGenerator;

public class TaskspaceToJointspaceHandForcefeedbackControlState extends TrajectoryBasedTaskspaceHandControlState
{
	private final static boolean TRAJECTORY_FORCEFEEDBACK = false; //DO NOT SET TO TRUE! NOT ENTIRELY TESTED YET!! T.Meier
	private final static boolean SIMULATION = false;
	private final BooleanYoVariable forcefeedback;

	private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
	private final ReferenceFrame handControlFrame;
	private PoseTrajectoryGenerator poseTrajectoryGenerator;

	private final FramePose desiredPose = new FramePose();

	private final FramePoint desiredPosition = new FramePoint(worldFrame);
	private final FrameVector desiredVelocity = new FrameVector(worldFrame);
	private final FrameVector desiredAcceleration = new FrameVector(worldFrame);

	private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
	private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
	private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

	private TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator;

	private enum JointControlMode
	{
		FORCE_CONTROL, POSITION_CONTROL
	};

	private JointControlMode jointControlMode;

	private final double dtControl;
	private final OneDoFJoint[] oneDoFJoints;
	private final boolean[] doIntegrateDesiredAccelerations;
	private final LinkedHashMap<OneDoFJoint, PIDController> jointPIDControllers;
	private final LinkedHashMap<OneDoFJoint, RateLimitedYoVariable> jointRateLimitedAccelerations;
	private final DoubleYoVariable jointMaxAcceleration;

	private final DoubleYoVariable currentTimeInState;

	/**
	 *  MP/MPA Controller:
	 */
	// Force measurement
	private final String sensorPrefix;
	private ForceSensorDataReadOnly wristSensor;
	private final Point3d lastPositionInWorld = new Point3d();
	private final Point3d currentPositionInWorld = new Point3d();
	private Wrench wristSensorWrench = new Wrench();
	private final DoubleYoVariable fxRaw, fyRaw, fzRaw;
	private final AlphaFilteredYoVariable fxFiltered, fyFiltered, fzFiltered;
	private final double alpha;
	private double massHandandDrill;
	private static final double GRAVITY = 9.81;
	private static final double MASSDRILL = 1.442;
	private Vector3d circularTrajectoryOmega;
	private Vector3d circularTrajectoryRadius;
	private Point3d circularTrajectoryOrigin;
	private final IntegerYoVariable trajectoryNumber;

	// Cut force model
	private final DoubleYoVariable coeffC1, coeffC2;

	//MPA
	private double lnC1, c2;
	private final DoubleYoVariable epsilon;
	private final DoubleYoVariable modelError;
	private final DoubleYoVariable referenceError;
	private final DoubleYoVariable w1;
	private final DoubleYoVariable w2;
	private final double w1_0 = 1.0;
	private final double w2_0 = 8.0;

	//MPC
	private boolean trajectoryIsDone = false;
	private final DoubleYoVariable scaledTimeVariable;
	private final DoubleYoVariable timeParameterScaleFactor;
	private final static double MAX_TIME_SCALE = 3.0;
	private final DoubleYoVariable currentTangentialForce;
	private final DoubleYoVariable currentTangentialForceModel;
	private final DoubleYoVariable currentTangentialVelocity;
	private final DoubleYoVariable mpcVelocity;
	private final DoubleYoVariable desiredTangentialForce;
	private final DoubleYoVariable pCutControl;

	private final double referenceForce = -10.0;
	private final double pressForce = -5.0;

	private final FramePoint preScalingTrajectoryPosition = new FramePoint(worldFrame);
	private final FrameVector preScalingTrajectoryVelocity = new FrameVector(worldFrame);
	private final FrameVector preScalingTrajectoryAcceleration = new FrameVector(worldFrame);
	private double preScalingTrajectoryVelocityDouble;

	private final FramePose tempHandPose = new FramePose();
	private Vector3d tempVector = new Vector3d();
	private Vector3d tempVector2 = new Vector3d();
	private Vector3d forceVectorInWorld = new Vector3d();
	private Vector3d normalVectorInWorld = new Vector3d();
	private Vector3d tangentTrajectoryVectorInWorld = new Vector3d();
	private Point3d currentTrajectoryPositionInWorld = new Point3d();
	private Point3d lastTrajectoryPositionInWorld = new Point3d();
	private Point3d precalTrajectoryPositionInWorld = new Point3d();

	private final FramePoint currentHandPos = new FramePoint();
	private final FramePoint lastHandPos = new FramePoint();

	// Sensor noise for simulation
	private final long random = System.currentTimeMillis();
	private final Random randomGenerator;

	public TaskspaceToJointspaceHandForcefeedbackControlState(String namePrefix, HandControlState stateEnum, RobotSide robotSide,
			MomentumBasedController momentumBasedController, int jacobianId, RigidBody base, RigidBody endEffector, boolean doPositionControlOnJoints,
			YoPIDGains gains, YoVariableRegistry parentRegistry)
	{
		super(namePrefix, stateEnum, momentumBasedController, jacobianId, base, endEffector, parentRegistry);
		handControlFrame = endEffector.getBodyFixedFrame();
		currentTimeInState = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "CurrentTimeInCutForceControlState", registry);
		dtControl = momentumBasedController.getControlDT();
		oneDoFJoints = ScrewTools.createOneDoFJointPath(base, endEffector);
		doIntegrateDesiredAccelerations = new boolean[oneDoFJoints.length];

		forcefeedback = new BooleanYoVariable(robotSide.getShortLowerCaseName() + "_forceFeedback", parentRegistry);
		forcefeedback.set(TRAJECTORY_FORCEFEEDBACK);

		// TODO: Add mass of drill.
		massHandandDrill = TotalMassCalculator.computeSubTreeMass(momentumBasedController.getFullRobotModel().getHand(robotSide)) + MASSDRILL;

		for (int i = 0; i < oneDoFJoints.length; i++)
		{
			doIntegrateDesiredAccelerations[i] = oneDoFJoints[i].getIntegrateDesiredAccelerations();
		}

		if (doPositionControlOnJoints)
		{
			jointControlMode = JointControlMode.POSITION_CONTROL;

			// Position Controlled joints, for application on real robot.
			jointPIDControllers = null;
			jointRateLimitedAccelerations = null;
			jointMaxAcceleration = null;
		}
		else
		{
			jointControlMode = JointControlMode.FORCE_CONTROL;
			// Torque Controlled joints, for simulation
			jointPIDControllers = new LinkedHashMap<OneDoFJoint, PIDController>();
			jointRateLimitedAccelerations = new LinkedHashMap<OneDoFJoint, RateLimitedYoVariable>();
			jointMaxAcceleration = gains.getYoMaximumAcceleration();

			for (OneDoFJoint joint : oneDoFJoints)
			{
				String suffix = FormattingTools.lowerCaseFirstLetter(joint.getName());
				PIDController pidController = new PIDController(gains.getYoKp(), gains.getYoKi(), gains.getYoKd(), gains.getYoMaxIntegralError(), suffix, registry);
				jointPIDControllers.put(joint, pidController);

				RateLimitedYoVariable rateLimitedAcceleration = new RateLimitedYoVariable(suffix + "Acceleration", registry, gains.getYoMaximumJerk(), dtControl);
				jointRateLimitedAccelerations.put(joint, rateLimitedAcceleration);
			}
		}
		PrintTools.warn(this, "Joint control Mode: " + jointControlMode, true);	

		wristSensor = momentumBasedController.getWristForceSensor(robotSide);
		sensorPrefix = robotSide.getShortLowerCaseName() + "_wristSensor";

		alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0, dtControl);
		fxRaw = new DoubleYoVariable(sensorPrefix + "_Fx_raw", registry);
		fyRaw = new DoubleYoVariable(sensorPrefix + "_Fy_raw", registry);
		fzRaw = new DoubleYoVariable(sensorPrefix + "_Fz_raw", registry);
		fxFiltered = new AlphaFilteredYoVariable(sensorPrefix + "_Fx_filtered", registry, alpha, fxRaw);
		fyFiltered = new AlphaFilteredYoVariable(sensorPrefix + "_Fy_filtered", registry, alpha, fyRaw);
		fzFiltered = new AlphaFilteredYoVariable(sensorPrefix + "_Fz_filtered", registry, alpha, fzRaw);
		trajectoryNumber = new IntegerYoVariable(sensorPrefix + "_trajectory_number", registry);
		trajectoryNumber.set(0);

		currentTangentialForce = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_F_tangential", registry);
		currentTangentialForceModel = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_F_tangential_model", registry);
		currentTangentialVelocity = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_v_tangential", registry);
		scaledTimeVariable = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_scaled_time", registry);
		timeParameterScaleFactor = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_scale_factor", registry);
		mpcVelocity = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_v_MPC", registry);

		desiredTangentialForce = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_F_tangential_desired", registry);
		pCutControl = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_P_Control", registry);
		pCutControl.set(0.2);
		desiredTangentialForce.set(referenceForce);

		coeffC1 = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_C1", registry);
		coeffC2 = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_C2", registry);
		epsilon = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_epsilon", registry);
		modelError = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_model_error", registry);
		referenceError = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_reference_error", registry);

		//TODO: find reasonable initial coeffs
		coeffC1.set(90.00);//100.0);
		coeffC2.set(1.0);//2.0);
		epsilon.set(0.0);
		lnC1 = Math.log(coeffC1.getDoubleValue());
		c2 = coeffC2.getDoubleValue();

		w1 = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_w1", registry);
		w2 = new DoubleYoVariable(robotSide.getShortLowerCaseName() + "_w2", registry);
		w1.set(w1_0);
		w2.set(w2_0);

		if (SIMULATION && TRAJECTORY_FORCEFEEDBACK)
		{
			randomGenerator = new Random(random);
		}
		else
		{
			randomGenerator = null;
		}
	}

	private void resetFilterVariables(){
		fxFiltered.reset();
		fyFiltered.reset();
		fzFiltered.reset();
	}

	private void getTangentForce()
	{
		/**
		 * Get tangent vector of trajectory to project force on it
		 */
		poseTrajectoryGenerator.compute(scaledTimeVariable.getDoubleValue() + dtControl);
		poseTrajectoryGenerator.get(tempHandPose);
		poseTrajectoryGenerator.packLinearData(preScalingTrajectoryPosition, preScalingTrajectoryVelocity, preScalingTrajectoryAcceleration);
		preScalingTrajectoryVelocityDouble = preScalingTrajectoryVelocity.length();
		if (tempHandPose.getReferenceFrame() != worldFrame)
		{
			tempHandPose.changeFrame(worldFrame);
		}
		tempHandPose.getPosition(precalTrajectoryPositionInWorld);

		poseTrajectoryGenerator.compute(scaledTimeVariable.getDoubleValue());
		poseTrajectoryGenerator.get(tempHandPose);
		if (tempHandPose.getReferenceFrame() != worldFrame)
		{
			tempHandPose.changeFrame(worldFrame);
		}
		tempHandPose.getPosition(currentTrajectoryPositionInWorld);
		tangentTrajectoryVectorInWorld.sub(precalTrajectoryPositionInWorld, currentTrajectoryPositionInWorld);


		wristSensorUpdate(forceVectorInWorld, scaledTimeVariable.getDoubleValue());

		if (tangentTrajectoryVectorInWorld.length() > 0.0)
		{
			tangentTrajectoryVectorInWorld.normalize();
			if (SIMULATION)
			{
				currentTangentialForce.set(forceVectorInWorld.dot(tangentTrajectoryVectorInWorld)
						//                  + generateSinusoidalDisturbance(10.0, 0.1, Math.PI / 3.0)
						//                  + generateDriftDisturbance(0.01)
						);
			}
			else
			{
				currentTangentialForce.set(forceVectorInWorld.dot(tangentTrajectoryVectorInWorld));
			}
		}
		else
		{
			trajectoryIsDone = true;
			currentTangentialForce.set(0.0);
		}

		lastTrajectoryPositionInWorld.set(currentTrajectoryPositionInWorld);
	}

	private void mpaMpcControl()
	{
		getTangentForce();

		// calculate current velocity in desired trajectory plane by finite differences:
		currentHandPos.setToZero(handControlFrame);
		currentHandPos.changeFrame(worldFrame);
		lastHandPos.changeFrame(worldFrame);
		tempVector.set(currentHandPos.getPoint());
		tempVector.sub(lastHandPos.getPoint());
		tempVector.scale(1.0 / dtControl);
		tempVector.setX(0.0);
		lastHandPos.setIncludingFrame(currentHandPos);
		currentTangentialVelocity.set(tempVector.length());

		mpcVelocity.set(0.0);

		/**
		 * Control:
		 */
		if (trajectoryIsDone == false /*&& timeParameterScaleFactor.getDoubleValue() */)
		{
			// MPA
			currentTangentialForceModel.set(coeffC1.getDoubleValue() * (Math.exp(coeffC2.getDoubleValue() * currentTangentialVelocity.getDoubleValue()) - 1.0));


			// model defined for positive forces: abs()
			//			epsilon.set(Math.log(Math.abs(currentTangentialForce.getDoubleValue()) + coeffC1.getDoubleValue())
			//					- Math.log(currentTangentialForceModel.getDoubleValue() + coeffC1.getDoubleValue()));
			//			modelError.set(currentTangentialForce.getDoubleValue() + currentTangentialForceModel.getDoubleValue());
			//			referenceError.set(currentTangentialForce.getDoubleValue() - desiredTangentialForce.getDoubleValue());
			//
			//			lnC1 += w1.getDoubleValue() * epsilon.getDoubleValue();
			//			c2 += w2.getDoubleValue() * c2 * epsilon.getDoubleValue();
			//			coeffC1.set(Math.exp(lnC1));
			//			coeffC2.set(c2);
			//			
			//			
			//			if(coeffC2.getDoubleValue() < 0.01)
			//			{
			//				coeffC2.set(0.01);
			//				c2 = 0.01;
			//			}

			//			MPC
			mpcVelocity.set(1.0 / coeffC2.getDoubleValue() * Math.log(Math.abs(desiredTangentialForce.getDoubleValue()) / coeffC1.getDoubleValue() + 1.0));

		}
		else
		{
			currentTangentialForceModel.set(coeffC1.getDoubleValue() * (Math.exp(coeffC2.getDoubleValue() * currentTangentialVelocity.getDoubleValue()) - 1.0));

			mpcVelocity.set(0.0);
		}

		if (preScalingTrajectoryVelocityDouble == 0)
		{
			// Either beginning or end of Trajectory, scale factor = 1.0
			timeParameterScaleFactor.set(1.0);
		}
		else
		{
			timeParameterScaleFactor.set(mpcVelocity.getDoubleValue() / preScalingTrajectoryVelocityDouble + referenceError.getDoubleValue()
					* 0.0/*pCutControl.getDoubleValue()*/);
		}
		// Check if scaled too much:
		if (timeParameterScaleFactor.getDoubleValue() > MAX_TIME_SCALE)
		{
			timeParameterScaleFactor.set(MAX_TIME_SCALE);
			PrintTools.warn(this, "Scale factor saturation.");
		}



		// DEBUG: SET TO 1 anyway
		//    timeParameterScaleFactor.set(1.0);

		// evaluate the trajectory at new parameter value
		scaledTimeVariable.add(dtControl * timeParameterScaleFactor.getDoubleValue());
		poseTrajectoryGenerator.compute(scaledTimeVariable.getDoubleValue());

		poseTrajectoryGenerator.get(desiredPose);

		poseTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredAcceleration);

		// TODO:
		/**
		 * PID Control in direction of wall
		 */
		//		desiredPosition.checkReferenceFrameMatch(worldFrame);
		//		
		//		forceVectorInWorld.scale(1.0);
		//		forceVectorInWorld.dot(normalVectorInWorld);
		//		
		//		tempVector.set(-0.01*(forceVectorInWorld.length() - Math.abs(pressForce)), 0.0, 0.0);
		//		desiredPosition.add(tempVector);

		desiredVelocity.scale(timeParameterScaleFactor.getDoubleValue());
		desiredAcceleration.scale(timeParameterScaleFactor.getDoubleValue() * timeParameterScaleFactor.getDoubleValue());
		poseTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
		desiredAngularVelocity.scale(timeParameterScaleFactor.getDoubleValue());
		desiredAngularAcceleration.scale(timeParameterScaleFactor.getDoubleValue() * timeParameterScaleFactor.getDoubleValue());

		ReferenceFrame controlFrame = taskspaceToJointspaceCalculator.getControlFrame();
		desiredVelocity.changeFrame(controlFrame);
		desiredAngularVelocity.changeFrame(controlFrame);

		taskspaceToJointspaceCalculator.compute(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity);
		taskspaceToJointspaceCalculator.packDesiredJointAnglesIntoOneDoFJoints(oneDoFJoints);
		taskspaceToJointspaceCalculator.packDesiredJointVelocitiesIntoOneDoFJoints(oneDoFJoints);
		taskspaceToJointspaceCalculator.packDesiredJointAccelerationsIntoOneDoFJoints(oneDoFJoints);
	}

	private void trajectoryPositionControl()
	{
		scaledTimeVariable.set(currentTimeInState.getDoubleValue());

		getTangentForce();

		poseTrajectoryGenerator.compute(currentTimeInState.getDoubleValue());
		poseTrajectoryGenerator.get(desiredPose);
		poseTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredAcceleration);
		poseTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);
		trajectoryIsDone = poseTrajectoryGenerator.isDone();

		ReferenceFrame controlFrame = taskspaceToJointspaceCalculator.getControlFrame();

		desiredVelocity.changeFrame(controlFrame);
		desiredAngularVelocity.changeFrame(controlFrame);

		desiredPosition.changeFrame(controlFrame);
		desiredAcceleration.changeFrame(controlFrame);
		desiredOrientation.changeFrame(controlFrame);
		desiredAngularAcceleration.changeFrame(controlFrame);

		taskspaceToJointspaceCalculator.compute(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity);
		taskspaceToJointspaceCalculator.packDesiredJointAnglesIntoOneDoFJoints(oneDoFJoints);
		taskspaceToJointspaceCalculator.packDesiredJointVelocitiesIntoOneDoFJoints(oneDoFJoints);
		taskspaceToJointspaceCalculator.packDesiredJointAccelerationsIntoOneDoFJoints(oneDoFJoints);
	}

	private void wristSensorUpdate(Vector3d vectorToPack, double trajectoryParameter)
	{
		wristSensor.packWrench(wristSensorWrench);

		if (wristSensorWrench.getExpressedInFrame() != worldFrame)
		{
			wristSensorWrench.changeFrame(worldFrame);
		}

		if (SIMULATION)
		{
			addWhiteNoise(wristSensorWrench, 0.5);
		}

		fxRaw.set(wristSensorWrench.getLinearPartX());
		fyRaw.set(wristSensorWrench.getLinearPartY());
		fzRaw.set(wristSensorWrench.getLinearPartZ());
		// Compensate the gravity: 
		fzRaw.add(massHandandDrill * GRAVITY);

		lastPositionInWorld.set(currentPositionInWorld);
		wristSensor.getMeasurementFrame().getTransformToWorldFrame().getTranslation(currentPositionInWorld);


		// For a cicular trajectory the radial force must be compensated as well. Assume we move on circular trajectory: Fy gets filtered perfectly, some remaining errors in x and z 
		if (circularTrajectoryOrigin != null && trajectoryParameter > 0.0)
		{
			// Get radius and omega
			circularTrajectoryRadius.sub(currentPositionInWorld, circularTrajectoryOrigin);
			tempVector.sub(currentPositionInWorld, lastPositionInWorld);
			tempVector.scale(1.0 / (dtControl * circularTrajectoryRadius.length()));

			circularTrajectoryOmega.normalize();
			circularTrajectoryOmega.scale(tempVector.length());

			// radial force:
			tempVector2.cross(circularTrajectoryOmega, circularTrajectoryRadius);
			tempVector.cross(circularTrajectoryOmega, tempVector2);

			tempVector.scale(massHandandDrill);

			fxRaw.add(tempVector.getX());
			fyRaw.add(tempVector.getY());
			fzRaw.add(tempVector.getZ());
		}

		fxFiltered.update();
		fyFiltered.update();
		fzFiltered.update();

		vectorToPack.set(fxFiltered.getDoubleValue(), fyFiltered.getDoubleValue(), fzFiltered.getDoubleValue());
	}

	private void addWhiteNoise(Wrench wrench, double linearAmp)
	{
		tempVector.setX(randomGenerator.nextDouble() * linearAmp);
		tempVector.setY(randomGenerator.nextDouble() * linearAmp);
		tempVector.setZ(randomGenerator.nextDouble() * linearAmp);
		wrench.addLinearPart(tempVector);
	}

	@Override
	public boolean isDone() {
		return trajectoryIsDone;
	};

	@Override
	public void setTrajectory(PoseTrajectoryGenerator poseTrajectoryGenerator)
	{
		this.poseTrajectoryGenerator = poseTrajectoryGenerator;
	}

	@Override
	public void setHoldPositionDuration(double holdPositionDuration)
	{
		// TODO Auto-generated method stub

	}

	@Override
	public FramePose getDesiredPose()
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public ReferenceFrame getReferenceFrame()
	{
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void setControlModuleForForceControl(RigidBodySpatialAccelerationControlModule handRigidBodySpatialAccelerationControlModule)
	{
		// TODO Auto-generated method stub

	}

	@Override
	public void setControlModuleForPositionControl(TaskspaceToJointspaceCalculator taskspaceToJointspaceCalculator)
	{
		this.taskspaceToJointspaceCalculator = taskspaceToJointspaceCalculator;

	}

	public void initialize(Point3d rotationAxisOriginInWorld, Vector3d rotationAxisInWorld)
	{
		if (rotationAxisOriginInWorld != null)
		{
			normalVectorInWorld.set(rotationAxisInWorld);
			normalVectorInWorld.normalize();
			circularTrajectoryOrigin = new Point3d(rotationAxisOriginInWorld);
			circularTrajectoryOmega = new Vector3d(rotationAxisInWorld);
			circularTrajectoryRadius = new Vector3d();
		}
		else
		{
			normalVectorInWorld.set(rotationAxisInWorld);
			normalVectorInWorld.normalize();
			circularTrajectoryOrigin = null;
			circularTrajectoryOmega = null;
			circularTrajectoryRadius = null;
		}
	}

	@Override
	public void doTransitionIntoAction()
	{
		trajectoryNumber.add(1);
		resetFilterVariables();
		currentTimeInState.set(0.0);
		scaledTimeVariable.set(0.0);

		poseTrajectoryGenerator.showVisualization();
		poseTrajectoryGenerator.initialize();
		taskspaceToJointspaceCalculator.setSelectionMatrix(selectionMatrix);

		currentHandPos.setToZero(handControlFrame);
		lastHandPos.setToZero(handControlFrame);

		if (jointControlMode == JointControlMode.POSITION_CONTROL)
		{
			enableJointPositionControl();
		}
	}

	@Override
	public void doTransitionOutOfAction()
	{
		trajectoryIsDone = false;
		if (jointControlMode == JointControlMode.POSITION_CONTROL)
		{
			disableJointPositionControl();
		}
	}

	public void enableJointPositionControl()
	{
		for (OneDoFJoint joint : oneDoFJoints)
		{
			joint.setIntegrateDesiredAccelerations(false);
			joint.setUnderPositionControl(true);
		}
	}

	public void disableJointPositionControl()
	{
		for (int i = 0; i < oneDoFJoints.length; i++)
		{
			OneDoFJoint joint = oneDoFJoints[i];
			joint.setIntegrateDesiredAccelerations(doIntegrateDesiredAccelerations[i]);
			joint.setUnderPositionControl(false);
		}
	}

	private double generateSinusoidalDisturbance(double amp, double freq, double phase)
	{
		return Math.sin(2.0 * Math.PI * freq * currentTimeInState.getDoubleValue() + phase % (Math.PI * 2.0));
	}

	private double generateDriftDisturbance(double slope)
	{
		return slope * currentTimeInState.getDoubleValue();
	}

	private double step(double amp, double timestamp)
	{
		if (currentTimeInState.getDoubleValue() < timestamp)
		{
			return 0.0;
		}
		else
		{
			return -amp;
		}
	}

	@Override
	public void doAction()
	{
		currentTimeInState.set(getTimeInCurrentState());

		if (TRAJECTORY_FORCEFEEDBACK)
		{
			mpaMpcControl();
		}
		else
		{
			// Development mode: Position controlled Trajectory
			trajectoryPositionControl();
		}

		switch (jointControlMode)
		{
		case FORCE_CONTROL :
			for (int i = 0; i < oneDoFJoints.length; i++)
			{
				OneDoFJoint joint = oneDoFJoints[i];
				double q = joint.getQ();
				double qd = joint.getQd();
				double qDes = joint.getqDesired();
				double qdDes = joint.getQdDesired();

				RateLimitedYoVariable rateLimitedAcceleration = jointRateLimitedAccelerations.get(joint);

				PIDController pidController = jointPIDControllers.get(joint);
				double desiredAcceleration = pidController.computeForAngles(q, qDes, qd, qdDes, dtControl);

				desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, jointMaxAcceleration.getDoubleValue());
				rateLimitedAcceleration.update(desiredAcceleration);
				desiredAcceleration = rateLimitedAcceleration.getDoubleValue();

				momentumBasedController.setOneDoFJointAcceleration(joint, desiredAcceleration);
			}
			break;

		case POSITION_CONTROL :
			for (OneDoFJoint joint : oneDoFJoints)
			{
				// set desired qdd to zero.
				momentumBasedController.setOneDoFJointAcceleration(joint, 0.0);
			}
			break;

		default :
			PrintTools.error(this, "No joint control mode set.");
		}
	}
}