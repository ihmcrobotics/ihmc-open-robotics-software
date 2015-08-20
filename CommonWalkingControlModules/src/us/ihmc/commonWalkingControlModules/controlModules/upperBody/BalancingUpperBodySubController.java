package us.ihmc.commonWalkingControlModules.controlModules.upperBody;

import java.util.EnumMap;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector2d;

import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.ArmControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineLungingControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.UpperBodySubController;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.tools.containers.ContainerTools;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePoint2d;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;


public class BalancingUpperBodySubController implements UpperBodySubController
{
	private final String name = "BalancingUpperBodySubController";
	private final YoVariableRegistry registry = new YoVariableRegistry(name);

	private final boolean LATCH_LUNGE_AXIS_TO_PITCH_OR_ROLL = true;
	public static final boolean SCALE_PITCH_ROLL_FEEDBACK_GAINS_PROPORTIONAL_TO_LUNGE_AXIS_PITCH_ROLL_COMPONENTS = false;  //Otherwise just set feedback to zero when lunging

	private final CouplingRegistry couplingRegistry;
	private final ProcessedSensorsInterface processedSensors;
	private final CommonHumanoidReferenceFrames referenceFrames;

	private final ArmControlModule armControlModule;
	private final SpineLungingControlModule spineControlModule;

	private final EnumMap<NeckJointName, PIDController> neckControllers = ContainerTools.createEnumMap(NeckJointName.class);
	private final EnumMap<NeckJointName, DoubleYoVariable> desiredNeckPositions = ContainerTools.createEnumMap(NeckJointName.class);

	private final double controlDT;
	private NeckTorques neckTorques = new NeckTorques();
	private Wrench totalUpperBodyWrench;

	private final StateMachine stateMachine;
	private final BooleanYoVariable forceControllerIntoState = new BooleanYoVariable("force" + name + "IntoState", registry);
	private final EnumYoVariable<BalancingUpperBodySubControllerState> forcedControllerState = new EnumYoVariable<BalancingUpperBodySubControllerState>("forced"
			+ name + "State", registry, BalancingUpperBodySubControllerState.class);

	private final double robotMass;
	private final double gravity;
	private final double maxCmpDisplacement;

	private final RigidBody chest;
	private final RigidBody pelvis;
	private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
	private final ReferenceFrame pelvisFrame;



	private final DoubleYoVariable maxChestAngleMagnitude = new DoubleYoVariable("maxChestAngle", registry);
	private final DoubleYoVariable predictedCMPscaling = new DoubleYoVariable("predictedCMPscaling", registry);

	private final YoFramePoint robotCoMPosition = new YoFramePoint("comGraphic", "", worldFrame, registry);
	private final YoFramePoint chestCoMPosition = new YoFramePoint("chestCoMGraphic", "", worldFrame, registry);

	private final YoFrameVector lungeAxisGraphic = new YoFrameVector("lungeAxisGraphic", "", worldFrame, registry);
	private final FrameVector2d chestAngularVelocityXY = new FrameVector2d(worldFrame);

	private final YoFramePoint2d icpDesired = new YoFramePoint2d("icpLunging", "", worldFrame, registry);
	private final YoFramePoint2d icpActual = new YoFramePoint2d("icpActual", "", worldFrame, registry);
	private FrameVector2d icpDesiredToicpActualVector = new FrameVector2d(worldFrame);   

	private final DoubleYoVariable chestAngle = new DoubleYoVariable("chestAngle", registry);
	private final DoubleYoVariable chestAngularVelocityMagnitudeAboutLungeAxis = new DoubleYoVariable("chestAngularVelocity", registry);

	private final DoubleYoVariable predictedTimeToDecelerateChest = new DoubleYoVariable("predictedTimeToDecelerateChest", registry);
	private final DoubleYoVariable actualTimeToDecelerateChest = new DoubleYoVariable("actualTimeToDecelerateChest", registry);

	private final YoFramePoint2d actualICPAtChestStop = new YoFramePoint2d("actualICPAtChestStop", "", worldFrame, registry);
	private final YoFramePoint2d predictedICPAtChestStopYo = new YoFramePoint2d("predictedICPAtChestStop", "", worldFrame, registry);

	private final YoFramePoint2d maxCopYo = new YoFramePoint2d("lungeAxisBoSIntersection", "", worldFrame, registry);


	private EnumMap<BalancingUpperBodySubControllerState, Double> timeOfStateStart;
	private EnumMap<BalancingUpperBodySubControllerState, Double> minimumElapsedTimeInState;
	private EnumMap<BalancingUpperBodySubControllerState, Double> chestAngleOnStateStart;
	private EnumMap<BalancingUpperBodySubControllerState, Double> chestAngularVelocityAboutLungeAxisOnStateStart;

	private EnumMap<BalancingUpperBodySubControllerState, Double> desiredSpineCmpDirection;
	private EnumMap<BalancingUpperBodySubControllerState, Double> desiredCopDirection;


	public BalancingUpperBodySubController(CouplingRegistry couplingRegistry, ProcessedSensorsInterface processedSensors,
			CommonHumanoidReferenceFrames referenceFrames, double controlDT, RigidBody chest, ArmControlModule armControlModule,
			SpineLungingControlModule spineControlModule, YoVariableRegistry parentRegistry,
			YoGraphicsListRegistry yoGraphicsListRegistry)
	{
		this.couplingRegistry = couplingRegistry;
		this.processedSensors = processedSensors;
		this.referenceFrames = referenceFrames;
		this.controlDT = controlDT;
		this.armControlModule = armControlModule;
		this.spineControlModule = spineControlModule;

		this.robotMass = processedSensors.getTotalMass();
		this.gravity = processedSensors.getGravityInWorldFrame().getZ();
		this.chest = chest;
		this.pelvis = processedSensors.getFullRobotModel().getPelvis();
		this.pelvisFrame = pelvis.getBodyFixedFrame();
		this.totalUpperBodyWrench = new Wrench(pelvisFrame, pelvisFrame);

		this.stateMachine = new StateMachine(name + "State", name + "SwitchTime", BalancingUpperBodySubControllerState.class, processedSensors.getYoTime(),
				registry);
		this.timeOfStateStart = new EnumMap<BalancingUpperBodySubController.BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
		this.minimumElapsedTimeInState = new EnumMap<BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
		this.chestAngleOnStateStart = new EnumMap<BalancingUpperBodySubController.BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
		this.chestAngularVelocityAboutLungeAxisOnStateStart = new EnumMap<BalancingUpperBodySubController.BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);

		this.maxCmpDisplacement = spineControlModule.computeMaxCmpDisplacement();
		this.desiredSpineCmpDirection = new EnumMap<BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);
		this.desiredCopDirection = new EnumMap<BalancingUpperBodySubControllerState, Double>(BalancingUpperBodySubControllerState.class);


		populateDynamicsGraphicObjects(yoGraphicsListRegistry);
		populateYoVariables();

		populateControllers();
		parentRegistry.addChild(registry);

		setGains();
		setParameters();
		displayWarningsForBooleans();

//		setUpStateMachineSimple();
		setUpStateMachineBalanceBeam();


	}

	public void doUpperBodyControl(UpperBodyTorques upperBodyTorquesToPack)
	{
		updateVariables();
		updateDynamicsGraphicObjects();

		stateMachine.doAction();
		if (!forceControllerIntoState.getBooleanValue())
		{
			stateMachine.checkTransitionConditions();
		} else
		{
			stateMachine.setCurrentState(forcedControllerState.getEnumValue());
		}


		armControlModule.doArmControl(upperBodyTorquesToPack.getArmTorques());


		this.doNeckControl();
		upperBodyTorquesToPack.setNeckTorques(neckTorques);

		spineControlModule.getSpineTorques(upperBodyTorquesToPack.getSpineTorques());

		spineControlModule.computeTotalWrenchExertedOnPelvis(totalUpperBodyWrench);
		couplingRegistry.setActualUpperBodyLungingWrench(totalUpperBodyWrench);

		if (currentStateElapsedTime() < 0.0)  //TODO: This prevents problems when rewinding, but is inefficient.
		{
			timeOfStateStart.put((BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum(), processedSensors.getTime());
		}

	}

	private void setUpStateMachineSimple()
	{
		State base = new BaseState();
		State icpRecoverAccelerateState = new ICPRecoverAccelerateState();
		State icpRecoverDecelerateState = new ICPRecoverDecelerateSpineState();

		StateTransitionCondition isICPOutsideLungeRadius = new DoWeNeedToStartLungingCondition();
		StateTransitionCondition doWeNeedToDecelerate = new DoWeNeedToDecelerateNow();
		StateTransitionCondition isBodyAngularVelocityZero = new HasChestAngularVelocityCrossedZeroCondition();

		StateTransition toICPRecoverAccelerate = new StateTransition(icpRecoverAccelerateState.getStateEnum(), isICPOutsideLungeRadius);
		StateTransition toICPRecoverDecelerate = new StateTransition(icpRecoverDecelerateState.getStateEnum(), doWeNeedToDecelerate);
		StateTransition toBase = new StateTransition(base.getStateEnum(), isBodyAngularVelocityZero);

		base.addStateTransition(toICPRecoverAccelerate);
		icpRecoverAccelerateState.addStateTransition(toICPRecoverDecelerate);
		icpRecoverDecelerateState.addStateTransition(toBase);   //TODO now going back to base

		stateMachine.addState(base);
		stateMachine.addState(icpRecoverAccelerateState);
		stateMachine.addState(icpRecoverDecelerateState);

		if (forceControllerIntoState.getBooleanValue())
		{
			stateMachine.setCurrentState(forcedControllerState.getEnumValue());
		} else
		{
			stateMachine.setCurrentState(base.getStateEnum());
		}

		minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.BASE, 100.0*controlDT);
		minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 10.0*controlDT);
		minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, 10.0*controlDT);
	}

	private void setUpStateMachineBalanceBeam()
	{
		State base = new BaseState();
		State icpRecoverAccelerateState = new ICPRecoverAccelerateState();
		State icpRecoverDecelerateState = new ICPRecoverDecelerateSpineState();
		State orientationRecoverAccelerateState = new OrientationRecoverAccelerateState();
		State orientationRecoverDecelerateState = new OrientationRecoverDecelerateState();

		StateTransitionCondition isICPOutsideLungeRadius = new DoWeNeedToStartLungingCondition();
		StateTransitionCondition doWeNeedToDecelerate = new DoWeNeedToDecelerateNow();
		StateTransitionCondition isBodyAngularVelocityZero = new HasChestAngularVelocityCrossedZeroCondition();
		StateTransitionCondition doWeNeedToDecelerateAgain = new DecelerateNowToStopUpperBodyAtUprightCondition();
		StateTransitionCondition isChestUpright = new isChestFrameAlignedWithGravityCondition();

		StateTransition toICPRecoverAccelerate = new StateTransition(icpRecoverAccelerateState.getStateEnum(), isICPOutsideLungeRadius);
		StateTransition toICPRecoverDecelerate = new StateTransition(icpRecoverDecelerateState.getStateEnum(), doWeNeedToDecelerate);
		StateTransition toOrientationRecoverAccelerate = new StateTransition(orientationRecoverAccelerateState.getStateEnum(), isBodyAngularVelocityZero);
		StateTransition toOrientationRecoverDecelerate = new StateTransition(orientationRecoverDecelerateState.getStateEnum(), doWeNeedToDecelerateAgain);
		StateTransition toBase = new StateTransition(base.getStateEnum(), isChestUpright);

		base.addStateTransition(toICPRecoverAccelerate);
		icpRecoverAccelerateState.addStateTransition(toICPRecoverDecelerate);
		icpRecoverDecelerateState.addStateTransition(toOrientationRecoverAccelerate);
		orientationRecoverAccelerateState.addStateTransition(toOrientationRecoverDecelerate);
		orientationRecoverDecelerateState.addStateTransition(toBase);

		stateMachine.addState(base);
		stateMachine.addState(icpRecoverAccelerateState);
		stateMachine.addState(icpRecoverDecelerateState);
		stateMachine.addState(orientationRecoverAccelerateState);
		stateMachine.addState(orientationRecoverDecelerateState);

		if (forceControllerIntoState.getBooleanValue())
		{
			stateMachine.setCurrentState(forcedControllerState.getEnumValue());
		} else
		{
			stateMachine.setCurrentState(base.getStateEnum());
		}

		minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.BASE, 100.0*controlDT);
		minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 10.0*controlDT);
		minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, 10.0*controlDT);
		minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.OR_REC_ACC, 10.0*controlDT);
		minimumElapsedTimeInState.put(BalancingUpperBodySubControllerState.OR_REC_DEC, 10.0*controlDT);
	}


	private enum BalancingUpperBodySubControllerState
	{
		BASE, ICP_REC_ACC, ICP_REC_DEC, OR_REC_ACC, OR_REC_DEC;
	}


	private class BaseState extends State
	{
		private BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.BASE;

		public BaseState()
		{
			super(BalancingUpperBodySubControllerState.BASE);
		}

		public void doTransitionIntoAction()
		{
			timeOfStateStart.put(currentState, processedSensors.getTime());
			chestAngleOnStateStart.put(currentState, chestAngle.getDoubleValue());
			setLungeAxisInWorldFrame(0.0, 0.0);
		}

		public void doAction()
		{
			spineControlModule.setSpineTorquesForZeroQdd();
//			spineControlModule.setSpineTorquesForGravityCancel();
		}

		public void doTransitionOutOfAction()
		{
			System.out.println("\n Exit Base State");
			SpineTorques torques = new SpineTorques();
			spineControlModule.getSpineTorques(torques);
		}
	}


	private class ICPRecoverAccelerateState extends State
	{
		private BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.ICP_REC_ACC;


		public ICPRecoverAccelerateState()
		{
			super(BalancingUpperBodySubControllerState.ICP_REC_ACC);
		}

		public void doAction()
		{
			spineControlModule.setSpineTorquesForDeltaCmpUsingID();
		}

		public void doTransitionIntoAction()
		{
			timeOfStateStart.put(currentState, processedSensors.getTime());

			setLungeAxisNormalToVectorFromIcpDesiredToIcpActual();

			chestAngleOnStateStart.put(currentState, chestAngle.getDoubleValue());
			setChestAngularVelocityAboutLungeAxisOnStateStart(currentState);

			setCmpInSpineControlModule(currentState, false);

		}

		public void doTransitionOutOfAction()
		{
			System.out.println("Exit ICPRecoverAccelerateSpineState");
		}
	}


	private class ICPRecoverDecelerateSpineState extends State
	{
		private BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.ICP_REC_DEC;

		public ICPRecoverDecelerateSpineState()
		{
			super(BalancingUpperBodySubControllerState.ICP_REC_DEC);
		}

		public void doAction()
		{
			spineControlModule.setSpineTorquesForDeltaCmpUsingID();
		}

		public void doTransitionIntoAction()
		{
			timeOfStateStart.put(currentState, processedSensors.getTime());
			setChestAngularVelocityAboutLungeAxisOnStateStart(currentState);
			flipVisualizedLungeAxisGraphic();

			setCmpInSpineControlModule(currentState, false);

			System.out.println("Desired ICP at Chest Stop: " + icpDesired);
			System.out.println("Predicted ICP at Chest Stop: " + predictedICPAtChestStopYo);
			System.out.println("Predicted Elapsed Time to Stop Chest: " + predictedTimeToStopChestDuring(currentState));
		}

		public void doTransitionOutOfAction()
		{
			actualTimeToDecelerateChest.set(currentStateElapsedTime());
			actualICPAtChestStop.set(couplingRegistry.getCapturePointInFrame(ReferenceFrame.getWorldFrame()).toFramePoint2d());

			System.out.println("Actual ICP at Chest Stop: " + actualICPAtChestStop);
			System.out.println("Actual Elapsed Time to Stop Chest: " + actualTimeToDecelerateChest.getDoubleValue());
			System.out.println("Exit ICPRecoverDecelerateSpineState");
		}
	}


	private class OrientationRecoverAccelerateState extends State
	{
		BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.OR_REC_ACC;


		public OrientationRecoverAccelerateState()
		{
			super(BalancingUpperBodySubControllerState.OR_REC_ACC);
		}

		public void doAction()
		{
			spineControlModule.setSpineTorquesForDeltaCmpUsingID();
		}

		public void doTransitionIntoAction()
		{
			timeOfStateStart.put(currentState, processedSensors.getTime());

			flipVisualizedLungeAxisGraphic();

			setCmpInSpineControlModule(currentState, false);
		}

		public void doTransitionOutOfAction()
		{

		}
	}

	private class OrientationRecoverDecelerateState extends State
	{
		BalancingUpperBodySubControllerState currentState = BalancingUpperBodySubControllerState.OR_REC_DEC;

		public OrientationRecoverDecelerateState()
		{
			super(BalancingUpperBodySubControllerState.OR_REC_DEC);
		}

		public void doAction()
		{
			spineControlModule.setSpineTorquesForDeltaCmpUsingID();
		}

		public void doTransitionIntoAction()
		{
			timeOfStateStart.put(BalancingUpperBodySubControllerState.OR_REC_DEC, processedSensors.getTime());

			flipVisualizedLungeAxisGraphic();

			setCmpInSpineControlModule(currentState, true);

		}

		public void doTransitionOutOfAction()
		{

		}
	}

	private void setLungeAxisNormalToVectorFromIcpDesiredToIcpActual()
	{
		setLungeAxisInWorldFrame( -icpDesiredToicpActualVector.getY(), icpDesiredToicpActualVector.getX());  // just rotate 90 degrees about z-axis
	}


	private void setLungeAxisInWorldFrame(double newX, double newY)
	{
		FrameVector2d tempLungeAxis = new FrameVector2d(ReferenceFrame.getWorldFrame(), newX, newY);

		if (tempLungeAxis.length() != 0.0)
		{
			tempLungeAxis.normalize();
			if (LATCH_LUNGE_AXIS_TO_PITCH_OR_ROLL)
			{
				tempLungeAxis.set(Math.round(tempLungeAxis.getX()),Math.round(tempLungeAxis.getY()));
			}
		}

		lungeAxisGraphic.set(tempLungeAxis.getX(), tempLungeAxis.getY(), 0.0);
		couplingRegistry.setLungeAxis(tempLungeAxis);

		spineControlModule.computeTotalUpperBodyMoIProjectedAbout(tempLungeAxis);
	}



	private void flipVisualizedLungeAxisGraphic()
	{
		Vector2d temp = couplingRegistry.getLungeAxisInFrame(ReferenceFrame.getWorldFrame()).getVectorCopy();
		temp.negate();
		lungeAxisGraphic.setX(temp.getX()); lungeAxisGraphic.setY(temp.getY());
	}

	
	private FramePoint2d getNominalCoP()
	{
		FrameConvexPolygon2d supportPolygon = couplingRegistry.getOldBipedSupportPolygons().getSupportPolygonInMidFeetZUp();
		
		FramePoint2d nominalCoP = supportPolygon.getCentroidCopy();
		
		return nominalCoP;
	}

	private double computeBosRadius(BalancingUpperBodySubControllerState state) 
	{
		FramePoint2d nominalCoP = getNominalCoP();
		FramePoint2d maxCoP = computeMaxCop(state);

		double deltaX = maxCoP.getX() - nominalCoP.getX();
		double deltaY = maxCoP.getY() - nominalCoP.getY();
		double bosRadius = Math.sqrt( deltaX*deltaX + deltaY*deltaY );
		
		return bosRadius;
	}
	

	private FramePoint2d computeIntersectionWithSupportPolygon(FramePoint2d insidePolygonPoint, FramePoint2d outsidePolygonPoint)
	{
		FrameConvexPolygon2d supportPolygon = couplingRegistry.getOldBipedSupportPolygons().getSupportPolygonInMidFeetZUp();
		FrameLineSegment2d lineSegment = new FrameLineSegment2d(insidePolygonPoint, outsidePolygonPoint);

		FramePoint2d[] intersectionPoints = lineSegment.intersectionWith(supportPolygon);
		FramePoint2d ret = intersectionPoints[0];

		// no intersections
		if (ret == null)
		{
			System.out.println("Could not find any intersection with BoS Polygon in " + this.name + "!  Returning outsidePolygonPoint.");
			ret = outsidePolygonPoint;
		}

		return ret;
	}


	private class DoWeNeedToStartLungingCondition implements StateTransitionCondition
	{
		public boolean checkCondition()
		{         
			return !isCapturePointInsideSupportPolygon() && isElapsedTimeInCurrentStateAboveMinimumThreshold();
		}
	}

	private double currentStateElapsedTime()
	{
		double deltaTimeInState = processedSensors.getTime() - timeOfStateStart.get((BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum());

		return deltaTimeInState;
	}

	private boolean isElapsedTimeInCurrentStateAboveMinimumThreshold()
	{
		boolean minimumElapsedTimeInCurrentState = currentStateElapsedTime() > minimumElapsedTimeInState.get((BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum());
		return minimumElapsedTimeInCurrentState;
	}

	private class DoWeNeedToDecelerateNow implements StateTransitionCondition
	{
		BalancingUpperBodySubControllerState currentState;
		
		ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
		FramePoint2d predictedICPWhenChestStops = new FramePoint2d(midFeetZUpFrame);

		public boolean checkCondition()
		{
			currentState = (BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum();

			boolean doWeNeedToDecelerateNow;

			double predictedChestAngleMagnitudeIfWeDecelerateNow = computeChestAngleMagnitudeAboutLungeAxis() + predictedDistanceToStopChestDuring(currentState);
			boolean doWeNeedToDecelerateBecauseOfAngleLimit = predictedChestAngleMagnitudeIfWeDecelerateNow > maxChestAngleMagnitude.getDoubleValue();

			if (doWeNeedToDecelerateBecauseOfAngleLimit)
			{
				doWeNeedToDecelerateNow = doWeNeedToDecelerateBecauseOfAngleLimit && isElapsedTimeInCurrentStateAboveMinimumThreshold();
				System.out.println("Transition to slow down due to Angle Limit.");
			}
			else
			{
				doWeNeedToDecelerateNow = willICPEndUpAtDesiredPositionIfWeDecelerateNow() && isElapsedTimeInCurrentStateAboveMinimumThreshold();
				if (doWeNeedToDecelerateNow) System.out.println("Transition to slow down due favorable predicted ICP location after slow down.");
			}


			return doWeNeedToDecelerateNow;
		}


		/**
		 * Predict where the ICP will be located when the chest comes to rest if we begin to decelerate the chest now.
		 * Assume that the CoP, net torque on the chest, and resulting CmP are constant during deceleration.
		 * First, predict the time it takes to decelerate the chest based upon the constant torque and a known chest moment of inertia.
		 * Using the first-order ICP dynamics, solve for the change in ICP position over this time period, given the anticipated *constant* CmP location.
		 * @return
		 */
		private boolean willICPEndUpAtDesiredPositionIfWeDecelerateNow()
		{  
			

			FramePoint2d icpActual = couplingRegistry.getCapturePointInFrame(midFeetZUpFrame).toFramePoint2d();
			FramePoint2d predictedCMPDuringChestSlowDown = computeExpectedCmpDuring(currentState, midFeetZUpFrame);
			FramePoint2d desiredICPWhenChestStops = getIcpDesired(); desiredICPWhenChestStops.changeFrame(midFeetZUpFrame);

			
			predictedICPWhenChestStops = predictICPWhenChestStopsIfWeSlowDownNow(icpActual, predictedCMPDuringChestSlowDown, predictedTimeToStopChestDuring(currentState));
			FramePoint2d predictedICPWhenChestStopsInWorld = new FramePoint2d(predictedICPWhenChestStops);
			predictedICPWhenChestStopsInWorld.changeFrame(worldFrame);
			predictedICPAtChestStopYo.set(predictedICPWhenChestStopsInWorld);

			FrameVector2d vectorFromDesiredToPredictedICP = new FrameVector2d(desiredICPWhenChestStops, predictedICPWhenChestStops);

			boolean hasVectorFromDesiredToPredictedIcpFlippedDirection = vectorFromDesiredToPredictedICP.dot(getNewUnitVectorNormalToLungeAxis(midFeetZUpFrame)) < 0.0;

			return hasVectorFromDesiredToPredictedIcpFlippedDirection;
		}
	}


	private void setChestAngularVelocityAboutLungeAxisOnStateStart(BalancingUpperBodySubControllerState currentState)
	{
		if (currentStateElapsedTime() > 0.0) throw new RuntimeException("Can only set this value at the transition into a state.");

		computeChestAngularVelocityAboutLungeAxis();

		chestAngularVelocityAboutLungeAxisOnStateStart.put(currentState, chestAngularVelocityMagnitudeAboutLungeAxis.getDoubleValue());
	}

	private void computeChestAngularVelocityAboutLungeAxis()
	{
		FrameVector chestAngularVelocityVector = processedSensors.getChestAngularVelocityInChestFrame();
		chestAngularVelocityVector.changeFrame(worldFrame);
		chestAngularVelocityXY.set(chestAngularVelocityVector.getX(), chestAngularVelocityVector.getY());

		FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(worldFrame);

		if (lungeAxis == null)
		{
			chestAngularVelocityMagnitudeAboutLungeAxis.set(0.0);
		}
		else
		{
			chestAngularVelocityMagnitudeAboutLungeAxis.set(chestAngularVelocityXY.dot( couplingRegistry.getLungeAxisInFrame(worldFrame) ) );
		}
	}


	private double computeChestAngleMagnitudeAboutLungeAxis()
	{
		ReferenceFrame expressedInFrame = ReferenceFrame.getWorldFrame();
		FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(expressedInFrame);

		double scaledAngle = 0.0;
		if (lungeAxis != null)
		{
			RigidBodyTransform chestTransformToWorld = chest.getBodyFixedFrame().getTransformToDesiredFrame(expressedInFrame);
			Quat4d chestToWorldQuat = new Quat4d();
			chestTransformToWorld.get(chestToWorldQuat);
			AxisAngle4d chestToWorldAxisAndAngle = new AxisAngle4d();
			chestToWorldAxisAndAngle.set(chestToWorldQuat);

			FrameVector2d axis = new FrameVector2d(expressedInFrame, chestToWorldAxisAndAngle.getX(), chestToWorldAxisAndAngle.getY());
			double angle = chestToWorldAxisAndAngle.angle;

			scaledAngle = angle * axis.dot(lungeAxis);
		}

		return scaledAngle;
	}



	private double predictedChestAccelerationDuring(BalancingUpperBodySubControllerState predictedState)
	{
		FrameVector2d deltaCMP = computeDesiredDeltaCmp(predictedState, worldFrame, false);
		FrameVector2d predictedSpineTorqueVector = spineControlModule.computeTorqueVectorForDeltaCMP(deltaCMP, false);

		double upperBodyMoIProjected = spineControlModule.getTotalUpperBodyMoIProjected();

		double predictedChestAngularDecel = predictedSpineTorqueVector.length() / upperBodyMoIProjected;

		return predictedChestAngularDecel;
	}


	private double predictedTimeToStopChestDuring(BalancingUpperBodySubControllerState predictedState)
	{
		double predictedTimeToStopChest = Math.abs(chestAngularVelocityMagnitudeAboutLungeAxis.getDoubleValue() / predictedChestAccelerationDuring(predictedState));

		predictedTimeToDecelerateChest.set(predictedTimeToStopChest);
		return predictedTimeToStopChest;
	}

	private double predictedDistanceToStopChestDuring(BalancingUpperBodySubControllerState state)
	{
		double deltaT = predictedTimeToStopChestDuring(state);
		double predictedStopDistance = chestAngularVelocityMagnitudeAboutLungeAxis.getDoubleValue() * deltaT - 0.5 * predictedChestAccelerationDuring(state) * deltaT * deltaT;

		return predictedStopDistance;
	}


	private FramePoint2d computeExpectedCmpDuring(BalancingUpperBodySubControllerState state, ReferenceFrame desiredFrame)
	{
		FramePoint2d expectedCmp = computeMaxCop(state);
		expectedCmp.changeFrame(desiredFrame);
		
		boolean clipTorqueToKeepCmpInsideBoS;
		if (state.equals(BalancingUpperBodySubControllerState.OR_REC_ACC) || state.equals(BalancingUpperBodySubControllerState.OR_REC_DEC))
		{
			clipTorqueToKeepCmpInsideBoS = true;
		}
		else
		{
			clipTorqueToKeepCmpInsideBoS = false;
		}
		
		expectedCmp.add(computeDesiredDeltaCmp(state, desiredFrame, clipTorqueToKeepCmpInsideBoS));

		expectedCmp.scale(predictedCMPscaling.getDoubleValue());  //Tune the desired overshoot for which to transition to chest deceleration
		return expectedCmp;
	}

	private FramePoint2d predictICPWhenChestStopsIfWeSlowDownNow(FramePoint2d icpCurrent, FramePoint2d cmpDuringChestDecelerate, double elapsedTimeToStop)
	{
		//Predict where the ICP will be at the time the chest comes to rest, if we start to decelerate NOW.
		//Assume a constant CmP location proportional to the maxHipTorque.
		double gravity = Math.abs(this.gravity);
		double CoMHeight = processedSensors.getCenterOfMassPositionInFrame(worldFrame).getZ();
		double omega0 = Math.sqrt( gravity / CoMHeight );

		// instantaneousCapturePoint(timeCurrent + elapsedTimeToStop) = cmp(timeCurrent) + [ instantaneousCapturePoint(timeCurrent) - cmp(timeCurrent) ] * exp(omega0 * elapsedTimeToStop)
		double deltaTPrime = omega0 * elapsedTimeToStop;

		FramePoint2d icpPredicted = new FramePoint2d(icpCurrent.getReferenceFrame(), icpCurrent.getPointCopy());
		icpPredicted.sub(cmpDuringChestDecelerate);
		icpPredicted.scale(Math.exp(deltaTPrime));
		icpPredicted.add(cmpDuringChestDecelerate);

		return icpPredicted;
	}




	private class HasChestAngularVelocityCrossedZeroCondition implements StateTransitionCondition
	{

		public HasChestAngularVelocityCrossedZeroCondition()
		{
		}

		public boolean checkCondition()
		{
			boolean ret = chestAngularVelocityMagnitudeAboutLungeAxis.getDoubleValue() < 1e-2 &&  isElapsedTimeInCurrentStateAboveMinimumThreshold();
			return ret;
		}
	}



	private class DecelerateNowToStopUpperBodyAtUprightCondition implements StateTransitionCondition
	{
		public boolean checkCondition()
		{
			BalancingUpperBodySubControllerState currentState = (BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum();
			double angularDisplacmentAboutLungeAxis = Math.abs( computeChestAngleMagnitudeAboutLungeAxis() );
			double predictedStopDisplacement = Math.abs(predictedDistanceToStopChestDuring(currentState));
			
			boolean ret = angularDisplacmentAboutLungeAxis - predictedStopDisplacement <= 0.0 && currentStateElapsedTime() > minimumElapsedTimeInState.get(currentState);
			
			return ret;
		}
	}

	private class isChestFrameAlignedWithGravityCondition implements StateTransitionCondition
	{
		public boolean checkCondition()
		{
			return chestAngle.getDoubleValue() < 1e-1 && isElapsedTimeInCurrentStateAboveMinimumThreshold();
		}
	}



	public boolean isCapturePointInsideSupportPolygon()
	{
		FrameConvexPolygon2d supportPolygon = couplingRegistry.getOldBipedSupportPolygons().getSupportPolygonInMidFeetZUp();
		FramePoint2d capturePoint = couplingRegistry.getCapturePointInFrame(supportPolygon.getReferenceFrame()).toFramePoint2d();

		return supportPolygon.isPointInside(capturePoint);
	}

	private void doNeckControl()
	{
		for (NeckJointName neckJointName : NeckJointName.values())
		{
			PIDController pidController = neckControllers.get(neckJointName);
			double desiredPosition = desiredNeckPositions.get(neckJointName).getDoubleValue();
			double desiredVelocity = 0.0;

			double actualPosition = processedSensors.getNeckJointPosition(neckJointName);
			double actualVelcoity = processedSensors.getNeckJointVelocity(neckJointName);

			double torque = pidController.compute(actualPosition, desiredPosition, actualVelcoity, desiredVelocity, controlDT);
			neckTorques.setTorque(neckJointName, torque);
		}
	}

	private void populateYoVariables()
	{
		for (NeckJointName neckJointName : NeckJointName.values())
		{
			String varName = "desired" + neckJointName.getCamelCaseNameForMiddleOfExpression();
			DoubleYoVariable variable = new DoubleYoVariable(varName, registry);

			desiredNeckPositions.put(neckJointName, variable);
		}
	}

	private void populateControllers()
	{
		for (NeckJointName neckJointName : NeckJointName.values())
		{
			neckControllers.put(neckJointName, new PIDController(neckJointName.getCamelCaseNameForStartOfExpression(), registry));
		}
	}

	private void populateDynamicsGraphicObjects(YoGraphicsListRegistry yoGraphicsListRegistry)
	{
		YoGraphicVector lungeAxisVisual = new YoGraphicVector("lungeAxisVisual", robotCoMPosition, lungeAxisGraphic, 1.0, YoAppearance.DarkRed());
		yoGraphicsListRegistry.registerYoGraphic(name, lungeAxisVisual);

		YoGraphicPosition icpDesiredVisual = new YoGraphicPosition("icpDesiredVisual", icpDesired, 0.05, YoAppearance.Yellow());
		yoGraphicsListRegistry.registerYoGraphic(name, icpDesiredVisual);

		YoGraphicPosition icpActualVisual = new YoGraphicPosition("icpActualVisual", icpActual, 0.05, YoAppearance.Blue());
		yoGraphicsListRegistry.registerYoGraphic(name, icpActualVisual);

		YoGraphicPosition icpPredictedVisual = new YoGraphicPosition("icpPredictedVisual", predictedICPAtChestStopYo, 0.05, YoAppearance.Purple());
		yoGraphicsListRegistry.registerYoGraphic(name, icpPredictedVisual);

		YoGraphicPosition lungeAxisPerpIntersectionWithBoSVisual = new YoGraphicPosition("lungeAxisPerpIntersectionWithBoSVisual", maxCopYo, 0.02, YoAppearance.Pink());
		yoGraphicsListRegistry.registerYoGraphic(name, lungeAxisPerpIntersectionWithBoSVisual);

		YoGraphicVector chestAxisAngleVisual = new YoGraphicVector("chestAxisVisual", chestCoMPosition, lungeAxisGraphic, 2.0, YoAppearance.DarkRed());
		yoGraphicsListRegistry.registerYoGraphic(name, chestAxisAngleVisual);

	}


	private void updateVariables()
	{
		updateChestAngle();
		computeChestAngularVelocityAboutLungeAxis();

		updateIcpDesired();
		updateIcpActual();
	}

	private void updateChestAngle() 
	{
		Quat4d chestToWorld = processedSensors.getChestOrientationInFrame(ReferenceFrame.getWorldFrame()).getQuaternionCopy();
		AxisAngle4d chestToWorldAxisAngle = new AxisAngle4d();
		chestToWorldAxisAngle.set(chestToWorld);

		double bodyAngleToWorld = chestToWorldAxisAngle.getAngle();

		chestAngle.set(bodyAngleToWorld);
	}


	private void updateDynamicsGraphicObjects()
	{
		robotCoMPosition.set(processedSensors.getCenterOfMassPositionInFrame(ReferenceFrame.getWorldFrame()));

		FramePoint chestCoM = chestCoMPosition.getFramePointCopy();
		this.chest.packCoMOffset(chestCoM);
		chestCoM.changeFrame(chestCoMPosition.getReferenceFrame());
		chestCoMPosition.set(chestCoM);

		ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
		if (couplingRegistry.getLungeAxisInFrame(referenceFrame) != null) maxCopYo.set(computeMaxCop((BalancingUpperBodySubControllerState) stateMachine.getCurrentStateEnum()));

	}


	private void setParameters()
	{
		maxChestAngleMagnitude.set(0.5*Math.PI);

		forceControllerIntoState.set(false);
		forcedControllerState.set(BalancingUpperBodySubControllerState.BASE);

		setLungeAxisInWorldFrame(0.0, 0.0);

		predictedCMPscaling.set(0.7); // 0.75

		desiredSpineCmpDirection.put(BalancingUpperBodySubControllerState.BASE, 0.0);
		desiredSpineCmpDirection.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 1.0);
		desiredSpineCmpDirection.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, -1.0);
		desiredSpineCmpDirection.put(BalancingUpperBodySubControllerState.OR_REC_ACC, -1.0);
		desiredSpineCmpDirection.put(BalancingUpperBodySubControllerState.OR_REC_DEC, 1.0);
		
		desiredCopDirection.put(BalancingUpperBodySubControllerState.BASE, 0.0);
		desiredCopDirection.put(BalancingUpperBodySubControllerState.ICP_REC_ACC, 1.0);
		desiredCopDirection.put(BalancingUpperBodySubControllerState.ICP_REC_DEC, 1.0);
		desiredCopDirection.put(BalancingUpperBodySubControllerState.OR_REC_ACC, 1.0);
		desiredCopDirection.put(BalancingUpperBodySubControllerState.OR_REC_DEC, -1.0);


		FramePoint2d icpDesiredInWorld = new FramePoint2d(referenceFrames.getMidFeetZUpFrame());
		icpDesiredInWorld.changeFrame(worldFrame);
      icpDesired.set(icpDesiredInWorld);

	}


	private void setGains()
	{
		neckControllers.get(NeckJointName.LOWER_NECK_PITCH).setProportionalGain(500.0);
		neckControllers.get(NeckJointName.NECK_YAW).setProportionalGain(100.0);
		neckControllers.get(NeckJointName.UPPER_NECK_PITCH).setProportionalGain(300.0);

		neckControllers.get(NeckJointName.LOWER_NECK_PITCH).setDerivativeGain(50.0);
		neckControllers.get(NeckJointName.NECK_YAW).setDerivativeGain(5.0);
		neckControllers.get(NeckJointName.UPPER_NECK_PITCH).setDerivativeGain(30.0);
	}


	private void updateIcpDesired()
	{
		FramePoint2d icpDesiredFromCouplingRegistry = couplingRegistry.getDesiredCapturePointInFrame(ReferenceFrame.getWorldFrame());
		icpDesired.set( icpDesiredFromCouplingRegistry.getX(), icpDesiredFromCouplingRegistry.getY() );
		updateIcpDesiredToIcpActualVector();
	}
	
	private FramePoint2d getIcpDesired()
	{
		return icpDesired.getFramePoint2dCopy();
	}

	private void updateIcpActual()
	{
		FramePoint icpActualFromCouplingRegistry = couplingRegistry.getCapturePointInFrame(ReferenceFrame.getWorldFrame()); 
		icpActual.set( icpActualFromCouplingRegistry.getX(), icpActualFromCouplingRegistry.getY() );
		updateIcpDesiredToIcpActualVector();
	}

	private void updateIcpDesiredToIcpActualVector()
	{
		icpDesiredToicpActualVector.set(icpActual.getFramePoint2dCopy());
		icpDesiredToicpActualVector.sub(icpDesired.getFramePoint2dCopy());
	}


	public abstract class NoTransitionActionsState extends State
	{
		public NoTransitionActionsState(Enum<?> stateEnum)
		{
			super(stateEnum);
		}

		public void doTransitionIntoAction()
		{
		}

		public void doTransitionOutOfAction()
		{
		}
	}

	private FrameVector2d getNewUnitVectorNormalToLungeAxis(ReferenceFrame desiredFrame)
	{
		// Lunge axis and ICP direction are just orthogonal in the x-y plane
		FrameVector2d lungeAxis = couplingRegistry.getLungeAxisInFrame(worldFrame);
		FrameVector2d icpDirection = new FrameVector2d(lungeAxis.getReferenceFrame(), lungeAxis.getY(), -lungeAxis.getX());
		
		icpDirection.changeFrame(desiredFrame);
		icpDirection.normalize();
		return icpDirection;
	}


	private void setCmpInSpineControlModule(BalancingUpperBodySubControllerState currentState, boolean clipTorqueToKeepCmpInsideBoS)
	{

		FrameVector2d deltaCmp = computeDesiredDeltaCmp(currentState, pelvisFrame, clipTorqueToKeepCmpInsideBoS);

		spineControlModule.setDesiredDeltaCmp(deltaCmp);  
	}



	private FrameVector2d computeDesiredDeltaCmp(BalancingUpperBodySubControllerState state, ReferenceFrame desiredFrame, boolean clipTorqueToKeepCmpInsideBoS)
	{
		FrameVector2d deltaCmp = getNewUnitVectorNormalToLungeAxis(desiredFrame);
		deltaCmp.scale(desiredSpineCmpDirection.get(state));
		
		if (clipTorqueToKeepCmpInsideBoS)
		{
			double bosRadius = computeBosRadius(state);
			deltaCmp.normalize();
			deltaCmp.scale(bosRadius);
		}
		else
		{
			deltaCmp.scale(maxCmpDisplacement);
		}
		
		return deltaCmp;
	}
	


	private FrameVector2d computeDesiredDeltaCop(BalancingUpperBodySubControllerState state, ReferenceFrame desiredFrame, boolean clipToSupportPolygon) 
	{
		FrameVector2d deltaCoP = getNewUnitVectorNormalToLungeAxis(desiredFrame);
		
		if (deltaCoP != null)
		{
			deltaCoP.scale(desiredCopDirection.get(state));  //set direction
		}
		
		if (clipToSupportPolygon)
		{
			deltaCoP.normalize();
			double bosRadius = computeBosRadius(state);
			deltaCoP.scale(bosRadius);
		}

		return deltaCoP;
	}

	private FramePoint2d computeDesiredCop(BalancingUpperBodySubControllerState state, ReferenceFrame desiredFrame, boolean clipToSupportPolygon)
	{
		FramePoint2d desiredCoP = new FramePoint2d(getNominalCoP());
		desiredCoP.changeFrame(desiredFrame);
		desiredCoP.add(computeDesiredDeltaCop(state, desiredFrame, clipToSupportPolygon));

		return desiredCoP;
	}
	
	private FramePoint2d computeMaxCop(BalancingUpperBodySubControllerState state)
	{
		FramePoint2d nominalCoP = getNominalCoP();
		FramePoint2d desiredCoP = computeDesiredCop(state, nominalCoP.getReferenceFrame(), false);
		
		FramePoint2d maxCoP = computeIntersectionWithSupportPolygon(nominalCoP, desiredCoP);
	
		return maxCoP;
	}










	private void displayWarningsForBooleans()
	{
		if (forceControllerIntoState.getBooleanValue())
		{
			System.out.println("Warning! Controller " + this.name + " is forced to remain in the " + forcedControllerState.toString() + " state!");
		}
	}


}
