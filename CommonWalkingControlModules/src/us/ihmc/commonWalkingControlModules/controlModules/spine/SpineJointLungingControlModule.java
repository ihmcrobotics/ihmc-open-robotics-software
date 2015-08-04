package us.ihmc.commonWalkingControlModules.controlModules.spine;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineLungingControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.robotics.containers.ContainerTools;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.humanoidRobot.partNames.SpineJointName;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CompositeRigidBodyInertia;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public class SpineJointLungingControlModule implements SpineLungingControlModule
{
	private final YoVariableRegistry registry = new YoVariableRegistry("SpineJointLungingControlModule");

	private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
	private final ReferenceFrame pelvisFrame;
	private final ReferenceFrame chestFrame;
	
	private final double robotMass;
	private final double gravity;

	private final FrameVector forceVectorDueToGravity;
	
	private LinkedHashMap<InverseDynamicsJoint, FrameVector> torqueAboutJointOriginDueToGravity;
	private LinkedHashMap<InverseDynamicsJoint, Double> torqueAboutJointOriginDueToGravityAboutJointAxis;
	private final double maxHipTorque;
	

	private final ArrayList<InverseDynamicsJoint> allIDJointsAbovePelvis;
	private final InverseDynamicsJoint spineRollIDjoint;
	private final InverseDynamicsJoint spineYawIDjoint;
	private final InverseDynamicsJoint spinePitchIDjoint;

	private CompositeRigidBodyInertia upperBodyMoI;
	private double upperBodyMoIProjected;


	private final EnumMap<SpineJointName, DoubleYoVariable> desiredAngles = ContainerTools.createEnumMap(SpineJointName.class);
	private final EnumMap<SpineJointName, PIDController> spinePDControllers = ContainerTools.createEnumMap(SpineJointName.class);
	private final EnumMap<SpineJointName, PIDController> spinePDControllersForInvDynamicsQddTrajectory = ContainerTools.createEnumMap(SpineJointName.class);


	private final YoFrameVector wrenchOnPelvisAngular = new YoFrameVector("wrenchOnPelvisAngular", "", ReferenceFrame.getWorldFrame(), registry);
	private final YoFrameVector wrenchOnPelvisLinear = new YoFrameVector("wrenchOnPelvislinear", "", ReferenceFrame.getWorldFrame(), registry);

	private FrameVector2d desiredDeltaCMP = new FrameVector2d(worldFrame);

	private final ProcessedSensorsInterface processedSensors;
	private final double controlDT;

	private final InverseDynamicsCalculator spineJointIDCalc;
	private final RigidBody pelvis;
	private final RigidBody chest;
	private final EnumMap<SpineJointName, RevoluteJoint> spineRevoluteJointList;

	private SpineTorques spineTorques = new SpineTorques();

	private final CommonHumanoidReferenceFrames referenceFrames;

	public SpineJointLungingControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry,
			InverseDynamicsCalculator spineJointIDCalc, RigidBody chest, EnumMap<SpineJointName, RevoluteJoint> spineRevoluteJointList,
			CommonHumanoidReferenceFrames referenceFrames, double maxHipTorque)
	{
		this.robotMass = processedSensors.getTotalMass();
		this.gravity = processedSensors.getGravityInWorldFrame().getZ();
		this.maxHipTorque = maxHipTorque;
		this.spineJointIDCalc = spineJointIDCalc;
		this.pelvis = processedSensors.getFullRobotModel().getPelvis();
		this.chest = chest;
		this.chestFrame = chest.getBodyFixedFrame();
		this.pelvisFrame = pelvis.getBodyFixedFrame();
		this.spineRevoluteJointList = spineRevoluteJointList;
		this.processedSensors = processedSensors;
		this.controlDT = controlDT;
		this.referenceFrames = referenceFrames;
		parentRegistry.addChild(registry);

		this.spineRollIDjoint =  pelvis.getChildrenJoints().get(0);
		this.spineYawIDjoint = spineRollIDjoint.getSuccessor().getChildrenJoints().get(0);
		this.spinePitchIDjoint = spineYawIDjoint.getSuccessor().getChildrenJoints().get(0);

		checkSpineIDJointNames();

		ArrayList<InverseDynamicsJoint> allJoints = populateAllIDjointsAbovePelvis();
		this.allIDJointsAbovePelvis = allJoints;
		this.upperBodyMoI = new CompositeRigidBodyInertia();
		computeTotalUpperBodyMoI();

		this.forceVectorDueToGravity = new FrameVector(worldFrame, 0.0, 0.0, gravity * upperBodyMoI.getMass());
		
		this.torqueAboutJointOriginDueToGravity = new LinkedHashMap<InverseDynamicsJoint, FrameVector>(3);
		this.torqueAboutJointOriginDueToGravity.put(spineRollIDjoint, new FrameVector(worldFrame));
		this.torqueAboutJointOriginDueToGravity.put(spineYawIDjoint, new FrameVector(worldFrame));
		this.torqueAboutJointOriginDueToGravity.put(spinePitchIDjoint, new FrameVector(worldFrame));
		
		this.torqueAboutJointOriginDueToGravityAboutJointAxis = new LinkedHashMap<InverseDynamicsJoint, Double>(3);

		populateYoVariables();
		populateControllers();
		setDesireds();
		setGains();
	}

	
	public double computeMaxCmpDisplacement()
	{
		boolean computeMaxGravityTorque = true;
		double maxNetTorque = maxHipTorque - computeTorqueDueToGravity(spinePitchIDjoint, computeMaxGravityTorque);  //Using Spine Pitch for now

		double maxCmpDisplacement = maxNetTorque / (robotMass*gravity);
		
		return maxCmpDisplacement;	
	}
	

	// THIS METHOD IS NOT USED
	public void doSpineControl(SpineTorques spineTorquesToPack)
	{
		for (SpineJointName spineJointName : SpineJointName.values)
		{
			PIDController pidController = spinePDControllers.get(spineJointName);

			double desiredPosition = desiredAngles.get(spineJointName).getDoubleValue();
			double desiredVelocity = 0.0;

			double actualPosition = processedSensors.getSpineJointPosition(spineJointName); // actualAngles.get(spineJointName).getDoubleValue();
			double actualVelocity = processedSensors.getSpineJointVelocity(spineJointName); //actualAngleVelocities.get(spineJointName).getDoubleValue();

			double torque = pidController.compute(actualPosition, desiredPosition, actualVelocity, desiredVelocity, controlDT);
			spineTorques.setTorque(spineJointName, torque);
		}
	}

	public void setSpineTorquesForZeroQdd()
	{
		spineTorques.setTorquesToZero();

//		setDesiredAccelerationOnSpineJointsToZero();
		setDesiredAccelerationOnSpineJointsUsingPDcontrol();

		//      ReferenceFrame pelvisFrame = pelvisRigidBody.getBodyFixedFrame();
		//      Wrench wrenchByLowerBody = new Wrench(pelvisFrame,pelvisFrame, new Vector3d(0.0, 0.0, 0.0), new Vector3d());
		//      spineJointIDCalc.setExternalWrench(pelvisRigidBody, wrenchByLowerBody);

		spineJointIDCalc.compute();

		for (SpineJointName spineJointName : SpineJointName.values)
		{
			double torque = spineRevoluteJointList.get(spineJointName).getTau();
			spineTorques.setTorque(spineJointName, torque);
		}
	}


	public void setSpineTorquesForDeltaCmp() 
	{
		spineTorques.setTorquesToZero();
		
		boolean returnActuatorTorqueContributionOnly = true;
		FrameVector2d torqueVector = computeTorqueVectorForDeltaCMP(desiredDeltaCMP, returnActuatorTorqueContributionOnly);
		
		mapTorqueVectorToSpineJointTorques(torqueVector);
	
	}
	

	public void setSpineTorquesForDeltaCmpUsingID() 
	{
		spineTorques.setTorquesToZero();
		setDesiredAccelerationOnSpineJointsToZero();
		
		boolean returnActuatorTorqueContributionOnly = false;
		FrameVector2d torqueVector = computeTorqueVectorForDeltaCMP(desiredDeltaCMP, returnActuatorTorqueContributionOnly);
		
		Wrench virtualWrench = new Wrench(chestFrame, chestFrame);
		virtualWrench.setAngularPartX(-torqueVector.getX());
		virtualWrench.setAngularPartY(-torqueVector.getY());
		
		spineJointIDCalc.setExternalWrench(chest, virtualWrench);
		spineJointIDCalc.compute();
		
		for (SpineJointName spineJointName : SpineJointName.values)
		{
			double torque = spineRevoluteJointList.get(spineJointName).getTau();
			spineTorques.setTorque(spineJointName, torque);
		}
		
	}
	
	public void setSpineTorquesForGravityCancel() 
	{
		spineTorques.setTorquesToZero();
		
		boolean returnActuatorTorqueContributionOnly = true;
		FrameVector2d torqueVector = computeTorqueVectorForDeltaCMP(new FrameVector2d(worldFrame), returnActuatorTorqueContributionOnly);
		
		mapTorqueVectorToSpineJointTorques(torqueVector);
	
	}


	public void setDesiredDeltaCmp(FrameVector2d deltaCmp)
	{
		deltaCmp.changeFrame(desiredDeltaCMP.getReferenceFrame());
		this.desiredDeltaCMP.set(deltaCmp);
	}


	public void getSpineTorques(SpineTorques spineTorquesToPack)
	{
		spineTorquesToPack.setTorques(this.spineTorques.getTorquesCopy());
	}


	public void updateTotalUpperBodyMoI()
	{
		computeTotalUpperBodyMoI();
	}


	public CompositeRigidBodyInertia getTotalUpperBodyMoI() 
	{
		return this.upperBodyMoI;	
	}


	public void computeTotalUpperBodyMoIProjectedAbout(FrameVector2d projectionAxis)
	{
		computeTotalUpperBodyMoI();   //TODO: May not be necessary to re-compute, depending on whether the arms affected the total MoI by changing orientation
		
		projectionAxis.changeFrame(worldFrame);
		
		double projectedMOI;
		if (projectionAxis != null)
		{
			projectionAxis.normalize();
			
			double moiAboutSpinePitch = getTotalUpperBodyCompositeRigidBodyInertiaCopy(spinePitchIDjoint.getFrameAfterJoint()).getMassMomentOfInertiaPartCopy().m11;
			double moiAboutSpineRoll = getTotalUpperBodyCompositeRigidBodyInertiaCopy(spineRollIDjoint.getFrameBeforeJoint()).getMassMomentOfInertiaPartCopy().m00;
	
			double pitchComponentSquared = projectionAxis.getY()*projectionAxis.getY();
			double rollComponentSquared = projectionAxis.getX()*projectionAxis.getX();
			projectedMOI =  moiAboutSpinePitch * pitchComponentSquared +  moiAboutSpineRoll * rollComponentSquared;
		}
		else
		{
			projectedMOI = 0.0;
		}
	
		System.out.println("projected MOI: " + projectedMOI);
	
		upperBodyMoIProjected = projectedMOI;
	}


	public double getTotalUpperBodyMoIProjected() 
	{
		return upperBodyMoIProjected;
	}


	public FrameVector2d computeTorqueVectorForDeltaCMP(FrameVector2d deltaCMP, boolean returnActuatorTorqueContributionOnly)
	{		
		FrameVector2d spineTorqueVectorCMP = new FrameVector2d(deltaCMP);
		spineTorqueVectorCMP.changeFrame(worldFrame);
		spineTorqueVectorCMP.setX(-deltaCMP.getY());
		spineTorqueVectorCMP.setY(deltaCMP.getX());
		spineTorqueVectorCMP.scale( robotMass * gravity );
		
		if (returnActuatorTorqueContributionOnly)
		{
			//Subtract the torque contribution from gravity
			FrameVector2d spineTorqueVectorGravity = new FrameVector2d(worldFrame, computeTorqueDueToGravity(spineRollIDjoint, false), computeTorqueDueToGravity(spinePitchIDjoint, false));
			spineTorqueVectorCMP.sub(spineTorqueVectorGravity);
		}
		
	
		return spineTorqueVectorCMP;
	}


	public void computeTotalWrenchExertedOnPelvis(Wrench totalUpperBodyWrench)
		   {
		      ReferenceFrame expressedInFrame = ReferenceFrame.getWorldFrame();
		      
	//	      totalUpperBodyWrench.setAngularPart(this.desiredWrenchOnPelvis.getAngularPartCopy());   //FIXME: THIS NEEDS TO BE SET
		      
		      double upperBodyTotalMass = getTotalUpperBodyMoI().getMass();
		      FramePoint lungeJointOrigin = new FramePoint(spinePitchIDjoint.getFrameBeforeJoint());
		      FramePoint upperBodyCoM = this.upperBodyMoI.getCenterOfMassOffset();
		      upperBodyCoM.changeFrame(expressedInFrame);
		      
		      FrameVector spinePitchAngularAcceleration = computeAngularAccelerationOfUpperBodyRelativeToPelvis(expressedInFrame);
		      
		      FrameVector comAcceleration = new FrameVector(expressedInFrame);
		      comAcceleration.cross(upperBodyCoM, spinePitchAngularAcceleration);
	
		      comAcceleration.scale(upperBodyTotalMass);
		      comAcceleration.changeFrame(totalUpperBodyWrench.getExpressedInFrame());
		      
		      totalUpperBodyWrench.setLinearPart(comAcceleration.getVectorCopy());
		   }


	private void mapTorqueVectorToSpineJointTorques(FrameVector2d torqueVector) //TODO: THIS IS ONLY VALID FOR ON-AXIS LUNGING
	{
		torqueVector.changeFrame(pelvisFrame);
		
		spineTorques.setTorque(SpineJointName.SPINE_PITCH, torqueVector.getY());
		spineTorques.setTorque(SpineJointName.SPINE_ROLL, torqueVector.getX());
		
	}



	private void setDesiredAccelerationOnSpineJointsUsingPDcontrol()
	{
		for (SpineJointName spineJointName : SpineJointName.values)
		{
			RevoluteJoint spineRevoluteJoint = spineRevoluteJointList.get(spineJointName);

			double actualPosition = processedSensors.getSpineJointPosition(spineJointName);
			double actualVelocity = processedSensors.getSpineJointVelocity(spineJointName);

			double desiredPosition = desiredAngles.get(spineJointName).getDoubleValue();
			double desiredVelocity = 0.0;

			double qddDesired = spinePDControllersForInvDynamicsQddTrajectory.get(spineJointName).compute(actualPosition, desiredPosition, actualVelocity, desiredVelocity, controlDT);
			spineRevoluteJoint.setQddDesired(qddDesired);
		}
	}

	private void setDesiredAccelerationOnSpineJointsToZero()
	{
		for (SpineJointName spineJointName : SpineJointName.values)
		{
			RevoluteJoint spineRevoluteJoint = spineRevoluteJointList.get(spineJointName);
			spineRevoluteJoint.setQddDesired(0.0);
		}
	}



	private void computeSpineTorqueUsingPDControl(SpineTorques spineTorquesToPack)
	{
		for (SpineJointName spineJointName : SpineJointName.values)
		{
			PIDController pidController = spinePDControllers.get(spineJointName);

			double desiredPosition = desiredAngles.get(spineJointName).getDoubleValue();
			double desiredVelocity = 0.0;

			double actualPosition = processedSensors.getSpineJointPosition(spineJointName); // actualAngles.get(spineJointName).getDoubleValue();
			double actualVelocity = processedSensors.getSpineJointVelocity(spineJointName); //actualAngleVelocities.get(spineJointName).getDoubleValue();

			double torqueValue = pidController.compute(actualPosition, desiredPosition, actualVelocity, desiredVelocity, controlDT);
			spineTorquesToPack.setTorque(spineJointName, torqueValue);
		}
	}


	private double computeTorqueDueToGravity(InverseDynamicsJoint jointName, boolean computeMaxTorque)
	{
		double torqueAppliedToJoint = 0.0;
		FrameVector vectorFromJointOriginToUpperBodyCoM = getVectorFromJointOriginToUpperBodyCoM(jointName);

		if (computeMaxTorque)
		{
			torqueAppliedToJoint = vectorFromJointOriginToUpperBodyCoM.length() * forceVectorDueToGravity.length();
		}
		else
		{
			FrameVector torqueVectorDueToGravity = torqueAboutJointOriginDueToGravity.get(jointName);
			torqueVectorDueToGravity.cross(vectorFromJointOriginToUpperBodyCoM, forceVectorDueToGravity);

			torqueAppliedToJoint = torqueVectorDueToGravity.dot(getIDRevoluteJointAxis(jointName, worldFrame));
			torqueAboutJointOriginDueToGravityAboutJointAxis.put(jointName, torqueAppliedToJoint);
		}

		return torqueAppliedToJoint;
	}

	private FrameVector getVectorFromJointOriginToUpperBodyCoM(InverseDynamicsJoint jointName)
	{
		FrameVector vectorFromJointOriginToUpperBodyCoM = new FrameVector(worldFrame);

		FramePoint jointOrigin = new FramePoint(jointName.getFrameBeforeJoint());
		jointOrigin.changeFrame(worldFrame);

		FramePoint upperBodyCoM = upperBodyMoI.getCenterOfMassOffset();
		upperBodyCoM.changeFrame(worldFrame);
		vectorFromJointOriginToUpperBodyCoM.sub(upperBodyCoM, jointOrigin);
		return vectorFromJointOriginToUpperBodyCoM;
	}

	private FrameVector getIDRevoluteJointAxis(InverseDynamicsJoint jointName, ReferenceFrame expressedInFrame)
	{
		Twist twistToPack = new Twist();
		jointName.packJointTwist(twistToPack);


		Vector3d axis = twistToPack.getAngularPartCopy();
		axis.normalize();
		FrameVector jointAxis = new FrameVector(jointName.getFrameAfterJoint(), axis); //shouldn't matter whether using getFrameBeforeJoint() or getFrameAfterJoint()
		jointAxis.changeFrame(expressedInFrame);
		return jointAxis;
	}

	private ArrayList<InverseDynamicsJoint> populateAllIDjointsAbovePelvis()
	{

		ArrayList<InverseDynamicsJoint> allJoints = new ArrayList<InverseDynamicsJoint>();
		List<InverseDynamicsJoint> jointsToIgnore = new ArrayList<InverseDynamicsJoint>();
		ArrayList<RigidBody> morgue = new ArrayList<RigidBody>();

		morgue.add(pelvis);

		InverseDynamicsJoint leftHipPitch = pelvis.getChildrenJoints().get(1);
		InverseDynamicsJoint rightHipPitch = pelvis.getChildrenJoints().get(2);

		jointsToIgnore.add(leftHipPitch);
		jointsToIgnore.add(rightHipPitch);


		while (!morgue.isEmpty())
		{
			RigidBody currentBody = morgue.get(0);

			if (currentBody.hasChildrenJoints())
			{
				List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
				for (InverseDynamicsJoint joint : childrenJoints)
				{
					if (!jointsToIgnore.contains(joint))
					{
						RigidBody successor = joint.getSuccessor();
						if (successor != null)
						{
							allJoints.add(joint);
							morgue.add(successor);
						}
					}
				}
			}

			morgue.remove(currentBody);
		}

		return allJoints;
	}

	private void checkSpineIDJointNames()
	{
		if (!spinePitchIDjoint.getName().equalsIgnoreCase("spinePitch"))
		{
			throw new RuntimeException("Name of Inverse dynamics joint, " + spinePitchIDjoint.getName() + ", does not match expected name.");
		}
		if (!spineRollIDjoint.getName().equalsIgnoreCase("spineRoll"))
		{
			throw new RuntimeException("Name of Inverse dynamics joint, " + spineRollIDjoint.getName() + ", does not match expected name.");
		}
		if (!spineYawIDjoint.getName().equalsIgnoreCase("spineYaw"))
		{
			throw new RuntimeException("Name of Inverse dynamics joint, " + spineYawIDjoint.getName() + ", does not match expected name.");
		}
	}


	private void computeTotalUpperBodyMoI()
	{  
		CompositeRigidBodyInertia totalCompositeRigidBodyInertia = new CompositeRigidBodyInertia(pelvis.getBodyFixedFrame(), new Matrix3d(), 0.0);

		for(InverseDynamicsJoint childJoint : allIDJointsAbovePelvis)
		{
			RigidBody rigidBody = childJoint.getSuccessor();
			CompositeRigidBodyInertia inertia = new CompositeRigidBodyInertia(rigidBody.getBodyFixedFrame(), rigidBody.getInertia().getMassMomentOfInertiaPartCopy(), rigidBody.getInertia().getMass());

			inertia.changeFrame(totalCompositeRigidBodyInertia.getExpressedInFrame());
			totalCompositeRigidBodyInertia.add(inertia);
		}

		setTotalUpperBodyCompositeRigidBodyInertia(totalCompositeRigidBodyInertia);
	}
	
	private void setTotalUpperBodyCompositeRigidBodyInertia(CompositeRigidBodyInertia totalCompositeRigidBodyInertia)
	{
		upperBodyMoI.set(totalCompositeRigidBodyInertia);
	}

	private CompositeRigidBodyInertia getTotalUpperBodyCompositeRigidBodyInertiaCopy(ReferenceFrame expressedInFrame)
	{
		CompositeRigidBodyInertia MoI = new CompositeRigidBodyInertia();
		MoI.set(this.upperBodyMoI);

		MoI.changeFrame(expressedInFrame);

		return MoI;
	}


	private Vector2d computeDeltaCMPDueToSpineTorque (Vector2d spineActuatorTorque, boolean includeGravityTorque)
	{
		Vector2d deltaCMP = new Vector2d();

		if (includeGravityTorque)
		{
			Vector2d totalSpineTorque = new Vector2d(computeTorqueDueToGravity(spineRollIDjoint, false), computeTorqueDueToGravity(spinePitchIDjoint, false));
			totalSpineTorque.add(spineActuatorTorque);
			deltaCMP.set( -totalSpineTorque.getY(), totalSpineTorque.getX());
		}
		else
		{
			deltaCMP.set( -spineActuatorTorque.getY(), spineActuatorTorque.getX());

		}

		deltaCMP.scale( 1 / (robotMass*gravity) );
		return deltaCMP;
	}

	private FrameVector computeAngularAccelerationOfUpperBodyRelativeToPelvis(ReferenceFrame expressedInFrame) 
		{
			//FIXME: This only computes pitch axis angular acceleration
			FrameVector spinePitchAngularAcceleration = new FrameVector(expressedInFrame);
		    SpatialAccelerationVector spinePitchAccelToPack = new SpatialAccelerationVector();
		    spinePitchIDjoint.packJointAcceleration(spinePitchAccelToPack);
		    spinePitchAngularAcceleration.setIncludingFrame(spinePitchAccelToPack.getExpressedInFrame(), spinePitchAccelToPack.getAngularPartCopy());
		    spinePitchAngularAcceleration.changeFrame(expressedInFrame);
			return spinePitchAngularAcceleration;
		}

	   private FrameVector computeAngularAccelerationAcrossSpineJoints()
	   {
//	      spinePitchIDjoint.packJointAcceleration(totalPelvisAccelRelativeToChest);
	      FrameVector totalAngularAccelerationAcrossJoints = new FrameVector(pelvisFrame);
	      
	      SpatialAccelerationVector spinePitchAccelToPack = new SpatialAccelerationVector();
	      spinePitchIDjoint.packJointAcceleration(spinePitchAccelToPack);
	      FrameVector spinePitchAngularAccel = new FrameVector(spinePitchAccelToPack.getExpressedInFrame(), spinePitchAccelToPack.getAngularPartCopy());
	      spinePitchAngularAccel.changeFrame(pelvisFrame);
	      
	      SpatialAccelerationVector spineYawAccelToPack = new SpatialAccelerationVector();
	      spineYawIDjoint.packJointAcceleration(spineYawAccelToPack);
	      FrameVector spineYawAngularAccel = new FrameVector(spineYawAccelToPack.getExpressedInFrame(), spineYawAccelToPack.getAngularPartCopy()); 
	      spineYawAngularAccel.changeFrame(pelvisFrame);
	      
	      SpatialAccelerationVector spineRollAccelToPack = new SpatialAccelerationVector();
	      spineRollIDjoint.packJointAcceleration(spineRollAccelToPack);
	      FrameVector spineRollAngularAccel = new FrameVector(spineRollAccelToPack.getExpressedInFrame(), spineRollAccelToPack.getAngularPartCopy());
	      spineRollAngularAccel.changeFrame(pelvisFrame);
	      
	      totalAngularAccelerationAcrossJoints.add(spinePitchAngularAccel);
	      totalAngularAccelerationAcrossJoints.add(spineYawAngularAccel);
	      totalAngularAccelerationAcrossJoints.add(spineRollAngularAccel);

	      return totalAngularAccelerationAcrossJoints;
	   }


	private void setDesireds()
	{
		desiredAngles.get(SpineJointName.SPINE_PITCH).set(0.0);
		desiredAngles.get(SpineJointName.SPINE_ROLL).set(0.0);
		desiredAngles.get(SpineJointName.SPINE_YAW).set(0.0);
	}


	private void populateControllers()
	{
		for (SpineJointName spineJointName : SpineJointName.values)
		{
			spinePDControllers.put(spineJointName, new PIDController(spineJointName.getCamelCaseNameForStartOfExpression() + "ctrl", registry));
			spinePDControllersForInvDynamicsQddTrajectory.put(spineJointName, new PIDController(spineJointName.getCamelCaseNameForStartOfExpression() + "qddDesired" + "ctrl", registry));            
		}
	}


	private void setGains()
	{
		spinePDControllers.get(SpineJointName.SPINE_YAW).setProportionalGain(3000.0);
		spinePDControllers.get(SpineJointName.SPINE_PITCH).setProportionalGain(3000.0);
		spinePDControllers.get(SpineJointName.SPINE_ROLL).setProportionalGain(3000.0);
	
		spinePDControllers.get(SpineJointName.SPINE_YAW).setDerivativeGain(200.0);
		spinePDControllers.get(SpineJointName.SPINE_PITCH).setDerivativeGain(200.0);
		spinePDControllers.get(SpineJointName.SPINE_ROLL).setDerivativeGain(200.0);
	
	
		spinePDControllersForInvDynamicsQddTrajectory.get(SpineJointName.SPINE_YAW).setProportionalGain(3000.0);
		spinePDControllersForInvDynamicsQddTrajectory.get(SpineJointName.SPINE_PITCH).setProportionalGain(700.0);
		spinePDControllersForInvDynamicsQddTrajectory.get(SpineJointName.SPINE_ROLL).setProportionalGain(10000.0);
	
		spinePDControllersForInvDynamicsQddTrajectory.get(SpineJointName.SPINE_YAW).setDerivativeGain(200.0);
		spinePDControllersForInvDynamicsQddTrajectory.get(SpineJointName.SPINE_PITCH).setDerivativeGain(100.0);
		spinePDControllersForInvDynamicsQddTrajectory.get(SpineJointName.SPINE_ROLL).setDerivativeGain(1000.0);
	
	}


	private void populateYoVariables()
	{
		for (SpineJointName spineJointName : SpineJointName.values)
		{
			String name = "desired" + spineJointName.getCamelCaseNameForMiddleOfExpression();
			DoubleYoVariable variable = new DoubleYoVariable(name, registry);
			desiredAngles.put(spineJointName, variable);
		}
	}



	
	




}