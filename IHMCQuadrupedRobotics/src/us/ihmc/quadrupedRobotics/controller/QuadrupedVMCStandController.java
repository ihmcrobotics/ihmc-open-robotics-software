package us.ihmc.quadrupedRobotics.controller;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.SdfLoader.OutputWriter;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.PointJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class QuadrupedVMCStandController extends State<QuadrupedControllerState> {
	
	private final double dt;
	private final SDFFullRobotModel sdfFullRobotModel;
	private final QuadrupedRobotParameters robotParameters;
	private final QuadrupedReferenceFrames referenceFrames;
	private final YoGraphicsListRegistry yoGraphicsListRegistry;
	private final DoubleYoVariable robotTimestamp;
	private final YoVariableRegistry registry;

	// Utilities
	private final double mass;
	private final double gravity;
	private final ReferenceFrame worldFrame;
	private final ReferenceFrame bodyFrame;
	private final ReferenceFrame comFrame;
	private CenterOfMassJacobian comJacobian;
	private TwistCalculator twistCalculator;
	private AxisAngleOrientationController bodyOrientationController;
	private PIDController comHeightPIDController;
	private PIDController icpForwardPIDController;
	private PIDController icpLateralPIDController;
	private double startTime;
	
	// Setpoints
	private QuadrantDependentList<YoFrameVector> footForceSetpoint;
	private YoFrameOrientation bodyOrientationSetpoint;
	private YoFrameVector bodyAngularVelocitySetpoint;
	private YoFrameVector bodyTorqueFeedforwardSetpoint;
	private YoFrameVector bodyTorqueSetpoint;
	private DoubleYoVariable comHeightSetpoint;
	private DoubleYoVariable icpOmegaSetpoint;
	private YoFramePoint icpPositionSetpoint;
	private YoFramePoint cmpPositionSetpoint; 
	private YoFrameVector comForceSetpoint;
	
	// Estimates
	private QuadrantDependentList<YoFramePoint> footPositionEstimate;
	private QuadrupedSupportPolygon supportPolygonEstimate;
	private YoFramePoint supportCentroidEstimate;
	private YoFrameOrientation bodyOrientationEstimate;
	private YoFramePoint bodyPositionEstimate;
	private YoFrameVector bodyAngularVelocityEstimate;
	private YoFramePoint comPositionEstimate;
	private YoFrameVector comVelocityEstimate;
	private DoubleYoVariable comHeightEstimate;
	private YoFramePoint icpPositionEstimate;
	
	// Storage
	private final int STORAGE_SIZE = 5;
	private Twist[] twistStorage;
	private FramePose[] framePoseStorage;
	private FramePoint[] framePointStorage;
	private FrameVector[] frameVectorStorage;
	private FrameOrientation[] frameOrientationStorage;

	public QuadrupedVMCStandController(
		double dt,
		QuadrupedRobotParameters robotParameters,
		SDFFullRobotModel sdfFullRobotModel,
		DoubleYoVariable robotTimestamp,
      YoVariableRegistry parentRegistry, 
      YoGraphicsListRegistry yoGraphicsListRegistry)
	{
	   super(QuadrupedControllerState.VMC_STAND);
		this.dt = dt;
		this.robotParameters = robotParameters;
		this.sdfFullRobotModel = sdfFullRobotModel;
		this.robotTimestamp = robotTimestamp;
		this.yoGraphicsListRegistry = yoGraphicsListRegistry;

		// Utilities
	   registry = new YoVariableRegistry(getClass().getSimpleName());
	   referenceFrames = new QuadrupedReferenceFrames(sdfFullRobotModel, robotParameters.getJointMap(), robotParameters.getPhysicalProperties());
		mass = sdfFullRobotModel.getTotalMass();
		gravity = 9.81;
		comFrame = referenceFrames.getCenterOfMassFrame();
		bodyFrame = referenceFrames.getBodyFrame();
		worldFrame = QuadrupedReferenceFrames.getWorldFrame();
	   comJacobian =  new CenterOfMassJacobian(sdfFullRobotModel.getElevator());
		twistCalculator = new TwistCalculator(worldFrame, sdfFullRobotModel.getElevator());
		bodyOrientationController = new AxisAngleOrientationController("bodyOrientation", bodyFrame, dt, registry);
		bodyOrientationController.setProportionalGains(2000, 2000, 2000);
		bodyOrientationController.setIntegralGains(0, 0, 0, 0);
		bodyOrientationController.setDerivativeGains(300, 300, 300);
		comHeightPIDController = new PIDController("bodyHeight", registry);
		comHeightPIDController.setProportionalGain(5000);
		comHeightPIDController.setIntegralGain(100);
		comHeightPIDController.setMaxIntegralError(0);
		comHeightPIDController.setDerivativeGain(1000);
		icpForwardPIDController = new PIDController("icpForward", registry);
		icpForwardPIDController.setProportionalGain(5);
		icpForwardPIDController.setIntegralGain(0);
		icpForwardPIDController.setMaxIntegralError(0);
		icpForwardPIDController.setDerivativeGain(0);
		icpLateralPIDController = new PIDController("icpLateral", registry);
		icpLateralPIDController.setProportionalGain(5);
		icpLateralPIDController.setIntegralGain(0);
		icpLateralPIDController.setMaxIntegralError(0);
		icpLateralPIDController.setDerivativeGain(0);
		startTime = robotTimestamp.getDoubleValue();

		// Setpoints
		footForceSetpoint = new QuadrantDependentList<>();
		for (RobotQuadrant robotQuadrant : RobotQuadrant.values) {
			String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
			footForceSetpoint.put(robotQuadrant, new YoFrameVector(prefix + "FootForceSetpoint", worldFrame, registry));
		}
		bodyOrientationSetpoint = new YoFrameOrientation("bodyOrientationSetpoint", worldFrame, registry);
		bodyAngularVelocitySetpoint = new YoFrameVector("bodyAngularVelocitySetpoint", worldFrame, registry);
		bodyTorqueFeedforwardSetpoint = new YoFrameVector("bodyTorqueFeedforwardSetpoint", worldFrame, registry);
		bodyTorqueSetpoint = new YoFrameVector("bodyTorqueSetpoint", worldFrame, registry);
		comHeightSetpoint = new DoubleYoVariable("comHeightSetpoint", registry);
		icpOmegaSetpoint = new DoubleYoVariable("icpOmegaSetpoint", registry);
		icpPositionSetpoint = new YoFramePoint("icpPositionSetpoint", worldFrame, registry);
		cmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", worldFrame, registry);
		comForceSetpoint = new YoFrameVector("comForceSetpoint", worldFrame, registry);
		
		// Estimates
		footPositionEstimate = new QuadrantDependentList<>();
		for (RobotQuadrant robotQuadrant : RobotQuadrant.values) {
			String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
			footPositionEstimate.put(robotQuadrant, new YoFramePoint(prefix + "FootPositionEstimate", worldFrame, registry));
		}
		supportPolygonEstimate = new QuadrupedSupportPolygon();
		supportCentroidEstimate = new YoFramePoint("supportCentroidEstimate", worldFrame, registry);
		bodyOrientationEstimate = new YoFrameOrientation("bodyOrientationEstimate", worldFrame, registry);
		bodyPositionEstimate = new YoFramePoint("bodyPositionEstimate", worldFrame, registry);
		bodyAngularVelocityEstimate = new YoFrameVector("bodyAngularVelocityEstimate", worldFrame, registry);
		comPositionEstimate = new YoFramePoint("comPositionEstimate", worldFrame, registry);
		comVelocityEstimate = new YoFrameVector("comVelocityEstimate", worldFrame, registry);
		comHeightEstimate = new DoubleYoVariable("comHeightEstimate", registry);
		icpPositionEstimate = new YoFramePoint("icpPositionEstimate", worldFrame, registry);

		// Storage
		twistStorage = new Twist[STORAGE_SIZE];
		framePoseStorage = new FramePose[STORAGE_SIZE];
		framePointStorage = new FramePoint[STORAGE_SIZE];
		frameVectorStorage = new FrameVector[STORAGE_SIZE];
		frameOrientationStorage = new FrameOrientation[STORAGE_SIZE];
		for (int i = 0; i < STORAGE_SIZE; ++i) {
			twistStorage[i] = new Twist();
			framePoseStorage[i] = new FramePose(worldFrame);
			framePointStorage[i] = new FramePoint(worldFrame);
			frameVectorStorage[i] = new FrameVector(worldFrame);
			frameOrientationStorage[i] = new FrameOrientation(worldFrame);
		}
      parentRegistry.addChild(registry);
	}

	private void updateEstimates() {
		// compute forward kinematics
		referenceFrames.updateFrames();
		twistCalculator.compute();
		
		// compute foot positions
		for (RobotQuadrant robotQuadrant : RobotQuadrant.values) {
			ReferenceFrame footFrame = referenceFrames.getFootFrame(robotQuadrant);
			framePointStorage[0].setToZero(footFrame);
			framePointStorage[0].changeFrame(worldFrame);
			footPositionEstimate.get(robotQuadrant).setAndMatchFrame(framePointStorage[0]);
		}
		
		// compute support polygon and centroid
		for (RobotQuadrant robotQuadrant : RobotQuadrant.values) {
			supportPolygonEstimate.setFootstep(robotQuadrant, footPositionEstimate.get(robotQuadrant).getFrameTuple());
		}
		supportPolygonEstimate.getCentroid(framePointStorage[0]);
		supportCentroidEstimate.setAndMatchFrame(framePointStorage[0]);

		// compute body orientation
		frameOrientationStorage[0].setToZero(bodyFrame);
		frameOrientationStorage[0].changeFrame(worldFrame);
		bodyOrientationEstimate.setAndMatchFrame(frameOrientationStorage[0]);
		
		// compute body position
		framePointStorage[0].setToZero(bodyFrame);
		framePointStorage[0].changeFrame(worldFrame);
		bodyPositionEstimate.setAndMatchFrame(framePointStorage[0]);
		
		// compute body angular velocity
		twistCalculator.packTwistOfBody(twistStorage[0], sdfFullRobotModel.getPelvis());
		twistStorage[0].packAngularPart(frameVectorStorage[0]);
		bodyAngularVelocityEstimate.setAndMatchFrame(frameVectorStorage[0]);
		
		// compute center of mass position
		framePointStorage[0].setToZero(comFrame);
		framePointStorage[0].changeFrame(worldFrame);
		comPositionEstimate.setAndMatchFrame(framePointStorage[0]);

		// compute center of mass velocity
		comJacobian.compute();
		comJacobian.packCenterOfMassVelocity(frameVectorStorage[0]);
		frameVectorStorage[0].changeFrame(worldFrame);
		comVelocityEstimate.setAndMatchFrame(frameVectorStorage[0]);

		// compute center of mass height
		comHeightEstimate.set(comPositionEstimate.getZ() - supportCentroidEstimate.getZ());
		
		// compute instantaneous capture point
		double omega = icpOmegaSetpoint.getDoubleValue();
		icpPositionEstimate.setX(comPositionEstimate.getX() + comVelocityEstimate.getX()/omega);
		icpPositionEstimate.setY(comPositionEstimate.getY() + comVelocityEstimate.getY()/omega);
		icpPositionEstimate.setZ(supportCentroidEstimate.getZ());
	}
	
	private void updateSetpoints() {
		// compute capture point natural frequency
		icpOmegaSetpoint.set(Math.sqrt(gravity/comHeightSetpoint.getDoubleValue()));
		
		// compute body torque setpoints to track desired body orientation
		bodyOrientationSetpoint.getFrameOrientationIncludingFrame(frameOrientationStorage[0]);
		bodyAngularVelocitySetpoint.getFrameTupleIncludingFrame(frameVectorStorage[1]);
		bodyAngularVelocityEstimate.getFrameTupleIncludingFrame(frameVectorStorage[2]);
		bodyTorqueFeedforwardSetpoint.getFrameTupleIncludingFrame(frameVectorStorage[3]);
		frameOrientationStorage[0].changeFrame(bodyFrame);
		frameVectorStorage[0].changeFrame(bodyFrame);
		frameVectorStorage[1].changeFrame(bodyFrame);
		frameVectorStorage[2].changeFrame(bodyFrame);
		frameVectorStorage[3].changeFrame(bodyFrame);
		bodyOrientationController.compute(
			frameVectorStorage[0],
			frameOrientationStorage[0],
			frameVectorStorage[1],
			frameVectorStorage[2],
			frameVectorStorage[3]
		);
		frameVectorStorage[0].changeFrame(worldFrame);
		bodyTorqueSetpoint.setAndMatchFrame(frameVectorStorage[0]);
		
		// compute body force setpoints to track desired capture point and center of mass height
		double omega = icpOmegaSetpoint.getDoubleValue();
		double cmpX = icpPositionEstimate.getX() - 1/omega*icpForwardPIDController.compute(
			icpPositionEstimate.getX(),
			icpPositionSetpoint.getX(),
			0,
			0,
			dt
		);
		double cmpY = icpPositionEstimate.getY() - 1/omega*icpLateralPIDController.compute(
			icpPositionEstimate.getY(),
			icpPositionSetpoint.getY(),
			0,
			0,
			dt
		);
		double cmpZ = icpPositionSetpoint.getZ();
		double fX = mass*Math.pow(omega, 2)*(comPositionEstimate.getX() - cmpX);
		double fY = mass*Math.pow(omega, 2)*(comPositionEstimate.getY() - cmpY);
		double fZ = 0.75*mass*gravity + comHeightPIDController.compute(
			comHeightEstimate.getDoubleValue(),
			comHeightSetpoint.getDoubleValue(),
			comVelocityEstimate.getZ(),
			0,
			dt
		);
		cmpPositionSetpoint.set(cmpX, cmpY, cmpZ);
		comForceSetpoint.set(fX, fY, fZ);
		
		// compute ground reaction forces using least squares optimization
		distributeForcesFourLegs(
			comForceSetpoint.getX(),
			comForceSetpoint.getY(),
			comForceSetpoint.getZ(),
			bodyTorqueSetpoint.getX(),
			bodyTorqueSetpoint.getY(),
			bodyTorqueSetpoint.getZ()
		);
		
		// compute joint torques using virtual model control
		computeLegTorques();
	}
	
	public void distributeForcesFourLegs(double fX, double fY, double fZ, double nX, double nY, double nZ) {
		
		// compute constraint matrix (mapping from foot forces to body wrench)
		DenseMatrix64F forceConstraintsMatrix = new DenseMatrix64F(6, 12);
		forceConstraintsMatrix.zero();
		int columnOffset = 0;
		for (RobotQuadrant robotQuadrant : RobotQuadrant.values) {
			// compute foot offset relative to body in world coordinates
			FramePoint footOffset = framePointStorage[0];
			footOffset.setIncludingFrame(footPositionEstimate.get(robotQuadrant).getFrameTuple());
			footOffset.sub(bodyPositionEstimate.getFrameTuple());
			
			// populate constraint matrix coefficients
			forceConstraintsMatrix.set(0, 0 + columnOffset, 1.0); // fX row
			forceConstraintsMatrix.set(1, 1 + columnOffset, 1.0); // fY row
			forceConstraintsMatrix.set(2, 2 + columnOffset, 1.0); // fZ row
			forceConstraintsMatrix.set(3, 1 + columnOffset,-footOffset.getZ()); // nX row
			forceConstraintsMatrix.set(3, 2 + columnOffset, footOffset.getY());
			forceConstraintsMatrix.set(4, 0 + columnOffset, footOffset.getZ()); // nY row
			forceConstraintsMatrix.set(4, 2 + columnOffset,-footOffset.getX());
			forceConstraintsMatrix.set(5, 0 + columnOffset,-footOffset.getY()); // nZ row
			forceConstraintsMatrix.set(5, 1 + columnOffset, footOffset.getX());
			columnOffset += 3;
		}

		// compute constraint vector (desired body wrench)
		DenseMatrix64F totalForcesOnTheBody = new DenseMatrix64F(6, 1);
		totalForcesOnTheBody.set(0, 0, fX);
		totalForcesOnTheBody.set(1, 0, fY);
		totalForcesOnTheBody.set(2, 0, fZ);
		totalForcesOnTheBody.set(3, 0, nX);
		totalForcesOnTheBody.set(4, 0, nY);
		totalForcesOnTheBody.set(5, 0, nZ);

		// solve for optimal foot forces in world coordinates using least squares
		DenseMatrix64F forceConstraintsMatrixInverse = new DenseMatrix64F(12, 6);
		try {
			CommonOps.pinv(forceConstraintsMatrix, forceConstraintsMatrixInverse);
		}
		catch (Exception e) {
			System.out.println("forceConstraintsMatrix = " + forceConstraintsMatrix);
			return;
		}
		DenseMatrix64F distributedForcesOnTheLegs = new DenseMatrix64F(12, 1);
		CommonOps.mult(forceConstraintsMatrixInverse, totalForcesOnTheBody, distributedForcesOnTheLegs);
		int rowOffset = 0;
		for (RobotQuadrant robotQuadrant : RobotQuadrant.values) {
			footForceSetpoint.get(robotQuadrant).setX(distributedForcesOnTheLegs.get(0 + rowOffset, 0));
			footForceSetpoint.get(robotQuadrant).setY(distributedForcesOnTheLegs.get(1 + rowOffset, 0));
			footForceSetpoint.get(robotQuadrant).setZ(distributedForcesOnTheLegs.get(2 + rowOffset, 0));
			rowOffset += 3;
		}

//	    System.out.println("forceConstraintsMatrix = " + forceConstraintsMatrix);
//	    System.out.println("totalForcesOnTheBody = " + totalForcesOnTheBody);
//	    System.out.println("distributedForcesOnTheLegs = " + distributedForcesOnTheLegs);
	}
	
	private void computeLegTorques() {
		for (RobotQuadrant robotQuadrant : RobotQuadrant.values) {
			// compute foot force in body coordinates
		    FrameVector footForce = frameVectorStorage[0];
		    footForceSetpoint.get(robotQuadrant).getFrameTupleIncludingFrame(footForce);
		    footForce.changeFrame(bodyFrame);
		    
		    // compute foot position in body coordinates
		    FramePoint footPosition = framePointStorage[0];
		    footPositionEstimate.get(robotQuadrant).getFrameTupleIncludingFrame(footPosition);
		    footPosition.changeFrame(bodyFrame);
		    
		    // get joint array for the leg chain
		    OneDoFJoint kneeJoint = sdfFullRobotModel.getOneDoFJointByName(robotParameters.getJointMap().getJointBeforeFootName(robotQuadrant));
			RigidBody body = sdfFullRobotModel.getRootJoint().getSuccessor();
			RigidBody shin = kneeJoint.getSuccessor();
		    OneDoFJoint[] legJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(body, shin), OneDoFJoint.class);
		    
		    // compute foot Jacobian in body coordinates
		    GeometricJacobian shinJacobian = new GeometricJacobian(legJoints, bodyFrame);
		    shinJacobian.compute();
		    PointJacobian footJacobian = new PointJacobian();
		    footJacobian.set(shinJacobian, footPosition);
		    footJacobian.compute();

		    // solve for leg torques using Jacobian transpose
		    DenseMatrix64F jacobian = footJacobian.getJacobianMatrix();
		    DenseMatrix64F forces = new DenseMatrix64F(3, 1);
		    forces.set(0, 0, -footForce.getX());
		    forces.set(1, 0, -footForce.getY());
		    forces.set(2, 0, -footForce.getZ());
		    DenseMatrix64F torques = new DenseMatrix64F(1, jacobian.getNumCols());
		    CommonOps.multTransA(forces, jacobian, torques);
		    CommonOps.transpose(torques);
		    
		    // update joint torque setpoints in full robot models
		    int row = 0;
			for (OneDoFJoint joint : legJoints) {
				joint.setTau(torques.get(row++, 0));
			}
			
			// hack to get knees to bend without stand_prep state
			if (robotTimestamp.getDoubleValue() - startTime < 0.1) {
				for (OneDoFJoint joint : legJoints) {
					joint.setTau(0);
				}
				kneeJoint.setTau(robotQuadrant.getEnd().negateIfFrontEnd(5));		
			}
		    
//		    System.out.println("forces = " + forces);
//		    System.out.println("jacobian = " + jacobian);
//		    System.out.println("torques = " + torques);
		}
		

	}

   @Override
   public void doAction()
   {
      // TODO Auto-generated method stub
      updateEstimates();
      updateSetpoints();
   }

   @Override
   public void doTransitionIntoAction()
   {
      // initialize estimates
      updateEstimates();
      
      // initialize setpoints
      bodyOrientationSetpoint.set(bodyOrientationEstimate);
      bodyAngularVelocitySetpoint.setToZero();
      bodyTorqueFeedforwardSetpoint.setToZero();
      bodyTorqueSetpoint.setToZero();
//    comHeightSetpoint.set(comPositionEstimate.getZ() - supportCentroidEstimate.getZ());
      comHeightSetpoint.set(0.55);
      icpOmegaSetpoint.set(Math.sqrt(gravity/comHeightSetpoint.getDoubleValue()));
      icpPositionSetpoint.setX(supportCentroidEstimate.getX());
      icpPositionSetpoint.setY(supportCentroidEstimate.getY());
      icpPositionSetpoint.setZ(supportCentroidEstimate.getZ());
      cmpPositionSetpoint.set(icpPositionSetpoint);
      comForceSetpoint.setToZero();

      for(OneDoFJoint oneDofJoint : sdfFullRobotModel.getOneDoFJoints())
      {
         oneDofJoint.setUnderPositionControl(false);
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub
      
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
