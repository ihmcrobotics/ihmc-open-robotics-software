package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationData;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.simulatedSensors.PointPositionSensorDefinition;
import us.ihmc.sensorProcessing.simulatedSensors.PointVelocitySensorDefinition;
import us.ihmc.sensorProcessing.stateEstimation.DesiredAccelerationAndPointDataProducer;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromControllerSink;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.*;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.*;

public abstract class MomentumBasedController implements RobotController, DesiredAccelerationAndPointDataProducer
{
   private final String name = getClass().getSimpleName();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);

   protected final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected final ReferenceFrame elevatorFrame;
   protected final ReferenceFrame centerOfMassFrame;

   protected final FullRobotModel fullRobotModel;
   protected final CenterOfMassJacobian centerOfMassJacobian;
   protected final CommonWalkingReferenceFrames referenceFrames;
   protected final TwistCalculator twistCalculator;
   protected final List<ContactablePlaneBody> contactablePlaneBodies;

   private final LinkedHashMap<ContactablePlaneBody, DoubleYoVariable> groundReactionForceMagnitudes = new LinkedHashMap<ContactablePlaneBody,
         DoubleYoVariable>();
   protected final LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, YoPlaneContactState>();
   protected final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   protected final DoubleYoVariable yoTime;
   protected final double controlDT;
   protected final double gravity;

   protected final YoFrameVector finalDesiredPelvisLinearAcceleration;
   protected final YoFrameVector finalDesiredPelvisAngularAcceleration;
   protected final YoFrameVector desiredPelvisForce;
   protected final YoFrameVector desiredPelvisTorque;

   protected final YoFrameVector admissibleDesiredGroundReactionTorque;
   protected final YoFrameVector admissibleDesiredGroundReactionForce;
   protected final YoFrameVector groundReactionTorqueCheck;
   protected final YoFrameVector groundReactionForceCheck;

   protected final LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredAccelerationYoVariables = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();

   protected final ProcessedOutputsInterface processedOutputs;
   protected final MomentumRateOfChangeControlModule momentumRateOfChangeControlModule;
   protected final RootJointAccelerationControlModule rootJointAccelerationControlModule;
   protected final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final DesiredCoMAndAngularAccelerationGrabber desiredCoMAndAngularAccelerationGrabber;
   private PointPositionSensorGrabber pointPositionSensorGrabber;
   private PointVelocitySensorGrabber pointVelocitySensorGrabber;
   protected final OldMomentumControlModule momentumControlModule;

   protected final SpatialForceVector gravitationalWrench;
   protected final EnumYoVariable<RobotSide> upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", "", RobotSide.class, registry, true);    // FIXME: not general enough; this should not be here

   public MomentumBasedController(RigidBody estimationLink, ReferenceFrame estimationFrame, FullRobotModel fullRobotModel,
                                  CenterOfMassJacobian centerOfMassJacobian, CommonWalkingReferenceFrames referenceFrames, DoubleYoVariable yoTime,
                                  double gravityZ, TwistCalculator twistCalculator, Collection<? extends ContactablePlaneBody> contactablePlaneBodies,
                                  double controlDT, ProcessedOutputsInterface processedOutputs,
                                  GroundReactionWrenchDistributor groundReactionWrenchDistributor, ArrayList<Updatable> updatables,
                                  MomentumRateOfChangeControlModule momentumRateOfChangeControlModule,
                                  RootJointAccelerationControlModule rootJointAccelerationControlModule, double groundReactionWrenchBreakFrequencyHertz,
                                  DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      LinearSolver<DenseMatrix64F> jacobianSolver = createJacobianSolver();

      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.fullRobotModel = fullRobotModel;
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.referenceFrames = referenceFrames;
      this.twistCalculator = twistCalculator;
      this.contactablePlaneBodies = new ArrayList<ContactablePlaneBody>(contactablePlaneBodies);
      this.controlDT = controlDT;
      this.gravity = gravityZ;
      this.yoTime = yoTime;

      RigidBody elevator = fullRobotModel.getElevator();

      this.processedOutputs = processedOutputs;
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);

      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      this.desiredCoMAndAngularAccelerationGrabber = new DesiredCoMAndAngularAccelerationGrabber(estimationLink, estimationFrame, totalMass);

      gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());


      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      this.finalDesiredPelvisLinearAcceleration = new YoFrameVector("finalDesiredPelvisLinearAcceleration", "", pelvisFrame, registry);
      this.finalDesiredPelvisAngularAcceleration = new YoFrameVector("finalDesiredPelvisAngularAcceleration", "", pelvisFrame, registry);
      this.desiredPelvisForce = new YoFrameVector("desiredPelvisForce", "", centerOfMassFrame, registry);
      this.desiredPelvisTorque = new YoFrameVector("desiredPelvisTorque", "", centerOfMassFrame, registry);


      this.admissibleDesiredGroundReactionTorque = new YoFrameVector("admissibleDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.admissibleDesiredGroundReactionForce = new YoFrameVector("admissibleDesiredGroundReactionForce", centerOfMassFrame, registry);

      this.groundReactionTorqueCheck = new YoFrameVector("groundReactionTorqueCheck", centerOfMassFrame, registry);
      this.groundReactionForceCheck = new YoFrameVector("groundReactionForceCheck", centerOfMassFrame, registry);


      for (ContactablePlaneBody contactableBody : contactablePlaneBodies)
      {
         DoubleYoVariable forceMagnitude = new DoubleYoVariable(contactableBody.getRigidBody().getName() + "ForceMagnitude", registry);
         groundReactionForceMagnitudes.put(contactableBody, forceMagnitude);
      }


      if (updatables != null)
      {
         this.updatables.addAll(updatables);
      }


      elevatorFrame = fullRobotModel.getElevatorFrame();

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(rigidBody.getName(), contactablePlaneBody.getBodyFrame(),
               contactablePlaneBody.getPlaneFrame(), registry);
         double coefficientOfFriction = 1.0;    // TODO: magic number...
         contactState.set(contactablePlaneBody.getContactPoints2d(), coefficientOfFriction);    // initialize with flat 'feet'
         contactStates.put(contactablePlaneBody, contactState);
      }

      InverseDynamicsJoint[] joints = ScrewTools.computeJointsInOrder(elevator);
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof OneDoFJoint)
         {
            desiredAccelerationYoVariables.put((OneDoFJoint) joint, new DoubleYoVariable(joint.getName() + "qdd_d", registry));
         }
      }

      this.momentumRateOfChangeControlModule = momentumRateOfChangeControlModule;
      this.rootJointAccelerationControlModule = rootJointAccelerationControlModule;

      this.momentumControlModule = new OldMomentumControlModule(rootJoint, contactablePlaneBodies, gravityZ, groundReactionWrenchDistributor,
            centerOfMassFrame, controlDT, twistCalculator, jacobianSolver, dynamicGraphicObjectsListRegistry, registry);
      momentumControlModule.setGroundReactionWrenchBreakFrequencyHertz(groundReactionWrenchBreakFrequencyHertz);
   }

   protected static LinearSolver<DenseMatrix64F> createJacobianSolver()
   {
      DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
      jacobianSolver.setAlpha(5e-2);

      return jacobianSolver;
   }

   protected static double computeDesiredAcceleration(double k, double d, double qDesired, double qdDesired, OneDoFJoint joint)
   {
      return k * (qDesired - joint.getQ()) + d * (qdDesired - joint.getQd());
   }

   protected void setExternalHandWrench(RobotSide robotSide, Wrench handWrench)
   {
      inverseDynamicsCalculator.setExternalWrench(fullRobotModel.getHand(robotSide), handWrench);
   }

   public abstract void doMotionControl();

   public final void doControl()
   {
      callUpdatables();

      inverseDynamicsCalculator.reset();
      momentumControlModule.reset();

      doMotionControl();

      rootJointAccelerationControlModule.startComputation();
      rootJointAccelerationControlModule.waitUntilComputationIsDone();
      RootJointAccelerationData rootJointAccelerationData = rootJointAccelerationControlModule.getRootJointAccelerationOutputPort().getData();

      momentumRateOfChangeControlModule.startComputation();
      momentumRateOfChangeControlModule.waitUntilComputationIsDone();
      MomentumRateOfChangeData momentumRateOfChangeData = momentumRateOfChangeControlModule.getMomentumRateOfChangeOutputPort().getData();

      LinkedHashMap<ContactablePlaneBody, ? extends PlaneContactState> contactStates = this.contactStates;


      Map<ContactablePlaneBody, Wrench> externalWrenches = new LinkedHashMap<ContactablePlaneBody, Wrench>();

      SpatialForceVector desiredCentroidalMomentumRate = momentumControlModule.computeDesiredAccelerationsAndExternalWrenches(rootJointAccelerationData,
            momentumRateOfChangeData,
            contactStates, externalWrenches, upcomingSupportLeg.getEnumValue());


      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         PlaneContactState contactState = contactStates.get(contactablePlaneBody);
         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            inverseDynamicsCalculator.setExternalWrench(contactablePlaneBody.getRigidBody(), externalWrenches.get(contactablePlaneBody));
            FrameVector force = externalWrenches.get(contactablePlaneBody).getLinearPartAsFrameVectorCopy(); // TODO: copy
            groundReactionForceMagnitudes.get(contactablePlaneBody).set(force.length());

            // TODO: move filteredCentersOfPressureWorld to here
         } else
         {
            groundReactionForceMagnitudes.get(contactablePlaneBody).set(0.0);
            // TODO: move filteredCentersOfPressureWorld to here
         }
      }

      SpatialForceVector totalGroundReactionWrench = new SpatialForceVector(centerOfMassFrame);
      Wrench admissibleGroundReactionWrench = TotalWrenchCalculator.computeTotalWrench(externalWrenches.values(),
            totalGroundReactionWrench.getExpressedInFrame());
      admissibleDesiredGroundReactionTorque.set(admissibleGroundReactionWrench.getAngularPartCopy());
      admissibleDesiredGroundReactionForce.set(admissibleGroundReactionWrench.getLinearPartCopy());

      SpatialForceVector groundReactionWrenchCheck = inverseDynamicsCalculator.computeTotalExternalWrench(centerOfMassFrame);
      groundReactionTorqueCheck.set(groundReactionWrenchCheck.getAngularPartCopy());
      groundReactionForceCheck.set(groundReactionWrenchCheck.getLinearPartCopy());

      this.desiredCoMAndAngularAccelerationGrabber.set(inverseDynamicsCalculator.getSpatialAccelerationCalculator(), desiredCentroidalMomentumRate);

      updatePositionAndVelocitySensorGrabbers();

      inverseDynamicsCalculator.compute();

      doAdditionalTorqueControl();

      if (processedOutputs != null)
         fullRobotModel.setTorques(processedOutputs);
      updateYoVariables();
   }

   private void updatePositionAndVelocitySensorGrabbers()
   {
      if (pointPositionSensorGrabber != null)
      {
         ArrayList<PointPositionSensorDefinition> pointPositionSensorDefinitions = pointPositionSensorGrabber.getPointPositionSensorDefinitions();

         for (PointPositionSensorDefinition pointPositionSensorDefinition : pointPositionSensorDefinitions)
         {
            // TODO: Record and pass on the estimated position. Determine the offset based on state or whatever...
            // TODO: Determine the covariance based on the state and the pointPositionSensorDefinition. One means trust this, infinity means don't
            double covarianceScaling = Math.random();

            if (covarianceScaling < 0.5)
               covarianceScaling = Double.POSITIVE_INFINITY;
            Point3d positionInWorld = new Point3d();
            Vector3d offsetFromJointInJointFrame = new Vector3d();
            pointPositionSensorDefinition.getOffset(offsetFromJointInJointFrame);

            pointPositionSensorGrabber.setPositionAndCovarianceScaling(pointPositionSensorDefinition, offsetFromJointInJointFrame, positionInWorld,
                  covarianceScaling);
         }
      }

      if (pointVelocitySensorGrabber != null)
      {
         ArrayList<PointVelocitySensorDefinition> pointVelocitySensorDefinitions = pointVelocitySensorGrabber.getPointVelocitySensorDefinitions();

         for (PointVelocitySensorDefinition pointVelocitySensorDefinition : pointVelocitySensorDefinitions)
         {
            // Determine the covariance based on the state and the pointVelocitySensorDefinition.
            double covarianceScaling = Math.random();
            if (covarianceScaling < 0.5)
               covarianceScaling = Double.POSITIVE_INFINITY;

            Vector3d offsetFromJointInJointFrame = new Vector3d();
            pointVelocitySensorDefinition.getOffset(offsetFromJointInJointFrame);

            pointVelocitySensorGrabber.setVelocityToZeroAndCovarianceScaling(pointVelocitySensorDefinition, offsetFromJointInJointFrame, covarianceScaling);
         }
      }

   }

   protected void resetGroundReactionWrenchFilter()
   {
      momentumControlModule.resetGroundReactionWrenchFilter();
   }

   protected abstract void doAdditionalTorqueControl();

   private void callUpdatables()
   {
      double time = yoTime.getDoubleValue();
      for (Updatable updatable : updatables)
      {
         updatable.update(time);
      }
   }

   protected ReferenceFrame getHandFrame(RobotSide robotSide)
   {
      return fullRobotModel.getHand(robotSide).getBodyFixedFrame();
   }

   public void addUpdatable(Updatable updatable)
   {
      updatables.add(updatable);
   }

   protected void doPDControl(OneDoFJoint[] joints, double k, double d)
   {
      for (OneDoFJoint joint : joints)
      {
         doPDControl(joint, k, d, 0.0, 0.0);
      }
   }

   protected void doPDControl(OneDoFJoint joint, double k, double d, double desiredPosition, double desiredVelocity)
   {
      double desiredAcceleration = computeDesiredAcceleration(k, d, desiredPosition, desiredVelocity, joint);
      setOneDoFJointAcceleration(joint, desiredAcceleration);
   }

   protected void setOneDoFJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      DenseMatrix64F jointAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
      jointAcceleration.set(0, 0, desiredAcceleration);
      momentumControlModule.setDesiredJointAcceleration(joint, jointAcceleration);
   }

   private void updateYoVariables()
   {
      SpatialAccelerationVector pelvisAcceleration = new SpatialAccelerationVector();
      fullRobotModel.getRootJoint().packDesiredJointAcceleration(pelvisAcceleration);

      finalDesiredPelvisAngularAcceleration.checkReferenceFrameMatch(pelvisAcceleration.getExpressedInFrame());
      finalDesiredPelvisAngularAcceleration.set(pelvisAcceleration.getAngularPartCopy());

      finalDesiredPelvisLinearAcceleration.checkReferenceFrameMatch(pelvisAcceleration.getExpressedInFrame());
      finalDesiredPelvisLinearAcceleration.set(pelvisAcceleration.getLinearPartCopy());

      Wrench pelvisJointWrench = new Wrench();
      fullRobotModel.getRootJoint().packWrench(pelvisJointWrench);
      pelvisJointWrench.changeFrame(referenceFrames.getCenterOfMassFrame());
      desiredPelvisForce.set(pelvisJointWrench.getLinearPartCopy());
      desiredPelvisTorque.set(pelvisJointWrench.getAngularPartCopy());

      for (OneDoFJoint joint : desiredAccelerationYoVariables.keySet())
      {
         desiredAccelerationYoVariables.get(joint).set(joint.getQddDesired());
      }
   }

   public void initialize()
   {
      inverseDynamicsCalculator.compute();
      momentumControlModule.initialize();
      callUpdatables();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   protected FramePoint2d getCoP(ContactablePlaneBody contactablePlaneBody)
   {
      return momentumControlModule.getCoP(contactablePlaneBody);
   }

   public void attachStateEstimatorDataFromControllerSink(StateEstimationDataFromControllerSink stateEstimatorDataFromControllerSink,
                                                          boolean usePositionDataFromController)
   {
      if (this.pointPositionSensorGrabber != null)
         throw new RuntimeException("Already have set pointPositionSensorDataSource");


      desiredCoMAndAngularAccelerationGrabber.attachStateEstimationDataFromControllerSink(stateEstimatorDataFromControllerSink);

      if (usePositionDataFromController)
      {
         this.pointPositionSensorGrabber = new PointPositionSensorGrabber(stateEstimatorDataFromControllerSink);
         this.pointVelocitySensorGrabber = new PointVelocitySensorGrabber(stateEstimatorDataFromControllerSink);
      }
   }
}
