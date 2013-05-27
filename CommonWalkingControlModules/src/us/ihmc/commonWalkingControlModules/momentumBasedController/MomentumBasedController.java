package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableCylinderBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableRollingBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoCylindricalContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoRollingContactState;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.stateEstimation.DesiredCoMAndAngularAccelerationGrabber;
import us.ihmc.commonWalkingControlModules.stateEstimation.PointPositionGrabber;
import us.ihmc.commonWalkingControlModules.stateEstimation.PointPositionGrabberInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromControllerSink;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TotalWrenchCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class MomentumBasedController implements RobotController
{
   private static final boolean SPY_ON_MOMENTUM_BASED_CONTROLLER = false;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame elevatorFrame;
   private final ReferenceFrame centerOfMassFrame;

   private final FullRobotModel fullRobotModel;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;
   private final SideDependentList<ContactablePlaneBody> feet, handPalms;
   private final SideDependentList<ContactableCylinderBody> graspingHands;
   private final SideDependentList<ContactableRollingBody> rollingThighs;
   private final List<ContactablePlaneBody> listOfAllContactablePlaneBodies;
   private final List<ContactableRollingBody> listOfAllContactableRollingBodies;
   private final List<ContactableCylinderBody> listOfAllContactableCylinderBodies;

   // Creating a Map that will contain all of the YoPlaneContactState and YoRollingContactState to pass to the MomentumControlModule compute method
   private final Map<ContactablePlaneBody, PlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, PlaneContactState>();
   private final LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> planeContactStates = new LinkedHashMap<ContactablePlaneBody, YoPlaneContactState>();
   private final LinkedHashMap<ContactableRollingBody, YoRollingContactState> rollingContactStates = new LinkedHashMap<ContactableRollingBody,
                                                                                                           YoRollingContactState>();
   private final LinkedHashMap<ContactableCylinderBody, YoCylindricalContactState> cylindricalContactStates = new LinkedHashMap<ContactableCylinderBody, YoCylindricalContactState>();
   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();
   private final DoubleYoVariable yoTime;
   private final double controlDT;
   private final double gravity;

   private final YoFrameVector finalDesiredPelvisLinearAcceleration;
   private final YoFrameVector finalDesiredPelvisAngularAcceleration;
   private final YoFrameVector desiredPelvisForce;
   private final YoFrameVector desiredPelvisTorque;

   private final YoFrameVector admissibleDesiredGroundReactionTorque;
   private final YoFrameVector admissibleDesiredGroundReactionForce;
   private final YoFrameVector groundReactionTorqueCheck;
   private final YoFrameVector groundReactionForceCheck;

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredAccelerationYoVariables = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();

   private final ProcessedOutputsInterface processedOutputs;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final DesiredCoMAndAngularAccelerationGrabber desiredCoMAndAngularAccelerationGrabber;
   private final PointPositionGrabberInterface pointPositionGrabber;

   private final MomentumControlModule momentumControlModule;

   private final SpatialForceVector gravitationalWrench;
   private final EnumYoVariable<RobotSide> upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", "", RobotSide.class, registry, true);    // FIXME: not general enough; this should not be here

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final MomentumBasedControllerSpy momentumBasedControllerSpy;

   public MomentumBasedController(RigidBody estimationLink, ReferenceFrame estimationFrame, FullRobotModel fullRobotModel,
                                  CenterOfMassJacobian centerOfMassJacobian, CommonWalkingReferenceFrames referenceFrames, DoubleYoVariable yoTime,
                                  double gravityZ, TwistCalculator twistCalculator, SideDependentList<ContactablePlaneBody> feet,
                                  SideDependentList<ContactablePlaneBody> handPalms, SideDependentList<ContactableCylinderBody> graspingHands,
                                  SideDependentList<ContactableRollingBody> rollingThighs, double controlDT,
                                  ProcessedOutputsInterface processedOutputs, MomentumControlModule momentumControlModule, ArrayList<Updatable> updatables,
                                  StateEstimationDataFromControllerSink stateEstimationDataFromControllerSink,
                                  DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      if (SPY_ON_MOMENTUM_BASED_CONTROLLER) momentumBasedControllerSpy = new MomentumBasedControllerSpy(registry); 
      else momentumBasedControllerSpy = null;
      
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      this.momentumControlModule = momentumControlModule;

      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.fullRobotModel = fullRobotModel;
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.referenceFrames = referenceFrames;
      this.twistCalculator = twistCalculator;
      this.controlDT = controlDT;
      this.gravity = gravityZ;
      this.yoTime = yoTime;
      
      // Initialize the contactable bodies
      this.feet = feet;
      this.handPalms = handPalms; // Plane contact used to bear load with open hands
      this.graspingHands = graspingHands; // Cylindrical contact used to bear load while grasping a cylinder
      this.rollingThighs = rollingThighs;

      RigidBody elevator = fullRobotModel.getElevator();

      this.processedOutputs = processedOutputs;
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);

      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      if (stateEstimationDataFromControllerSink != null)
      {
         this.desiredCoMAndAngularAccelerationGrabber = new DesiredCoMAndAngularAccelerationGrabber(stateEstimationDataFromControllerSink, estimationLink,
                 estimationFrame, totalMass);

         double touchdownTime = 0.12;
         double minCoPDistance = 0.01;

//       this.pointPositionGrabber = new SingleReferenceFramePointPositionGrabber(stateEstimationDataFromControllerSink, registry, controlDT, touchdownTime, minCoPDistance);
         this.pointPositionGrabber = new PointPositionGrabber(stateEstimationDataFromControllerSink, registry, controlDT, touchdownTime, minCoPDistance);
      }
      else
      {
         this.desiredCoMAndAngularAccelerationGrabber = null;
         this.pointPositionGrabber = null;
      }

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


      if (updatables != null)
      {
         this.updatables.addAll(updatables);
      }


      elevatorFrame = fullRobotModel.getElevatorFrame();

      double coefficientOfFriction = 1.0;    // TODO: magic number...

      this.listOfAllContactablePlaneBodies = new ArrayList<ContactablePlaneBody>();

      if (feet != null)
      {
         this.listOfAllContactablePlaneBodies.addAll(feet.values());
      }
      
      if (handPalms != null)
      {
         this.listOfAllContactablePlaneBodies.addAll(handPalms.values());
      }

      for (ContactablePlaneBody contactablePlaneBody : this.listOfAllContactablePlaneBodies)
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(rigidBody.getName(), contactablePlaneBody.getBodyFrame(),
                                               contactablePlaneBody.getPlaneFrame(), registry);
         contactState.set(contactablePlaneBody.getContactPoints2d(), coefficientOfFriction);    // initialize with flat 'feet'
         planeContactStates.put(contactablePlaneBody, contactState);
      }

      InverseDynamicsJoint[] joints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof OneDoFJoint)
         {
            desiredAccelerationYoVariables.put((OneDoFJoint) joint, new DoubleYoVariable(joint.getName() + "qdd_d", registry));
         }
      }

      // Add contactableRollingBodies to the contactablePlaneBodies List, so it is passed to the PlaneContactWrenchProcessor
      this.listOfAllContactableRollingBodies = new ArrayList<ContactableRollingBody>();

      if (rollingThighs != null)
      {
         this.listOfAllContactableRollingBodies.addAll(rollingThighs.values());
         this.listOfAllContactablePlaneBodies.addAll(rollingThighs.values());
      }

      for (ContactableRollingBody contactableRollingBody : this.listOfAllContactableRollingBodies)
      {
         RigidBody rigidBody = contactableRollingBody.getRigidBody();

         // YoRollingContactState: similar to YoPlaneContactState but enables updating the contact points as the contactable cylindrical body rolls onto the ground
         // The contact points can be displayed with the dynamicGraphicObjectsListRegistry
         YoRollingContactState rollingContactState = new YoRollingContactState(rigidBody.getName(), contactableRollingBody, registry);
         rollingContactState.setCoefficientOfFriction(coefficientOfFriction);
         rollingContactStates.put(contactableRollingBody, rollingContactState);
      }

      for (ContactablePlaneBody contactablePlaneBody : planeContactStates.keySet())
      {
         contactStates.put(contactablePlaneBody, planeContactStates.get(contactablePlaneBody));
      }

      for (ContactableRollingBody contactableRollingBody : rollingContactStates.keySet())
      {
         contactStates.put(contactableRollingBody, rollingContactStates.get(contactableRollingBody));
      }
      
      this.listOfAllContactableCylinderBodies = new ArrayList<ContactableCylinderBody>();
      
      if(graspingHands != null)
      {
         this.listOfAllContactableCylinderBodies.addAll(graspingHands.values());
      }
      
      for (ContactableCylinderBody contactableCylinderBody : this.listOfAllContactableCylinderBodies)
      {
         RigidBody rigidBody = contactableCylinderBody.getRigidBody();
         // YoCylindricalContactState: used to enable load bearing with hands while grasping a cylinder
         YoCylindricalContactState cylindricalContactState = new YoCylindricalContactState(rigidBody.getName(), rigidBody.getParentJoint().getFrameAfterJoint(), registry);
         cylindricalContactState.set(coefficientOfFriction, contactableCylinderBody, false);
         cylindricalContactStates.put(contactableCylinderBody, cylindricalContactState);
      }


      this.planeContactWrenchProcessor = new PlaneContactWrenchProcessor(this.listOfAllContactablePlaneBodies, dynamicGraphicObjectsListRegistry, registry);
   }

   public SpatialForceVector getGravitationalWrench()
   {
      return gravitationalWrench;
   }

   private static double computeDesiredAcceleration(double k, double d, double qDesired, double qdDesired, OneDoFJoint joint)
   {
      return k * (qDesired - joint.getQ()) + d * (qdDesired - joint.getQd());
   }

   public void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench)
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setExternalWrenchToCompensateFor(rigidBody, wrench);
      }
      
      momentumControlModule.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   public void doMotionControl()
   {
   }

   // TODO: Temporary method for a big refactor allowing switching between high level behaviors
   public void doPrioritaryControl()
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.doPrioritaryControl();
      }
      
      for (ContactableRollingBody contactableRollingBody : listOfAllContactableRollingBodies)
      {
         // Update the contact points, so they remain underneath the cylindrical body
         rollingContactStates.get(contactableRollingBody).updateContactPoints();
      }

      callUpdatables();

      inverseDynamicsCalculator.reset();
      momentumControlModule.reset();
   }

   // TODO: Temporary method for a big refactor allowing switching between high level behaviors
   public void doSecondaryControl()
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.doSecondaryControl();
      }
      
      momentumControlModule.compute(this.contactStates, this.cylindricalContactStates, upcomingSupportLeg.getEnumValue());

      SpatialForceVector desiredCentroidalMomentumRate = momentumControlModule.getDesiredCentroidalMomentumRate();

      Map<RigidBody, Wrench> externalWrenches = momentumControlModule.getExternalWrenches();


      for (RigidBody rigidBody : externalWrenches.keySet())
      {
         inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrenches.get(rigidBody));
      }

      planeContactWrenchProcessor.compute(externalWrenches);

      SpatialForceVector totalGroundReactionWrench = new SpatialForceVector(centerOfMassFrame);
      Wrench admissibleGroundReactionWrench = TotalWrenchCalculator.computeTotalWrench(externalWrenches.values(),
                                                 totalGroundReactionWrench.getExpressedInFrame());
      admissibleDesiredGroundReactionTorque.set(admissibleGroundReactionWrench.getAngularPartCopy());
      admissibleDesiredGroundReactionForce.set(admissibleGroundReactionWrench.getLinearPartCopy());

      SpatialForceVector groundReactionWrenchCheck = inverseDynamicsCalculator.computeTotalExternalWrench(centerOfMassFrame);
      groundReactionTorqueCheck.set(groundReactionWrenchCheck.getAngularPartCopy());
      groundReactionForceCheck.set(groundReactionWrenchCheck.getLinearPartCopy());

      if (desiredCoMAndAngularAccelerationGrabber != null)
         this.desiredCoMAndAngularAccelerationGrabber.set(inverseDynamicsCalculator.getSpatialAccelerationCalculator(), desiredCentroidalMomentumRate);

      if (pointPositionGrabber != null)
         pointPositionGrabber.set(contactStates, planeContactWrenchProcessor.getCops());

      inverseDynamicsCalculator.compute();

      if (processedOutputs != null)
         fullRobotModel.setTorques(processedOutputs);
      updateYoVariables();
   }

   public final void doControl()
   {
      doPrioritaryControl();
      doMotionControl();
      doSecondaryControl();
   }

   private void resetGroundReactionWrenchFilter()
   {
      momentumControlModule.resetGroundReactionWrenchFilter();
   }

   private void callUpdatables()
   {
      double time = yoTime.getDoubleValue();
      for (Updatable updatable : updatables)
      {
         updatable.update(time);
      }
   }

   // TODO: visibility changed to public
   public ReferenceFrame getHandFrame(RobotSide robotSide)
   {
      return fullRobotModel.getHand(robotSide).getBodyFixedFrame();
   }

   public void addUpdatable(Updatable updatable)
   {
      updatables.add(updatable);
   }

   // TODO: visibility changed to public
   public void doPDControl(OneDoFJoint[] joints, double k, double d)
   {
      for (OneDoFJoint joint : joints)
      {
         doPDControl(joint, k, d, 0.0, 0.0);
      }
   }

   // TODO: visibility changed to public
   public void doPDControl(OneDoFJoint joint, double k, double d, double desiredPosition, double desiredVelocity)
   {
      double desiredAcceleration = computeDesiredAcceleration(k, d, desiredPosition, desiredVelocity, joint);
      setOneDoFJointAcceleration(joint, desiredAcceleration);
   }

   // TODO: visibility changed to public
   public void setOneDoFJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setOneDoFJointAcceleration(joint, desiredAcceleration);
      }
      
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
      planeContactWrenchProcessor.initialize();
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

   // TODO: visibility changed to public
   public FramePoint2d getCoP(ContactablePlaneBody contactablePlaneBody)
   {
      return planeContactWrenchProcessor.getCops().get(contactablePlaneBody);
   }

   // TODO: Following has been added for big refactor. Need to be checked.

   public LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> getContactStates()
   {
      return planeContactStates;
   }

   public LinkedHashMap<ContactableRollingBody, YoRollingContactState> getRollingContactStates()
   {
      return rollingContactStates;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   public void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData)
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
      }
      
      momentumControlModule.setDesiredSpatialAcceleration(jacobian, taskspaceConstraintData);
   }

   public void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
      }
      
      momentumControlModule.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
   }

   public ReferenceFrame getPelvisZUpFrame()
   {
      return referenceFrames.getPelvisZUpFrame();
   }

   public EnumYoVariable<RobotSide> getUpcomingSupportLeg()
   {
      return upcomingSupportLeg;
   }

   public CommonWalkingReferenceFrames getReferenceFrames()
   {
      return referenceFrames;
   }

   public PointPositionGrabberInterface getPointPositionGrabber()
   {
      return pointPositionGrabber;
   }

   public DoubleYoVariable getYoTime()
   {
      return yoTime;
   }

   public double getGravityZ()
   {
      return gravity;
   }

   public double getControlDT()
   {
      return controlDT;
   }

   public FullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

   public TwistCalculator getTwistCalculator()
   {
      return twistCalculator;
   }

   public CenterOfMassJacobian getCenterOfMassJacobian()
   {
      return centerOfMassJacobian;
   }

   public LinkedHashMap<ContactableCylinderBody, YoCylindricalContactState> getCylindricalContactStates()
   {
      return cylindricalContactStates;
   }

   public MomentumControlModule getMomentumControlModule()
   {
      return momentumControlModule;
   }

   public FrameVector getAdmissibleDesiredGroundReactionForceCopy()
   {
      return admissibleDesiredGroundReactionForce.getFrameVectorCopy();
   }

   public FrameVector getAdmissibleDesiredGroundReactionTorqueCopy()
   {
      return admissibleDesiredGroundReactionTorque.getFrameVectorCopy();
   }

   public SideDependentList<ContactablePlaneBody> getContactablePlaneFeet()
   {
      return feet;
   }

   public SideDependentList<ContactablePlaneBody> getContactablePlaneHands()
   {
      return handPalms;
   }

   public ContactablePlaneBody getContactablePlaneHand(RobotSide robotSide)
   {
      return handPalms.get(robotSide);
   }
   
   public SideDependentList<ContactableCylinderBody> getContactableCylinderHands()
   {
      return graspingHands;
   }
   
   public ContactableCylinderBody getContactableCylinderHand(RobotSide robotSide)
   {
      return graspingHands.get(robotSide);
   }
   
   public SideDependentList<ContactableRollingBody> getContactableRollingThighs()
   {
      return rollingThighs;
   }
}
