package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactPointVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableCylinderBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ModifiableContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoCylindricalContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumControlModuleBridge.MomentumControlModuleType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumControlModuleException;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OptimizationMomentumControlModule;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.stateEstimation.DesiredCoMAndAngularAccelerationGrabber;
import us.ihmc.commonWalkingControlModules.stateEstimation.PointPositionGrabber;
import us.ihmc.commonWalkingControlModules.stateEstimation.PointPositionGrabberInterface;
import us.ihmc.commonWalkingControlModules.visualizer.WrenchVisualizer;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimationDataFromController;
import us.ihmc.utilities.exeptions.NoConvergenceException;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
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

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.filter.RateLimitedYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class MomentumBasedController
{
   public static final boolean SPY_ON_MOMENTUM_BASED_CONTROLLER = false;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final ReferenceFrame centerOfMassFrame;

   private final FullRobotModel fullRobotModel;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;

   private final SideDependentList<ContactablePlaneBody> feet, handsWithFingersBentBack, thighs;
   private final ContactablePlaneBody pelvis, pelvisBack;

   private final SideDependentList<ContactableCylinderBody> graspingHands;
   private final List<ContactablePlaneBody> listOfAllContactablePlaneBodies;
   private final List<ContactableCylinderBody> listOfAllContactableCylinderBodies;
   
   private final DoubleYoVariable leftPassiveKneeTorque = new DoubleYoVariable("leftPassiveKneeTorque", registry);
   private final DoubleYoVariable rightPassiveKneeTorque = new DoubleYoVariable("rightPassiveKneeTorque", registry);
   private final SideDependentList<DoubleYoVariable> passiveKneeTorque = new SideDependentList<DoubleYoVariable>(leftPassiveKneeTorque, rightPassiveKneeTorque);
   
   private final DoubleYoVariable passiveQKneeThreshold = new DoubleYoVariable("passiveQKneeThreshold", registry);
   private final DoubleYoVariable passiveKneeMaxTorque = new DoubleYoVariable("passiveKneeMaxTorque", registry);
   private final DoubleYoVariable passiveKneeKv = new DoubleYoVariable("passiveKneeKv", registry);

   private final LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> yoPlaneContactStates = new LinkedHashMap<ContactablePlaneBody, YoPlaneContactState>();
   private final LinkedHashMap<ContactableCylinderBody, YoCylindricalContactState> yoCylindricalContactStates = new LinkedHashMap<ContactableCylinderBody, YoCylindricalContactState>();
   private final List<ModifiableContactState> modifiableContactStates = new ArrayList<ModifiableContactState>();

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

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> preRateLimitedDesiredAccelerations = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
   private final LinkedHashMap<OneDoFJoint, RateLimitedYoVariable> rateLimitedDesiredAccelerations = new LinkedHashMap<OneDoFJoint, RateLimitedYoVariable>();
   
   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> desiredAccelerationYoVariables = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();

   private final ProcessedOutputsInterface processedOutputs;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final DesiredCoMAndAngularAccelerationGrabber desiredCoMAndAngularAccelerationGrabber;
   private final BooleanYoVariable resetEstimatorPositionsToCurrent = new BooleanYoVariable("resetEstimatorPositionsToCurrent", registry);
   private final PointPositionGrabberInterface pointPositionGrabber;

   private final MomentumControlModuleBridge momentumControlModuleBridge;

   private final SpatialForceVector gravitationalWrench;
   private final EnumYoVariable<RobotSide> upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", "", RobotSide.class, registry, true); // FIXME: not general enough; this should not be here

   private final PlaneContactWrenchProcessor planeContactWrenchProcessor;
   private final MomentumBasedControllerSpy momentumBasedControllerSpy;
   private final ContactPointVisualizer contactPointVisualizer;
   private final WrenchVisualizer wrenchVisualizer;

   public MomentumBasedController(RigidBody estimationLink, ReferenceFrame estimationFrame, FullRobotModel fullRobotModel,
         CenterOfMassJacobian centerOfMassJacobian, CommonWalkingReferenceFrames referenceFrames, DoubleYoVariable yoTime, double gravityZ,
         TwistCalculator twistCalculator, SideDependentList<ContactablePlaneBody> feet, SideDependentList<ContactablePlaneBody> handsWithFingersBentBack,
         SideDependentList<ContactableCylinderBody> graspingHands, SideDependentList<ContactablePlaneBody> thighs, ContactablePlaneBody pelvis,
         ContactablePlaneBody pelvisBack, double controlDT, ProcessedOutputsInterface processedOutputs,
         OptimizationMomentumControlModule optimizationMomentumControlModule, OldMomentumControlModule oldMomentumControlModule,
         ArrayList<Updatable> updatables, StateEstimationDataFromController stateEstimationDataFromControllerSink,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      momentumControlModuleBridge = new MomentumControlModuleBridge(optimizationMomentumControlModule, oldMomentumControlModule, centerOfMassFrame, registry);

      if (SPY_ON_MOMENTUM_BASED_CONTROLLER)
         momentumBasedControllerSpy = new MomentumBasedControllerSpy(registry);
      else
         momentumBasedControllerSpy = null;

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
      this.handsWithFingersBentBack = handsWithFingersBentBack;
      this.graspingHands = graspingHands; // Cylindrical contact used to bear load while grasping a cylinder
      this.thighs = thighs;
      this.pelvis = pelvis;
      this.pelvisBack = pelvisBack;

      RigidBody elevator = fullRobotModel.getElevator();

      this.processedOutputs = processedOutputs;
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);

      double totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

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

      double coefficientOfFriction = 1.0; // TODO: magic number...

      // TODO: get rid of the null checks
      this.listOfAllContactablePlaneBodies = new ArrayList<ContactablePlaneBody>();

      if (feet != null)
      {
         this.listOfAllContactablePlaneBodies.addAll(feet.values()); //leftSole and rightSole
      }

      if (handsWithFingersBentBack != null)
      {
         this.listOfAllContactablePlaneBodies.addAll(handsWithFingersBentBack.values());
      }

      if (thighs != null)
      {
         this.listOfAllContactablePlaneBodies.addAll(thighs.values());
      }

      if (pelvis != null)
         this.listOfAllContactablePlaneBodies.add(pelvis);

      if (pelvisBack != null)
         this.listOfAllContactablePlaneBodies.add(pelvisBack);

      for (ContactablePlaneBody contactablePlaneBody : this.listOfAllContactablePlaneBodies)
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(contactablePlaneBody.getPlaneFrame().getName(), rigidBody,
               contactablePlaneBody.getPlaneFrame(), contactablePlaneBody.getContactPoints2d(), coefficientOfFriction, registry);
         yoPlaneContactStates.put(contactablePlaneBody, contactState);
      }

      if (stateEstimationDataFromControllerSink != null)
      {
         this.desiredCoMAndAngularAccelerationGrabber = new DesiredCoMAndAngularAccelerationGrabber(stateEstimationDataFromControllerSink, estimationLink,
               estimationFrame, totalMass, registry);

         double touchdownTime = 0.12;
         double minCoPDistance = 0.01;

         //       this.pointPositionGrabber = new SingleReferenceFramePointPositionGrabber(stateEstimationDataFromController, registry, controlDT, touchdownTime, minCoPDistance);
         this.pointPositionGrabber = new PointPositionGrabber(stateEstimationDataFromControllerSink, yoPlaneContactStates, registry, controlDT, touchdownTime,
               minCoPDistance);
         setDelayTimeBeforeTrustingContacts(touchdownTime);
      }
      else
      {
         this.desiredCoMAndAngularAccelerationGrabber = null;
         this.pointPositionGrabber = null;
      }

      InverseDynamicsJoint[] joints = ScrewTools.computeSupportAndSubtreeJoints(fullRobotModel.getRootJoint().getSuccessor());
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof OneDoFJoint)
         {
            desiredAccelerationYoVariables.put((OneDoFJoint) joint, new DoubleYoVariable(joint.getName() + "qdd_d", registry));
            rateLimitedDesiredAccelerations.put((OneDoFJoint) joint, new RateLimitedYoVariable(joint.getName() + "_rl_qdd_d", registry, 10000.0, controlDT));
            preRateLimitedDesiredAccelerations.put((OneDoFJoint) joint, new DoubleYoVariable(joint.getName() + "_prl_qdd_d", registry));
         }
      }

      this.listOfAllContactableCylinderBodies = new ArrayList<ContactableCylinderBody>();

      if (graspingHands != null)
      {
         this.listOfAllContactableCylinderBodies.addAll(graspingHands.values());
      }

      //    coefficientOfFriction = 0.0;
      for (ContactableCylinderBody contactableCylinderBody : this.listOfAllContactableCylinderBodies)
      {
         RigidBody rigidBody = contactableCylinderBody.getRigidBody();

         // YoCylindricalContactState: used to enable load bearing with hands by grasping a cylinder
         YoCylindricalContactState cylindricalContactState = new YoCylindricalContactState(rigidBody.getName(),
               rigidBody.getParentJoint().getFrameAfterJoint(), contactableCylinderBody.getCylinderFrame(), registry, dynamicGraphicObjectsListRegistry);
         cylindricalContactState.set(coefficientOfFriction, contactableCylinderBody, false);
         yoCylindricalContactStates.put(contactableCylinderBody, cylindricalContactState);
      }

      modifiableContactStates.addAll(yoPlaneContactStates.values());
      modifiableContactStates.addAll(yoCylindricalContactStates.values());

      this.planeContactWrenchProcessor = new PlaneContactWrenchProcessor(this.listOfAllContactablePlaneBodies, dynamicGraphicObjectsListRegistry, registry);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         contactPointVisualizer = new ContactPointVisualizer(this.yoPlaneContactStates.values(), dynamicGraphicObjectsListRegistry, registry);
         List<RigidBody> rigidBodies = Arrays.asList(ScrewTools.computeSupportAndSubtreeSuccessors(fullRobotModel.getRootJoint().getSuccessor()));
         wrenchVisualizer = new WrenchVisualizer("DesiredExternalWrench", rigidBodies, dynamicGraphicObjectsListRegistry, registry);
      }
      else
      {
         contactPointVisualizer = null;
         wrenchVisualizer = null;
      }
      
      
      passiveQKneeThreshold.set(0.3);
      passiveKneeMaxTorque.set(25.0);
      passiveKneeKv.set(5.0);      
   }

   public SideDependentList<ContactablePlaneBody> getFeet()
   {
      return feet;
   }

   public void getFeetContactStates(ArrayList<PlaneContactState> feetContactStatesToPack)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         feetContactStatesToPack.add(yoPlaneContactStates.get(feet.get(robotSide)));
      }
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

      //      momentumControlModuleBridge.getActiveMomentumControlModule().setExternalWrenchToCompensateFor(rigidBody, wrench);
      momentumControlModuleBridge.setExternalWrenchToCompensateFor(rigidBody, wrench);
   }

   // TODO: Temporary method for a big refactor allowing switching between high level behaviors
   public void doPrioritaryControl()
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.doPrioritaryControl();
      }

      callUpdatables();

      inverseDynamicsCalculator.reset();
      momentumControlModuleBridge.reset();

      resetWeightsForContactRegularization();
   }

   //TODO  (Sylvain): get rid of that
   private void resetWeightsForContactRegularization()
   {
      // TODO: get rid of contactStates or planeContactStates. This is confusing.
      for (PlaneContactState contactState : yoPlaneContactStates.values())
      {
         contactState.resetContactRegularization();
      }

      for (PlaneContactState contactState : yoPlaneContactStates.values())
      {
         contactState.resetContactRegularization();
      }

      for (CylindricalContactState contactState : yoCylindricalContactStates.values())
      {
         contactState.resetContactRegularization();
      }
   }

   // TODO: Temporary method for a big refactor allowing switching between high level behaviors
   public void doSecondaryControl()
   {
      if (contactPointVisualizer != null)
         contactPointVisualizer.update();

      updateMomentumBasedControllerSpy();

      MomentumModuleSolution momentumModuleSolution;
      try
      {
         momentumModuleSolution = momentumControlModuleBridge.compute(this.yoPlaneContactStates, this.yoCylindricalContactStates,
               upcomingSupportLeg.getEnumValue());
      }
      catch (MomentumControlModuleException momentumControlModuleException)
      {
         if (momentumBasedControllerSpy != null)
            momentumBasedControllerSpy.printMomentumCommands(System.err);
         
         // Don't crash and burn. Instead do the best you can with what you have.
         // Or maybe just use the previous ticks solution.
         // Need to test these.
         momentumModuleSolution = momentumControlModuleException.getMomentumModuleSolution();
         //throw new RuntimeException(momentumControlModuleException);
      }

      SpatialForceVector desiredCentroidalMomentumRate = momentumModuleSolution.getCentroidalMomentumRateSolution();
      Map<RigidBody, Wrench> externalWrenches = momentumModuleSolution.getExternalWrenchSolution();

      for (RigidBody rigidBody : externalWrenches.keySet())
      {
         inverseDynamicsCalculator.setExternalWrench(rigidBody, externalWrenches.get(rigidBody));
      }

      planeContactWrenchProcessor.compute(externalWrenches); 
      if (wrenchVisualizer != null)
         wrenchVisualizer.visualize(externalWrenches);

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
      {
         if (resetEstimatorPositionsToCurrent.getBooleanValue())
         {
            pointPositionGrabber.resetToCurrentLocations(yoPlaneContactStates, planeContactWrenchProcessor.getCops());
            resetEstimatorPositionsToCurrent.set(false);
         }
         pointPositionGrabber.set(yoPlaneContactStates, planeContactWrenchProcessor.getCops());
      }

      inverseDynamicsCalculator.compute();

      if (processedOutputs != null)
         fullRobotModel.setTorques(processedOutputs);
      updateYoVariables();
   }

   /**
    * Call this method after doSecondaryControl() to generate a small torque of flexion at the knees when almost straight.
    * This helps a lot when working near singularities but it is kinda hackish.
    */
   public void doPassiveKneeControl()
   {
      double maxPassiveTorque = passiveKneeMaxTorque.getDoubleValue();
      double kneeLimit = passiveQKneeThreshold.getDoubleValue();
      double kdKnee = passiveKneeKv.getDoubleValue();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         OneDoFJoint kneeJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE);
         double tauKnee = kneeJoint.getTau();
         double qKnee = kneeJoint.getQ();
         double qdKnee = kneeJoint.getQd();
         if (qKnee < kneeLimit)
         {
            passiveKneeTorque.get(robotSide).set(maxPassiveTorque * MathTools.square(1.0 - qKnee / kneeLimit) - kdKnee * qdKnee);
            tauKnee += passiveKneeTorque.get(robotSide).getDoubleValue();
            kneeJoint.setTau(tauKnee);
         }
      }
   }
   
   public void requestResetEstimatorPositionsToCurrent()
   {
      this.resetEstimatorPositionsToCurrent.set(true);
   }

   private void updateMomentumBasedControllerSpy()
   {
      if (momentumBasedControllerSpy != null)
      {
         for (ContactablePlaneBody contactablePlaneBody : yoPlaneContactStates.keySet())
         {
            YoPlaneContactState contactState = yoPlaneContactStates.get(contactablePlaneBody);
            if (contactState.inContact())
            {
               momentumBasedControllerSpy.setPlaneContactState(contactablePlaneBody, contactState.getContactFramePoints2dInContactCopy(),
                     contactState.getCoefficientOfFriction(), contactState.getContactNormalFrameVectorCopy());
            }
         }

         for (ContactableCylinderBody contactableCylinderBody : yoCylindricalContactStates.keySet())
         {
            YoCylindricalContactState contactState = yoCylindricalContactStates.get(contactableCylinderBody);
            if (contactState.isInContact())
               momentumBasedControllerSpy.setCylindricalContactInContact(contactableCylinderBody, contactState.isInContact());
         }

         momentumBasedControllerSpy.doSecondaryControl();
      }
   }

   private void resetGroundReactionWrenchFilter()
   {
      momentumControlModuleBridge.resetGroundReactionWrenchFilter();
   }

   private void callUpdatables()
   {
      double time = yoTime.getDoubleValue();
      for (Updatable updatable : updatables)
      {
         updatable.update(time);
      }
   }

   public void addUpdatable(Updatable updatable)
   {
      updatables.add(updatable);
   }

   public void doPDControl(OneDoFJoint[] joints, double kp, double kd, double maxAcceleration, double maxJerk)
   {
      for (OneDoFJoint joint : joints)
      {
         doPDControl(joint, kp, kd, 0.0, 0.0, maxAcceleration, maxJerk);
      }
   }

   public void doPDControl(OneDoFJoint joint, double kp, double kd, double desiredPosition, double desiredVelocity, double maxAcceleration, double maxJerk)
   {
      double desiredAcceleration = computeDesiredAcceleration(kp, kd, desiredPosition, desiredVelocity, joint);
      desiredAcceleration = MathTools.clipToMinMax(desiredAcceleration, maxAcceleration);
      preRateLimitedDesiredAccelerations.get(joint).set(desiredAcceleration);

      RateLimitedYoVariable rateLimitedDesiredAcceleration = this.rateLimitedDesiredAccelerations.get(joint);
      rateLimitedDesiredAcceleration.setMaxRate(maxJerk);
      rateLimitedDesiredAcceleration.update(desiredAcceleration);
      
      setOneDoFJointAcceleration(joint, rateLimitedDesiredAcceleration.getDoubleValue());
   }

   public void setOneDoFJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      DenseMatrix64F jointAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
      jointAcceleration.set(0, 0, desiredAcceleration);

      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredJointAcceleration(joint, jointAcceleration);
      }

      DesiredJointAccelerationCommand desiredJointAccelerationCommand = new DesiredJointAccelerationCommand(joint, jointAcceleration);
      momentumControlModuleBridge.setDesiredJointAcceleration(desiredJointAccelerationCommand);
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
      // When you initialize into this controller, reset the estimator positions to current. Otherwise it might be in a bad state 
      // where the feet are all jacked up. For example, after falling and getting back up.
      resetEstimatorPositionsToCurrent.set(true);
      inverseDynamicsCalculator.compute();
      momentumControlModuleBridge.initialize();
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

   public FramePoint2d getCoP(ContactablePlaneBody contactablePlaneBody)
   {
      return planeContactWrenchProcessor.getCops().get(contactablePlaneBody);
   }

   public void setPlaneContactState(ContactablePlaneBody contactableBody, boolean[] newContactPointStates)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.setContactPointsInContact(newContactPointStates);
      yoPlaneContactState.setContactNormalVector(null);
   }

   public void setPlaneContactState(ContactablePlaneBody contactableBody, boolean[] newContactPointStates, FrameVector normalContactVector)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.setContactPointsInContact(newContactPointStates);
      yoPlaneContactState.setContactNormalVector(normalContactVector);
   }

   public void setPlaneContactStateFullyConstrained(ContactablePlaneBody contactableBody)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.setFullyConstrained();
      yoPlaneContactState.setContactNormalVector(null);
   }

   public void setPlaneContactStateFullyConstrained(ContactablePlaneBody contactableBody, double coefficientOfFriction, FrameVector normalContactVector)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.setFullyConstrained();
      yoPlaneContactState.setCoefficientOfFriction(coefficientOfFriction);
      yoPlaneContactState.setContactNormalVector(normalContactVector);
   }

   public void setPlaneContactStateFree(ContactablePlaneBody contactableBody)
   {
      YoPlaneContactState yoPlaneContactState = yoPlaneContactStates.get(contactableBody);
      yoPlaneContactState.clear();
      yoPlaneContactState.setContactNormalVector(null);
   }

   public void setCylindricalContactInContact(ContactableCylinderBody contactableCylinderBody, boolean setInContact)
   {
      YoCylindricalContactState yoCylindricalContactState = yoCylindricalContactStates.get(contactableCylinderBody);
      yoCylindricalContactState.setInContact(setInContact);
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

      DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand = new DesiredSpatialAccelerationCommand(jacobian, taskspaceConstraintData);
      momentumControlModuleBridge.setDesiredSpatialAcceleration(desiredSpatialAccelerationCommand);
   }

   public void setDesiredPointAcceleration(GeometricJacobian rootToEndEffectorJacobian, FramePoint contactPoint, FrameVector desiredAcceleration)
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredPointAcceleration(rootToEndEffectorJacobian, contactPoint, desiredAcceleration);
      }

      DesiredPointAccelerationCommand desiredPointAccelerationCommand = new DesiredPointAccelerationCommand(rootToEndEffectorJacobian, contactPoint,
            desiredAcceleration);
      momentumControlModuleBridge.setDesiredPointAcceleration(desiredPointAccelerationCommand);
   }

   public void setDesiredPointAcceleration(GeometricJacobian rootToEndEffectorJacobian, FramePoint contactPoint, FrameVector desiredAcceleration,
         DenseMatrix64F selectionMatrix)
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredPointAcceleration(rootToEndEffectorJacobian, contactPoint, desiredAcceleration);
      }

      DesiredPointAccelerationCommand desiredPointAccelerationCommand = new DesiredPointAccelerationCommand(rootToEndEffectorJacobian, contactPoint,
            desiredAcceleration, selectionMatrix);
      momentumControlModuleBridge.setDesiredPointAcceleration(desiredPointAccelerationCommand);
   }

   public void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData)
   {
      if (momentumBasedControllerSpy != null)
      {
         momentumBasedControllerSpy.setDesiredRateOfChangeOfMomentum(momentumRateOfChangeData);
      }

      DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand = new DesiredRateOfChangeOfMomentumCommand(momentumRateOfChangeData);
      momentumControlModuleBridge.setDesiredRateOfChangeOfMomentum(desiredRateOfChangeOfMomentumCommand);
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

   public boolean isUsingOptimizationMomentumControlModule()
   {
      return momentumControlModuleBridge.isUsingOptimizationMomentumControlModule();
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

   public SideDependentList<ContactablePlaneBody> getContactablePlaneHandsWithFingersBentBack()
   {
      return handsWithFingersBentBack;
   }

   public ContactablePlaneBody getContactablePlaneHandWithFingersBentBack(RobotSide robotSide)
   {
      return handsWithFingersBentBack.get(robotSide);
   }

   public SideDependentList<ContactableCylinderBody> getContactableCylinderHands()
   {
      return graspingHands;
   }

   public ContactableCylinderBody getContactableCylinderHand(RobotSide robotSide)
   {
      return graspingHands.get(robotSide);
   }

   public SideDependentList<ContactablePlaneBody> getContactablePlaneThighs()
   {
      return thighs;
   }

   public List<FramePoint> getContactPoints(ContactablePlaneBody contactablePlaneBody)
   {
      return yoPlaneContactStates.get(contactablePlaneBody).getContactFramePointsInContactCopy();
   }

   public PlaneContactState getContactState(ContactablePlaneBody contactablePlaneBody)
   {
      return yoPlaneContactStates.get(contactablePlaneBody);
   }

   public Collection<? extends PlaneContactState> getPlaneContactStates()
   {
      return yoPlaneContactStates.values();
   }

   public void clearContacts()
   {
      for (ModifiableContactState modifiableContactState : modifiableContactStates)
      {
         modifiableContactState.clear();
      }
   }

   public ContactablePlaneBody getContactablePlanePelvis()
   {
      return pelvis;
   }

   public ContactablePlaneBody getContactablePlanePelvisBack()
   {
      return pelvisBack;
   }

   public void setMomentumControlModuleToUse(MomentumControlModuleType momentumControlModuleToUse)
   {
      momentumControlModuleBridge.setMomentumControlModuleToUse(momentumControlModuleToUse);
   }

   public void setDelayTimeBeforeTrustingContacts(double delayTimeBeforeTrustingContacts)
   {
      pointPositionGrabber.setDelayTimeBeforeTrustingContacts(delayTimeBeforeTrustingContacts);
   }

   //TODO (Sylvain): get rid of these methods changing wRho & wPhi of contact state
   public void setPlaneContactState_wRho(ContactablePlaneBody contactablePlaneBody, double wRho)
   {
      yoPlaneContactStates.get(contactablePlaneBody).setRhoContactRegularization(wRho);
   }

   public void setCylindricalContactState_wRho_wPhi(ContactableCylinderBody contactableCylinderBody, double wRho, double wPhi)
   {
      yoCylindricalContactStates.get(contactableCylinderBody).setRhoContactRegularization(wRho);
      yoCylindricalContactStates.get(contactableCylinderBody).setPhiContactRegularization(wPhi);
   }

   public void setCylindricalContactStateProperties(ContactableCylinderBody contactableCylinderBody, double coefficientOfFriction, double gripStrength,
         double cylinderRadius, double halfHandWidth, double gripWeaknessFactor, boolean inContact)
   {
      yoCylindricalContactStates.get(contactableCylinderBody).set(coefficientOfFriction, gripStrength, cylinderRadius, halfHandWidth, gripWeaknessFactor,
            inContact);
   }

   public void setCylindricalContactStateProperties(ContactableCylinderBody contactableCylinderBody, double coefficientOfFriction, boolean inContact)
   {
      yoCylindricalContactStates.get(contactableCylinderBody).set(coefficientOfFriction, contactableCylinderBody, inContact);
   }
}
