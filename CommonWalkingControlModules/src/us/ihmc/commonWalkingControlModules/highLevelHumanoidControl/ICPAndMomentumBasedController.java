package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;

import us.ihmc.commonWalkingControlModules.WrenchDistributorTools;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.calculators.Omega0Calculator;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributorInputData;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributorOutputData;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumSolver;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationData;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TotalWrenchCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoFrameVector;
import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

public abstract class ICPAndMomentumBasedController implements RobotController
{
   private static final long serialVersionUID = -7013956504623280825L;

   private final String name = getClass().getSimpleName();
   protected final YoVariableRegistry registry = new YoVariableRegistry(name);

   protected final MomentumSolver solver;

   protected final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected final ReferenceFrame elevatorFrame;
   private final ReferenceFrame centerOfMassFrame;

   protected final FullRobotModel fullRobotModel;
   private final CenterOfMassJacobian centerOfMassJacobian;
   protected final CommonWalkingReferenceFrames referenceFrames;
   protected final TwistCalculator twistCalculator;
   protected final SpatialAccelerationCalculator spatialAccelerationCalculator;
   protected final List<ContactablePlaneBody> contactablePlaneBodies;
   protected final SideDependentList<? extends ContactablePlaneBody> bipedFeet;
   private final HashMap<ContactablePlaneBody, YoFramePoint> centersOfPressure = new HashMap<ContactablePlaneBody, YoFramePoint>();
   protected final HashMap<ContactablePlaneBody, YoFramePoint2d> centersOfPressure2d = new HashMap<ContactablePlaneBody, YoFramePoint2d>();
   protected final LinkedHashMap<ContactablePlaneBody, YoPlaneContactState> contactStates = new LinkedHashMap<ContactablePlaneBody, YoPlaneContactState>();
   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();


   protected final DoubleYoVariable yoTime;
   protected final double controlDT;
   protected final double gravity;


   protected final SideDependentList<FootSwitchInterface> footSwitches;
   protected final BipedSupportPolygons bipedSupportPolygons;
   protected final YoFramePoint2d desiredICP;
   protected final YoFrameVector2d desiredICPVelocity;
   protected final EnumYoVariable<RobotSide> supportLeg;
   protected final EnumYoVariable<RobotSide> upcomingSupportLeg;
   protected final DoubleYoVariable desiredCoMHeightAcceleration;
   protected final YoFramePoint capturePoint;
   protected final DoubleYoVariable omega0;
   private final Omega0Calculator omega0Calculator;

   private final YoFrameVector finalDesiredPelvisLinearAcceleration;
   private final YoFrameVector finalDesiredPelvisAngularAcceleration;
   private final YoFrameVector desiredPelvisForce;
   private final YoFrameVector desiredPelvisTorque;

   private final DoubleYoVariable alphaGroundReactionWrench = new DoubleYoVariable("alphaGroundReactionWrench", registry);
   private final YoFrameVector unfilteredDesiredGroundReactionTorque;
   private final YoFrameVector unfilteredDesiredGroundReactionForce;
   private final AlphaFilteredYoFrameVector desiredGroundReactionTorque;
   private final AlphaFilteredYoFrameVector desiredGroundReactionForce;
   private final YoFrameVector admissibleDesiredGroundReactionTorque;
   private final YoFrameVector admissibleDesiredGroundReactionForce;
   private final YoFrameVector groundReactionTorqueCheck;
   private final YoFrameVector groundReactionForceCheck;

   private final HashMap<RevoluteJoint, DoubleYoVariable> desiredAccelerationYoVariables = new HashMap<RevoluteJoint, DoubleYoVariable>();
   private final SpatialForceVector gravitationalWrench;

   private final ProcessedOutputsInterface processedOutputs;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final double totalMass;

   private final GroundReactionWrenchDistributor groundReactionWrenchDistributor;

   private final MomentumRateOfChangeControlModule momentumRateOfChangeControlModule;
   private final RootJointAccelerationControlModule rootJointAccelerationControlModule;

   public ICPAndMomentumBasedController(FullRobotModel fullRobotModel, CenterOfMassJacobian centerOfMassJacobian, CommonWalkingReferenceFrames referenceFrames,
           DoubleYoVariable yoTime, double gravityZ, TwistCalculator twistCalculator, SideDependentList<? extends ContactablePlaneBody> bipedFeet,
           BipedSupportPolygons bipedSupportPolygons, double controlDT, ProcessedOutputsInterface processedOutputs,
           SideDependentList<FootSwitchInterface> footSwitches, GroundReactionWrenchDistributor groundReactionWrenchDistributor,
           ArrayList<Updatable> updatables, MomentumRateOfChangeControlModule momentumRateOfChangeControlModule,
           RootJointAccelerationControlModule rootJointAccelerationControlModule, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      LinearSolver<DenseMatrix64F> jacobianSolver = createJacobianSolver();
      this.solver = new MomentumSolver(rootJoint, rootJoint.getPredecessor(), centerOfMassFrame, twistCalculator, jacobianSolver, controlDT, registry);

      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.fullRobotModel = fullRobotModel;
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.referenceFrames = referenceFrames;
      this.twistCalculator = twistCalculator;
      this.bipedFeet = bipedFeet;
      this.contactablePlaneBodies = new ArrayList<ContactablePlaneBody>(bipedFeet.values());
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.controlDT = controlDT;
      this.footSwitches = footSwitches;
      this.gravity = gravityZ;
      this.yoTime = yoTime;

      RigidBody elevator = fullRobotModel.getElevator();
      this.spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, gravity, true);

      this.processedOutputs = processedOutputs;
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);

      this.totalMass = TotalMassCalculator.computeSubTreeMass(elevator);
      this.omega0Calculator = new Omega0Calculator(centerOfMassFrame, totalMass);

      this.groundReactionWrenchDistributor = groundReactionWrenchDistributor;

      omega0 = new DoubleYoVariable("omega0", registry);
      capturePoint = new YoFramePoint("capturePoint", worldFrame, registry);

      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      this.finalDesiredPelvisLinearAcceleration = new YoFrameVector("finalDesiredPelvisLinearAcceleration", "", pelvisFrame, registry);
      this.finalDesiredPelvisAngularAcceleration = new YoFrameVector("finalDesiredPelvisAngularAcceleration", "", pelvisFrame, registry);
      this.desiredPelvisForce = new YoFrameVector("desiredPelvisForce", "", centerOfMassFrame, registry);
      this.desiredPelvisTorque = new YoFrameVector("desiredPelvisTorque", "", centerOfMassFrame, registry);

      this.unfilteredDesiredGroundReactionTorque = new YoFrameVector("unfilteredDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.unfilteredDesiredGroundReactionForce = new YoFrameVector("unfilteredDesiredGroundReactionForce", centerOfMassFrame, registry);

      alphaGroundReactionWrench.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(7.0, controlDT));    // FIXME: use setter

      this.desiredGroundReactionTorque = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("desiredGroundReactionTorque", "", registry,
              alphaGroundReactionWrench, unfilteredDesiredGroundReactionTorque);
      this.desiredGroundReactionForce = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector("desiredGroundReactionForce", "", registry,
              alphaGroundReactionWrench, unfilteredDesiredGroundReactionForce);

      this.admissibleDesiredGroundReactionTorque = new YoFrameVector("admissibleDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.admissibleDesiredGroundReactionForce = new YoFrameVector("admissibleDesiredGroundReactionForce", centerOfMassFrame, registry);

      this.groundReactionTorqueCheck = new YoFrameVector("groundReactionTorqueCheck", centerOfMassFrame, registry);
      this.groundReactionForceCheck = new YoFrameVector("groundReactionForceCheck", centerOfMassFrame, registry);

      for (ContactablePlaneBody contactableBody : bipedFeet)
      {
         String copName = contactableBody.getRigidBody().getName() + "CoP";
         String listName = "cops";

         YoFramePoint cop = new YoFramePoint(copName, worldFrame, registry);
         centersOfPressure.put(contactableBody, cop);

         YoFramePoint2d cop2d = new YoFramePoint2d(copName + "2d", "", contactableBody.getPlaneFrame(), registry);
         centersOfPressure2d.put(contactableBody, cop2d);

         DynamicGraphicPosition copViz = cop.createDynamicGraphicPosition(copName, 0.005, YoAppearance.Navy(), GraphicType.BALL);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, copViz);
         dynamicGraphicObjectsListRegistry.registerArtifact(listName, copViz.createArtifact());
      }

      DynamicGraphicPosition capturePointViz = capturePoint.createDynamicGraphicPosition("Capture Point", 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("Capture Point", capturePointViz);
      dynamicGraphicObjectsListRegistry.registerArtifact("Capture Point", capturePointViz.createArtifact());

      if (updatables != null)
      {
         this.updatables.addAll(updatables);
      }

      this.desiredCoMHeightAcceleration = new DoubleYoVariable("desiredCoMHeightAcceleration", registry);

      desiredICP = new YoFramePoint2d("desiredICP", "", ReferenceFrame.getWorldFrame(), registry);
      desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", ReferenceFrame.getWorldFrame(), registry);

      supportLeg = EnumYoVariable.create("supportLeg", "", RobotSide.class, registry, true);
      upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", "", RobotSide.class, registry, true);

      elevatorFrame = fullRobotModel.getElevatorFrame();

      for (ContactablePlaneBody contactablePlaneBody : bipedFeet)
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         YoPlaneContactState contactState = new YoPlaneContactState(rigidBody.getName(), contactablePlaneBody.getPlaneFrame(), registry);
         contactState.setContactPoints(contactablePlaneBody.getContactPoints2d());    // initialize with flat 'feet'
         contactStates.put(contactablePlaneBody, contactState);
      }

      InverseDynamicsJoint[] joints = ScrewTools.computeJointsInOrder(elevator);
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof RevoluteJoint)
         {
            desiredAccelerationYoVariables.put((RevoluteJoint) joint, new DoubleYoVariable(joint.getName() + "qdd_d", registry));
         }
      }

      this.momentumRateOfChangeControlModule = momentumRateOfChangeControlModule;
      this.rootJointAccelerationControlModule = rootJointAccelerationControlModule;

      gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());

      omega0.set(3.39);    // FIXME: hack to resolve circularity
      computeCapturePoint();


      this.updatables.add(new FootPolygonVisualizer(contactStates.values(), dynamicGraphicObjectsListRegistry, registry));

   }


   private static LinearSolver<DenseMatrix64F> createJacobianSolver()
   {
      DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
      jacobianSolver.setAlpha(5e-2);

      return jacobianSolver;
   }

   protected void setExternalHandWrench(RobotSide robotSide, Wrench handWrench)
   {
      inverseDynamicsCalculator.setExternalWrench(fullRobotModel.getHand(robotSide), handWrench);
   }

   public abstract void doMotionControl();

   public final void doControl()
   {
      updateBipedSupportPolygons(bipedSupportPolygons);

      callUpdatables();

      inverseDynamicsCalculator.reset();
      solver.reset();

      doMotionControl();

      solver.compute();

      rootJointAccelerationControlModule.startComputation();
      rootJointAccelerationControlModule.waitUntilComputationIsDone();
      RootJointAccelerationData rootJointAccelerationData = rootJointAccelerationControlModule.getRootJointAccelerationOutputPort().getData();

      momentumRateOfChangeControlModule.startComputation();
      momentumRateOfChangeControlModule.waitUntilComputationIsDone();
      MomentumRateOfChangeData momentumRateOfChangeData = momentumRateOfChangeControlModule.getMomentumRateOfChangeOutputPort().getData();

      solver.solve(rootJointAccelerationData.getAccelerationSubspace(), rootJointAccelerationData.getAccelerationMultipliers(),
                   momentumRateOfChangeData.getMomentumSubspace(), momentumRateOfChangeData.getMomentumMultipliers());

      SpatialForceVector totalGroundReactionWrench = new SpatialForceVector(centerOfMassFrame);
      solver.getRateOfChangeOfMomentum(totalGroundReactionWrench);
      totalGroundReactionWrench.add(gravitationalWrench);

      unfilteredDesiredGroundReactionTorque.set(totalGroundReactionWrench.getAngularPartCopy());
      unfilteredDesiredGroundReactionForce.set(totalGroundReactionWrench.getLinearPartCopy());
      desiredGroundReactionTorque.update();
      desiredGroundReactionForce.update();
      totalGroundReactionWrench.setAngularPart(desiredGroundReactionTorque.getFrameVectorCopy().getVector());
      totalGroundReactionWrench.setLinearPart(desiredGroundReactionForce.getFrameVectorCopy().getVector());

      GroundReactionWrenchDistributorInputData groundReactionWrenchDistributorInputData = new GroundReactionWrenchDistributorInputData();

      double coefficientOfFriction = 1.0;    // 0.5;    // TODO
      double rotationalCoefficientOfFriction = 0.5;    // TODO

      groundReactionWrenchDistributorInputData.reset();

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         PlaneContactState contactState = contactStates.get(contactablePlaneBody);

         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            groundReactionWrenchDistributorInputData.addContact(contactState, coefficientOfFriction, rotationalCoefficientOfFriction);
         }
      }

      groundReactionWrenchDistributorInputData.setSpatialForceVectorAndUpcomingSupportSide(totalGroundReactionWrench, upcomingSupportLeg.getEnumValue());

      GroundReactionWrenchDistributorOutputData distributedWrench = new GroundReactionWrenchDistributorOutputData();
      groundReactionWrenchDistributor.solve(distributedWrench, groundReactionWrenchDistributorInputData);

      List<Wrench> wrenches = new ArrayList<Wrench>();
      List<FramePoint2d> cops = new ArrayList<FramePoint2d>();

      for (ContactablePlaneBody contactablePlaneBody : contactablePlaneBodies)
      {
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         PlaneContactState contactState = contactStates.get(contactablePlaneBody);
         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            FrameVector force = distributedWrench.getForce(contactState);
            FramePoint2d cop = distributedWrench.getCenterOfPressure(contactState);
            double normalTorque = distributedWrench.getNormalTorque(contactState);

            centersOfPressure2d.get(contactablePlaneBody).set(cop);

            cops.add(cop);
            FramePoint cop3d = cop.toFramePoint();
            cop3d.changeFrame(worldFrame);
            centersOfPressure.get(contactablePlaneBody).set(cop3d);
            Wrench groundReactionWrench = new Wrench(rigidBody.getBodyFixedFrame(), contactState.getPlaneFrame());
            WrenchDistributorTools.computeWrench(groundReactionWrench, force, cop, normalTorque);
            groundReactionWrench.changeFrame(rigidBody.getBodyFixedFrame());
            wrenches.add(groundReactionWrench);
            inverseDynamicsCalculator.setExternalWrench(rigidBody, groundReactionWrench);
         }
         else
         {
            centersOfPressure.get(contactablePlaneBody).setToNaN();
         }
      }

      Wrench admissibleGroundReactionWrench = TotalWrenchCalculator.computeTotalWrench(wrenches, totalGroundReactionWrench.getExpressedInFrame());
      admissibleDesiredGroundReactionTorque.set(admissibleGroundReactionWrench.getAngularPartCopy());
      admissibleDesiredGroundReactionForce.set(admissibleGroundReactionWrench.getLinearPartCopy());
      this.omega0.set(omega0Calculator.computeOmega0(cops, admissibleGroundReactionWrench));

      SpatialForceVector desiredCentroidalMomentumRate = new SpatialForceVector();
      desiredCentroidalMomentumRate.set(admissibleGroundReactionWrench);
      desiredCentroidalMomentumRate.sub(gravitationalWrench);

      solver.solve(desiredCentroidalMomentumRate);

      SpatialForceVector groundReactionWrenchCheck = inverseDynamicsCalculator.computeTotalExternalWrench(centerOfMassFrame);
      groundReactionTorqueCheck.set(groundReactionWrenchCheck.getAngularPartCopy());
      groundReactionForceCheck.set(groundReactionWrenchCheck.getLinearPartCopy());

      inverseDynamicsCalculator.compute();
      if (processedOutputs != null)
         fullRobotModel.setTorques(processedOutputs);
      updateYoVariables();
   }


   private void callUpdatables()
   {
      double time = yoTime.getDoubleValue();
      for (Updatable updatable : updatables)
      {
         updatable.update(time);
      }
   }

   // FIXME: get rid of this
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
         doPDControl(joint, k, d);
      }
   }

   protected void doPDControl(OneDoFJoint joint, double k, double d)
   {
      double desiredAcceleration = computeDesiredAcceleration(k, d, 0.0, 0.0, joint);
      DenseMatrix64F jointAcceleration = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
      jointAcceleration.set(0, 0, desiredAcceleration);
      solver.setDesiredJointAcceleration(joint, jointAcceleration);
   }

   protected static double computeDesiredAcceleration(double k, double d, double qDesired, double qdDesired, OneDoFJoint joint)
   {
      return k * (qDesired - joint.getQ()) + d * (qdDesired - joint.getQd());
   }

   protected void computeCapturePoint()
   {
      FramePoint centerOfMass = computeCenterOfMass();
      FrameVector centerOfMassVelocity = computeCenterOfMassVelocity();
      FramePoint2d capturePoint = CapturePointCalculator.computeCapturePoint(centerOfMass, centerOfMassVelocity, omega0.getDoubleValue());
      capturePoint.changeFrame(this.capturePoint.getReferenceFrame());
      this.capturePoint.set(capturePoint.getX(), capturePoint.getY(), 0.0);
   }

   private FramePoint computeCenterOfMass()
   {
      return new FramePoint(referenceFrames.getCenterOfMassFrame());
   }

   private FrameVector computeCenterOfMassVelocity()
   {
      centerOfMassJacobian.compute();
      FrameVector ret = new FrameVector(ReferenceFrame.getWorldFrame());
      centerOfMassJacobian.packCenterOfMassVelocity(ret);

      return ret;
   }

   protected double getOmega0()
   {
      return omega0.getDoubleValue();
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

      for (RevoluteJoint joint : desiredAccelerationYoVariables.keySet())
      {
         desiredAccelerationYoVariables.get(joint).set(joint.getQddDesired());
      }
   }

   protected void updateBipedSupportPolygons(BipedSupportPolygons bipedSupportPolygons)
   {
      SideDependentList<List<FramePoint>> footContactPoints = new SideDependentList<List<FramePoint>>();
      for (RobotSide robotSide : RobotSide.values())
      {
         footContactPoints.put(robotSide, contactStates.get(bipedFeet.get(robotSide)).getContactPoints());
      }

      bipedSupportPolygons.update(footContactPoints);
   }

   public void initialize()
   {
      solver.initialize();
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
}
