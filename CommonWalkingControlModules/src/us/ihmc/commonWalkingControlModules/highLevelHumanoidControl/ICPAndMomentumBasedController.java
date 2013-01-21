package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;

import us.ihmc.commonWalkingControlModules.CenterOfPressureTools;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.FootPolygonVisualizer;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionMomentControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributorInterface;
import us.ihmc.commonWalkingControlModules.controlModules.SacrificeDeltaCMPDesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.SimpleDesiredCenterOfPressureFilter;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.kinematics.SpatialAccelerationProjector;
import us.ihmc.commonWalkingControlModules.momentumBasedController.CapturePointCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.commonWalkingControlModules.sensors.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.MechanismGeometricJacobian;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.OriginAndPointFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.CenterOfMassJacobian;
import us.ihmc.utilities.screwTheory.EndEffectorPoseTwistAndSpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.MomentumCalculator;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TotalWrenchCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition.GraphicType;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameOrientation;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint2d;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector2d;

/**
 * @author twan
 *
 */
public abstract class ICPAndMomentumBasedController extends MomentumBasedController
{
   private static final long serialVersionUID = -7013956504623280825L;
   protected final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected final ReferenceFrame elevatorFrame;

   protected final FullRobotModel fullRobotModel;
   private final CenterOfMassJacobian centerOfMassJacobian;
   protected final CommonWalkingReferenceFrames referenceFrames;
   protected final TwistCalculator twistCalculator;
   protected final SpatialAccelerationCalculator spatialAccelerationCalculator;
   protected final SideDependentList<? extends ContactablePlaneBody> bipedFeet;
   private final HashMap<ContactablePlaneBody, YoFramePoint> centersOfPressure = new HashMap<ContactablePlaneBody, YoFramePoint>();
   protected final LinkedHashMap<RigidBody, YoPlaneContactState> contactStates = new LinkedHashMap<RigidBody, YoPlaneContactState>();
   private final ArrayList<Updatable> updatables = new ArrayList<Updatable>();

   protected final SideDependentList<FootSwitchInterface> footSwitches;

   protected final double controlDT;
   protected final double gravity;

   protected final DoubleYoVariable yoTime;

   protected final YoFramePoint2d desiredICP;
   protected final YoFrameVector2d desiredICPVelocity;
   protected final EnumYoVariable<RobotSide> supportLeg;
   protected final EnumYoVariable<RobotSide> upcomingSupportLeg;


   protected final DoubleYoVariable desiredCoMHeightAcceleration;
   protected final SideDependentList<EnumMap<LimbName, MechanismGeometricJacobian>> jacobians = SideDependentList.createListOfEnumMaps(LimbName.class);
   protected final MechanismGeometricJacobian spineJacobian;

   protected final YoFrameOrientation desiredPelvisOrientation;

   protected final YoFramePoint capturePoint;
   protected final DoubleYoVariable omega0;

   private final DesiredCoPAndCMPControlModule desiredCoPAndCMPControlModule;

   private final YoFrameVector desiredPelvisLinearAcceleration;
   private final YoFrameVector desiredPelvisAngularAcceleration;
   private final YoFrameVector desiredPelvisForce;
   private final YoFrameVector desiredPelvisTorque;

   private final YoFrameVector desiredGroundReactionTorque;
   private final YoFrameVector desiredGroundReactionForce;
   private final YoFrameVector admissibleDesiredGroundReactionTorque;
   private final YoFrameVector admissibleDesiredGroundReactionForce;

   private final HashMap<RevoluteJoint, DoubleYoVariable> desiredAccelerationYoVariables = new HashMap<RevoluteJoint, DoubleYoVariable>();
   private final DoubleYoVariable fZ = new DoubleYoVariable("fZ", registry);
   private final SpatialForceVector gravitationalWrench;

   private final ReferenceFrame centerOfMassFrame;

   private final MomentumCalculator momentumCalculator;

   private final ProcessedOutputsInterface processedOutputs;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   protected final BipedSupportPolygons bipedSupportPolygons;

   private final double gravityZ;
   private final double totalMass;

   private final GroundReactionMomentControlModule groundReactionMomentControlModule;

   private final GroundReactionWrenchDistributorInterface groundReactionWrenchDistributor;
   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

   private final SideDependentList<EndEffectorPoseTwistAndSpatialAccelerationCalculator> footPoseTwistAndSpatialAccelerationCalculators =
      new SideDependentList<EndEffectorPoseTwistAndSpatialAccelerationCalculator>();
   private final SideDependentList<SpatialAccelerationProjector> spatialAccelerationProjectors = new SideDependentList<SpatialAccelerationProjector>();
   private final SideDependentList<BooleanYoVariable> isCoPOnEdge = new SideDependentList<BooleanYoVariable>();

   private final OriginAndPointFrame copToCoPFrame = new OriginAndPointFrame("copToCoP", worldFrame);

   public ICPAndMomentumBasedController(FullRobotModel fullRobotModel, CenterOfMassJacobian centerOfMassJacobian, CommonWalkingReferenceFrames referenceFrames,
           DoubleYoVariable yoTime, double gravityZ, TwistCalculator twistCalculator, SideDependentList<? extends ContactablePlaneBody> bipedFeet,
           BipedSupportPolygons bipedSupportPolygons, double controlDT, ProcessedOutputsInterface processedOutputs,
           SideDependentList<FootSwitchInterface> footSwitches, GroundReactionWrenchDistributorInterface groundReactionWrenchDistributor,
           ArrayList<Updatable> updatables, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      super(fullRobotModel.getRootJoint(), referenceFrames.getCenterOfMassFrame(), twistCalculator, createJacobianSolver(), controlDT);
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.fullRobotModel = fullRobotModel;
      this.centerOfMassJacobian = centerOfMassJacobian;
      this.referenceFrames = referenceFrames;
      this.twistCalculator = twistCalculator;
      this.bipedFeet = bipedFeet;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.controlDT = controlDT;
      this.footSwitches = footSwitches;
      this.gravity = gravityZ;
      this.yoTime = yoTime;

      RigidBody elevator = fullRobotModel.getElevator();
      this.spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, gravity, true);
      this.centerOfMassFrame = referenceFrames.getCenterOfMassFrame();

      this.processedOutputs = processedOutputs;
      this.gravityZ = gravityZ;
      this.momentumCalculator = new MomentumCalculator(twistCalculator);
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);

      this.groundReactionMomentControlModule = new GroundReactionMomentControlModule(fullRobotModel.getPelvis().getBodyFixedFrame(), registry);
      this.groundReactionMomentControlModule.setGains(10.0, 100.0);    // kPelvisYaw was 0.0 for M3 movie
      this.totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      this.groundReactionWrenchDistributor = groundReactionWrenchDistributor;
      this.desiredPelvisOrientation = new YoFrameOrientation("desiredPelvis", worldFrame, registry);

      omega0 = new DoubleYoVariable("omega0", registry);
      capturePoint = new YoFramePoint("capturePoint", worldFrame, registry);

      this.desiredPelvisLinearAcceleration = new YoFrameVector("desiredPelvisLinearAcceleration", "", referenceFrames.getPelvisFrame(), registry);
      this.desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", "", referenceFrames.getPelvisFrame(), registry);
      this.desiredPelvisForce = new YoFrameVector("desiredPelvisForce", "", centerOfMassFrame, registry);
      this.desiredPelvisTorque = new YoFrameVector("desiredPelvisTorque", "", centerOfMassFrame, registry);

      this.desiredGroundReactionTorque = new YoFrameVector("desiredGroundReactionTorque", centerOfMassFrame, registry);
      this.desiredGroundReactionForce = new YoFrameVector("desiredGroundReactionForce", centerOfMassFrame, registry);
      this.admissibleDesiredGroundReactionTorque = new YoFrameVector("admissibleDesiredGroundReactionTorque", centerOfMassFrame, registry);
      this.admissibleDesiredGroundReactionForce = new YoFrameVector("admissibleDesiredGroundReactionForce", centerOfMassFrame, registry);

      for (ContactablePlaneBody contactableBody : bipedFeet)
      {
         String copName = contactableBody.getRigidBody().getName() + "CoP";
         String listName = "cops";
         YoFramePoint cop = new YoFramePoint(copName, worldFrame, registry);
         DynamicGraphicPosition copViz = cop.createDynamicGraphicPosition(copName, 0.005, YoAppearance.Navy(), GraphicType.BALL);
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(listName, copViz);
         dynamicGraphicObjectsListRegistry.registerArtifact(listName, copViz.createArtifact());
         centersOfPressure.put(contactableBody, cop);
      }

      DynamicGraphicPosition capturePointViz = capturePoint.createDynamicGraphicPosition("Capture Point", 0.01, YoAppearance.Blue(), GraphicType.ROTATED_CROSS);
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject("Capture Point", capturePointViz);
      dynamicGraphicObjectsListRegistry.registerArtifact("Capture Point", capturePointViz.createArtifact());

      if (updatables != null)
      {
         this.updatables.addAll(updatables);
      }

      this.spineJacobian = new MechanismGeometricJacobian(fullRobotModel.getPelvis(), fullRobotModel.getChest(),
              fullRobotModel.getRootJoint().getFrameAfterJoint());

      this.desiredCoMHeightAcceleration = new DoubleYoVariable("desiredCoMHeightAcceleration", registry);

      desiredICP = new YoFramePoint2d("desiredICP", "", ReferenceFrame.getWorldFrame(), registry);
      desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", "", ReferenceFrame.getWorldFrame(), registry);

      supportLeg = EnumYoVariable.create("supportLeg", "", RobotSide.class, registry, true);
      upcomingSupportLeg = EnumYoVariable.create("upcomingSupportLeg", "", RobotSide.class, registry, true);

      elevatorFrame = fullRobotModel.getElevatorFrame();

      EnumMap<LimbName, RigidBody> bases = new EnumMap<LimbName, RigidBody>(LimbName.class);
      bases.put(LimbName.LEG, fullRobotModel.getPelvis());
      bases.put(LimbName.ARM, fullRobotModel.getChest());

      for (RobotSide robotSide : RobotSide.values())
      {
         for (LimbName limbName : LimbName.values())
         {
            RigidBody endEffector = fullRobotModel.getEndEffector(robotSide, limbName);
            MechanismGeometricJacobian jacobian = new MechanismGeometricJacobian(bases.get(limbName), endEffector, endEffector.getBodyFixedFrame());
            jacobians.get(robotSide).put(limbName, jacobian);
            if (limbName == LimbName.LEG)    // because it needs sole frame
               contactStates.put(endEffector, new YoPlaneContactState(endEffector.getName(), referenceFrames.getSoleFrame(robotSide), registry));
         }
      }

      InverseDynamicsJoint[] joints = ScrewTools.computeJointsInOrder(elevator);
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof RevoluteJoint)
         {
            desiredAccelerationYoVariables.put((RevoluteJoint) joint, new DoubleYoVariable(joint.getName() + "qdd_d", registry));
         }
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ContactablePlaneBody contactablePlaneBody = bipedFeet.get(robotSide);
         contactStates.get(foot).setContactPoints(contactablePlaneBody.getContactPoints2d());    // flat feet
      }

      for (RobotSide robotSide : RobotSide.values())
      {
         spatialAccelerationProjectors.put(robotSide,
                                           new SpatialAccelerationProjector(robotSide.getCamelCaseNameForStartOfExpression()
                                              + "FootSpatialAccelerationProjector", registry));
         isCoPOnEdge.put(robotSide, new BooleanYoVariable("is" + robotSide.getCamelCaseNameForMiddleOfExpression() + "CoPOnEdge", registry));

         EndEffectorPoseTwistAndSpatialAccelerationCalculator feetPoseTwistAndSpatialAccelerationCalculator =
            new EndEffectorPoseTwistAndSpatialAccelerationCalculator(fullRobotModel.getEndEffector(robotSide, LimbName.LEG),
               fullRobotModel.getEndEffectorFrame(robotSide, LimbName.LEG), twistCalculator);
         footPoseTwistAndSpatialAccelerationCalculators.put(robotSide, feetPoseTwistAndSpatialAccelerationCalculator);
      }

      updateBipedSupportPolygons(bipedSupportPolygons);

      SimpleDesiredCenterOfPressureFilter desiredCenterOfPressureFilter = new SimpleDesiredCenterOfPressureFilter(bipedSupportPolygons, referenceFrames,
                                                                             controlDT, registry);
      desiredCenterOfPressureFilter.setParametersForR2InverseDynamics();

      CapturabilityBasedDesiredCoPVisualizer visualizer = new CapturabilityBasedDesiredCoPVisualizer(registry, dynamicGraphicObjectsListRegistry);
      SacrificeDeltaCMPDesiredCoPAndCMPControlModule desiredCoPAndCMPControlModule =
         new SacrificeDeltaCMPDesiredCoPAndCMPControlModule(desiredCenterOfPressureFilter, visualizer, bipedSupportPolygons,
            fullRobotModel.getPelvis().getBodyFixedFrame(), controlDT, registry);
      desiredCoPAndCMPControlModule.setGains(3e-2, 1.0, 1.5, 15.0);
      this.desiredCoPAndCMPControlModule = desiredCoPAndCMPControlModule;

      gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());

      omega0.set(3.39);    // FIXME: hack to resolve circularity
      computeCapturePoint();

      SideDependentList<ContactState> footContactStates = new SideDependentList<ContactState>();
      for (RobotSide robotSide : RobotSide.values())
      {
         footContactStates.put(robotSide, contactStates.get(fullRobotModel.getFoot(robotSide)));
      }

      this.updatables.add(new FootPolygonVisualizer(footContactStates, dynamicGraphicObjectsListRegistry, registry));
   }


   private static LinearSolver<DenseMatrix64F> createJacobianSolver()
   {
      DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
      jacobianSolver.setAlpha(5e-2);

      return jacobianSolver;
   }


   protected void setEndEffectorSpatialAcceleration(RobotSide robotSide, LimbName limbName,
           Pair<SpatialAccelerationVector, DenseMatrix64F> endEffectorSpatialAcceleration)
   {
      MechanismGeometricJacobian jacobian = jacobians.get(robotSide).get(limbName);
      endEffectorSpatialAcceleration.first().getBodyFrame().checkReferenceFrameMatch(jacobian.getEndEffectorFrame());
      solver.setDesiredSpatialAcceleration(jacobian, endEffectorSpatialAcceleration.first(), endEffectorSpatialAcceleration.second());    // TODO: get rid of pair
   }

   protected void setExternalHandWrench(RobotSide robotSide, Wrench handWrench)
   {
      inverseDynamicsCalculator.setExternalWrench(fullRobotModel.getHand(robotSide), handWrench);
   }

   public final void doControl()
   {
      Momentum momentum = computeCentroidalMomentum();

      updateBipedSupportPolygons(bipedSupportPolygons);

      update();

      inverseDynamicsCalculator.reset();
      solver.reset();

      doMotionControl();

      ReferenceFrame frame = worldFrame;
      RobotSide supportLeg = this.supportLeg.getEnumValue();

      FrameOrientation desiredPelvisOrientation = this.desiredPelvisOrientation.getFrameOrientationCopy();
      desiredCoPAndCMPControlModule.compute(capturePoint.getFramePoint2dCopy(), supportLeg, desiredICP.getFramePoint2dCopy(),
              desiredICPVelocity.getFrameVector2dCopy(), desiredPelvisOrientation, omega0.getDoubleValue(), momentum);
      FramePoint2d desiredCoP2d = new FramePoint2d(worldFrame);
      desiredCoPAndCMPControlModule.packCoP(desiredCoP2d);
      FramePoint2d desiredCMP2d = new FramePoint2d(worldFrame);
      desiredCoPAndCMPControlModule.packCMP(desiredCMP2d);

      desiredCoP2d.changeFrame(frame);
      GeometryTools.projectOntoPolygonAndCheckDistance(desiredCoP2d, bipedSupportPolygons.getSupportPolygonInMidFeetZUp(), 1e-10);    // fix numerical roundoff

      desiredCMP2d.changeFrame(frame);
      FrameVector2d desiredDeltaCMP = new FrameVector2d(desiredCMP2d);
      desiredDeltaCMP.sub(desiredCoP2d);

      this.fZ.set(computeFz());

      desiredPelvisOrientation.changeFrame(worldFrame);
      double desiredPelvisYaw = desiredPelvisOrientation.getYawPitchRoll()[0];
      FrameVector totalgroundReactionMoment = groundReactionMomentControlModule.determineGroundReactionMoment(momentum, desiredPelvisYaw);

      SpatialForceVector totalGroundReactionWrench = computeTotalGroundReactionWrench(desiredCoP2d, desiredCMP2d, totalgroundReactionMoment,
                                                        fZ.getDoubleValue(), omega0.getDoubleValue());
      totalGroundReactionWrench.changeFrame(centerOfMassFrame);

      desiredGroundReactionTorque.set(totalGroundReactionWrench.getAngularPartCopy());
      desiredGroundReactionForce.set(totalGroundReactionWrench.getLinearPartCopy());

      double coefficientOfFriction = 0.5;    // TODO
      double rotationalCoefficientOfFriction = 0.5;    // TODO

      groundReactionWrenchDistributor.reset();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody rigidBody = bipedFeet.get(robotSide).getRigidBody();
         PlaneContactState contactState = contactStates.get(rigidBody);

         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            groundReactionWrenchDistributor.addContact(contactState, coefficientOfFriction, rotationalCoefficientOfFriction);
         }
      }

      groundReactionWrenchDistributor.solve(totalGroundReactionWrench, upcomingSupportLeg.getEnumValue());
      List<Wrench> wrenches = new ArrayList<Wrench>();
      List<FramePoint2d> cops = new ArrayList<FramePoint2d>();

      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactablePlaneBody = bipedFeet.get(robotSide);
         RigidBody rigidBody = contactablePlaneBody.getRigidBody();
         PlaneContactState contactState = contactStates.get(rigidBody);
         List<FramePoint> footContactPoints = contactState.getContactPoints();

         if (footContactPoints.size() > 0)
         {
            FrameVector force = groundReactionWrenchDistributor.getForce(contactState);
            FramePoint2d cop = groundReactionWrenchDistributor.getCenterOfPressure(contactState);
            cops.add(cop);
            FramePoint cop3d = cop.toFramePoint();
            cop3d.changeFrame(worldFrame);
            centersOfPressure.get(contactablePlaneBody).set(cop3d);
            double normalTorque = groundReactionWrenchDistributor.getNormalTorque(contactState);
            Wrench groundReactionWrench = new Wrench(rigidBody.getBodyFixedFrame(), contactState.getPlaneFrame());
            CenterOfPressureTools.computeWrench(groundReactionWrench, force, cop, normalTorque);
            groundReactionWrench.changeFrame(rigidBody.getBodyFixedFrame());
            wrenches.add(groundReactionWrench);
            inverseDynamicsCalculator.setExternalWrench(rigidBody, groundReactionWrench);

            projectSpatialAccelerationIfNecessary(robotSide, rigidBody, footContactPoints, cop);
         }
         else
         {
            centersOfPressure.get(contactablePlaneBody).setToNaN();
         }
      }

      Wrench totalGroundReactionWrenchAfterProjection = TotalWrenchCalculator.computeTotalWrench(wrenches, totalGroundReactionWrench.getExpressedInFrame());
      admissibleDesiredGroundReactionTorque.set(totalGroundReactionWrenchAfterProjection.getAngularPartCopy());
      admissibleDesiredGroundReactionForce.set(totalGroundReactionWrenchAfterProjection.getLinearPartCopy());
      this.omega0.set(computeOmega0(cops, totalGroundReactionWrenchAfterProjection));

      SpatialForceVector desiredCentroidalMomentumRate = new SpatialForceVector(totalGroundReactionWrenchAfterProjection);
      desiredCentroidalMomentumRate.sub(gravitationalWrench);

      solver.compute();
      solver.solve(desiredCentroidalMomentumRate);

      inverseDynamicsCalculator.compute();
      if (processedOutputs != null)
         fullRobotModel.setTorques(processedOutputs);
      updateYoVariables();
   }


   private double computeOmega0(List<FramePoint2d> cop2ds, Wrench totalGroundReactionWrenchAfterProjection)
   {
      totalGroundReactionWrenchAfterProjection.changeFrame(centerOfMassFrame);
      double fz = totalGroundReactionWrenchAfterProjection.getLinearPartCopy().getZ();

      double deltaZ;
      if (cop2ds.size() == 1)
      {
         FramePoint cop = cop2ds.get(0).toFramePoint();
         cop.changeFrame(centerOfMassFrame);
         deltaZ = -cop.getZ();
      }
      else    // assume 2 CoPs
      {
         List<FramePoint> cops = new ArrayList<FramePoint>(cop2ds.size());
         for (FramePoint2d cop2d : cop2ds)
         {
            FramePoint cop = cop2d.toFramePoint();
            cop.changeFrame(copToCoPFrame.getParent());
            cops.add(cop);
         }

         copToCoPFrame.setOriginAndPositionToPointAt(cops.get(0), cops.get(1));
         copToCoPFrame.update();
         FramePoint2d pseudoCoP2d = new FramePoint2d(copToCoPFrame);
         centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(pseudoCoP2d, totalGroundReactionWrenchAfterProjection, copToCoPFrame);
         FramePoint pseudoCoP = pseudoCoP2d.toFramePoint();
         pseudoCoP.changeFrame(centerOfMassFrame);
         deltaZ = -pseudoCoP.getZ();
      }

      return Math.sqrt(fz / (totalMass * deltaZ));
   }

   private void projectSpatialAccelerationIfNecessary(RobotSide robotSide, RigidBody foot, List<FramePoint> footContactPoints, FramePoint2d footCoPOnSole2d)
   {
      footContactPoints = DesiredFootstepCalculatorTools.fixTwoPointsAndCopy(footContactPoints);    // TODO: terrible
      FrameConvexPolygon2d footPolygon = FrameConvexPolygon2d.constructByProjectionOntoXYPlane(footContactPoints, referenceFrames.getSoleFrame(robotSide));
      footCoPOnSole2d.changeFrame(footPolygon.getReferenceFrame());
      FrameLineSegment2d closestEdge = footPolygon.getClosestEdge(footCoPOnSole2d);
      double epsilonPointOnEdge = 1e-3;
      boolean isCoPOnEdge = closestEdge.distance(footCoPOnSole2d) < epsilonPointOnEdge;
      this.isCoPOnEdge.get(robotSide).set(isCoPOnEdge);

      MechanismGeometricJacobian jacobian = jacobians.get(robotSide).get(LimbName.LEG);

      // FIXME: nasty action at a distance
      SpatialAccelerationVector spatialAcceleration = solver.getSpatialAcceleration(jacobian);

      if (isCoPOnEdge)
      {
         spatialAccelerationProjectors.get(robotSide).projectAcceleration(spatialAcceleration, closestEdge);
      }
      else
      {
         // use zero angular acceleration and zero linear acceleration of origin
         spatialAcceleration.set(
             footPoseTwistAndSpatialAccelerationCalculators.get(robotSide).calculateDesiredEndEffectorSpatialAccelerationFromDesiredAccelerations(
                new FrameVector(worldFrame), new FrameVector(worldFrame), fullRobotModel.getElevator()));
         spatialAcceleration.changeFrameNoRelativeMotion(foot.getBodyFixedFrame());
         spatialAcceleration.changeBodyFrameNoRelativeAcceleration(foot.getBodyFixedFrame());
      }
   }

   private SpatialForceVector computeTotalGroundReactionWrench(FramePoint2d desiredCoP2d, FramePoint2d desiredCMP2d, FrameVector totalgroundReactionMoment,
           double fZ, double omega0)
   {
      FramePoint com = new FramePoint(centerOfMassFrame);
      double zCoP = com.getZ() - fZ / (totalMass * MathTools.square(omega0));    // FIXME: hack to resolve circularity

      FramePoint desiredCoP = desiredCoP2d.toFramePoint();
      desiredCoP.changeFrame(com.getReferenceFrame());
      desiredCoP.setZ(zCoP);

      FramePoint desiredCMP = desiredCMP2d.toFramePoint();
      desiredCMP.changeFrame(com.getReferenceFrame());
      desiredCMP.setZ(zCoP);

      FrameVector force = new FrameVector(com);
      force.sub(desiredCMP);
      force.scale(fZ / force.getZ());
      FrameVector momentArm = new FrameVector(desiredCoP);
      momentArm.sub(com);

      SpatialForceVector totalGroundReactionWrench = SpatialForceVector.createUsingArm(centerOfMassFrame, force.getVector(), momentArm.getVector());
      totalGroundReactionWrench.addAngularPart(totalgroundReactionMoment.getVector());

      return totalGroundReactionWrench;
   }

   public abstract void doMotionControl();

   private Momentum computeCentroidalMomentum()
   {
      Momentum momentum = new Momentum(centerOfMassFrame);
      momentumCalculator.computeAndPack(momentum);

      return momentum;
   }

   public void update()
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
         doPDControl(k, d, joint);
      }
   }

   protected void doPDControl(double k, double d, OneDoFJoint joint)
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

   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   public ContactState getContactState(RigidBody rigidBody)
   {
      return contactStates.get(rigidBody);
   }

   private double computeFz()
   {
      double fZ = totalMass * (gravityZ + desiredCoMHeightAcceleration.getDoubleValue());    // TODO: don't rely on DoubleYoVariable being set

      return fZ;
   }

   private void updateYoVariables()
   {
      SpatialAccelerationVector pelvisAcceleration = new SpatialAccelerationVector();
      fullRobotModel.getRootJoint().packDesiredJointAcceleration(pelvisAcceleration);
      desiredPelvisLinearAcceleration.set(pelvisAcceleration.getLinearPartCopy());
      desiredPelvisAngularAcceleration.set(pelvisAcceleration.getAngularPartCopy());

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

   private void updateBipedSupportPolygons(BipedSupportPolygons bipedSupportPolygons)
   {
      SideDependentList<List<FramePoint>> footContactPoints = new SideDependentList<List<FramePoint>>();
      for (RobotSide robotSide : RobotSide.values())
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         footContactPoints.put(robotSide, contactStates.get(foot).getContactPoints());
      }

      bipedSupportPolygons.update(footContactPoints);
   }
}
