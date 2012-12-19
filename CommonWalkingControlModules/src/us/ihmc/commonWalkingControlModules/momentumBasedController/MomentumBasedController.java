package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactState;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionMomentControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.GroundReactionWrenchDistributor;
import us.ihmc.commonWalkingControlModules.controlModules.SacrificeDeltaCMPDesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.CapturabilityBasedDesiredCoPVisualizer;
import us.ihmc.commonWalkingControlModules.controlModules.velocityViaCoP.SimpleDesiredCenterOfPressureFilter;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.kinematics.SpatialAccelerationProjector;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.referenceFrames.CommonWalkingReferenceFrames;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.MechanismGeometricJacobian;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.DampedLeastSquaresSolver;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.EndEffectorPoseTwistAndSpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.Momentum;
import us.ihmc.utilities.screwTheory.MomentumCalculator;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TotalWrenchCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.AxisAngleOrientationController;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicPosition;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class MomentumBasedController implements RobotController
{
   private static final long serialVersionUID = -744968203951905486L;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final MomentumCalculator momentumCalculator;

   private final ProcessedOutputsInterface processedOutputs;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final BipedSupportPolygons bipedSupportPolygons;

   private final double gravityZ;
   private final double totalMass;

   private final HighLevelHumanoidController highLevelHumanoidController;
   private final MomentumSolver solver;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FullRobotModel fullRobotModel;
   private final CommonWalkingReferenceFrames referenceFrames;

   private final SideDependentList<EnumMap<LimbName, SpatialAccelerationVector>> desiredEndEffectorAccelerationsInWorld =
      SideDependentList.createListOfEnumMaps(LimbName.class);
   private final SideDependentList<EndEffectorPoseTwistAndSpatialAccelerationCalculator> footPoseTwistAndSpatialAccelerationCalculators =
      new SideDependentList<EndEffectorPoseTwistAndSpatialAccelerationCalculator>();
   private final SideDependentList<SpatialAccelerationProjector> spatialAccelerationProjectors = new SideDependentList<SpatialAccelerationProjector>();
   private final SideDependentList<BooleanYoVariable> isCoPOnEdge = new SideDependentList<BooleanYoVariable>();
   private final SideDependentList<YoFramePoint> desiredFootPositionsInWorld = new SideDependentList<YoFramePoint>();

   private final DesiredCoPAndCMPControlModule desiredCoPAndCMPControlModule;

   private final YoFrameVector desiredPelvisLinearAcceleration;
   private final YoFrameVector desiredPelvisAngularAcceleration;
   private final YoFrameVector desiredPelvisForce;
   private final YoFrameVector desiredPelvisTorque;
   private final ReferenceFrame centerOfMassFrame;

   private final AxisAngleOrientationController pelvisOrientationController;


   private final HashMap<RevoluteJoint, DoubleYoVariable> desiredAccelerationYoVariables = new HashMap<RevoluteJoint, DoubleYoVariable>();


   private final DoubleYoVariable fZ = new DoubleYoVariable("fZ", registry);

// private final BooleanYoVariable leftInSingularRegion = new BooleanYoVariable("leftInSingularRegion", registry);
// private final BooleanYoVariable rightInSingularRegion = new BooleanYoVariable("rightInSingularRegion", registry);
// private final SideDependentList<BooleanYoVariable> inSingularRegions = new SideDependentList<BooleanYoVariable>(leftInSingularRegion, rightInSingularRegion);

   private final SpatialForceVector gravitationalWrench;

   private final GroundReactionMomentControlModule groundReactionMomentControlModule;
   private final GroundReactionWrenchDistributor groundReactionWrenchDistributor;

   public MomentumBasedController(FullRobotModel fullRobotModel, ProcessedOutputsInterface processedOutputs, double gravityZ,
                                  CommonWalkingReferenceFrames referenceFrames, TwistCalculator twistCalculator, double controlDT,
                                  DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, BipedSupportPolygons bipedSupportPolygons,
                                  HighLevelHumanoidController highLevelHumanoidController)
   {
      MathTools.checkIfInRange(gravityZ, 0.0, Double.POSITIVE_INFINITY);

      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = referenceFrames;
      this.processedOutputs = processedOutputs;
      this.gravityZ = gravityZ;

      this.momentumCalculator = new MomentumCalculator(twistCalculator);
      this.highLevelHumanoidController = highLevelHumanoidController;
      this.groundReactionMomentControlModule = new GroundReactionMomentControlModule(fullRobotModel.getPelvis().getBodyFixedFrame(), registry);
      this.groundReactionMomentControlModule.setGains(10.0, 100.0);    // kPelvisYaw was 0.0 for M3 movie
      this.groundReactionWrenchDistributor = new GroundReactionWrenchDistributor(referenceFrames, fullRobotModel, dynamicGraphicObjectsListRegistry, registry);

      RigidBody elevator = fullRobotModel.getElevator();
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravityZ);

      this.bipedSupportPolygons = bipedSupportPolygons;
      this.pelvisOrientationController = new AxisAngleOrientationController("pelvis", fullRobotModel.getRootJoint().getFrameAfterJoint(), registry);
      pelvisOrientationController.setProportionalGains(10.0, 10.0, 10.0);
      pelvisOrientationController.setDerivativeGains(2.0, 2.0, 2.0);

      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();
      for (RobotSide robotSide : RobotSide.values())
      {
         for (LimbName limbName : LimbName.values())
         {
            ReferenceFrame endEffectorFrame = fullRobotModel.getEndEffectorFrame(robotSide, limbName);
            desiredEndEffectorAccelerationsInWorld.get(robotSide).put(limbName,
                    new SpatialAccelerationVector(endEffectorFrame, elevatorFrame, endEffectorFrame));
         }

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
            fullRobotModel.getPelvis().getBodyFixedFrame(), registry);
      desiredCoPAndCMPControlModule.setGains(3e-2, 1.0, 1.5);
      this.desiredCoPAndCMPControlModule = desiredCoPAndCMPControlModule;


      // this.desiredCoPAndCMPControlModule = new SacrificeCMPCoPAndCMPControlModule(desiredCapturePointToDesiredCoPControlModule,
      // desiredCapturePointToDesiredCoPControlModule, desiredCenterOfPressureFilter, visualizer, bipedSupportPolygons, processedSensors, referenceFrames, registry).setGains(3e-2, 1.0);

      centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
      this.totalMass = TotalMassCalculator.computeSubTreeMass(elevator);

      gravitationalWrench = new SpatialForceVector(centerOfMassFrame, new Vector3d(0.0, 0.0, totalMass * gravityZ), new Vector3d());

      DampedLeastSquaresSolver jacobianSolver = new DampedLeastSquaresSolver(SpatialMotionVector.SIZE);
      jacobianSolver.setAlpha(5e-2);
      solver = new MomentumSolver(fullRobotModel.getRootJoint(), elevator, centerOfMassFrame, twistCalculator, jacobianSolver, controlDT, registry);

      this.desiredPelvisLinearAcceleration = new YoFrameVector("desiredPelvisLinearAcceleration", "", referenceFrames.getPelvisFrame(), registry);
      this.desiredPelvisAngularAcceleration = new YoFrameVector("desiredPelvisAngularAcceleration", "", referenceFrames.getPelvisFrame(), registry);
      this.desiredPelvisForce = new YoFrameVector("desiredPelvisForce", "", centerOfMassFrame, registry);
      this.desiredPelvisTorque = new YoFrameVector("desiredPelvisTorque", "", centerOfMassFrame, registry);

      for (RobotSide robotSide : RobotSide.values())
      {
         String swingfootPositionName = "desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "SwingFootPositionInWorld";
         YoFramePoint desiredSwingFootPosition = new YoFramePoint(swingfootPositionName, "", worldFrame, registry);
         desiredFootPositionsInWorld.put(robotSide, desiredSwingFootPosition);    // TODO: why is this here?
         DynamicGraphicPosition desiredSwingFootPositionViz = new DynamicGraphicPosition(swingfootPositionName, desiredSwingFootPosition, 0.03,
                                                                 YoAppearance.Orange());
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObject(name, desiredSwingFootPositionViz);
      }

      InverseDynamicsJoint[] joints = ScrewTools.computeJointsInOrder(elevator);
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof RevoluteJoint)
         {
            desiredAccelerationYoVariables.put((RevoluteJoint) joint, new DoubleYoVariable(joint.getName() + "qdd_d", registry));
         }
      }
   }

   public void initialize()
   {
      solver.initialize();
      highLevelHumanoidController.initialize();
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

   public void doControl()
   {
      Momentum momentum = computeCentroidalMomentum();

      updateBipedSupportPolygons(bipedSupportPolygons);

      highLevelHumanoidController.update();
      highLevelHumanoidController.doControl();

      doMomentumBasedControl(highLevelHumanoidController.getInstantaneousCapturePoint(), momentum);
      inverseDynamicsCalculator.compute();
      fullRobotModel.setTorques(processedOutputs);
      updateYoVariables();
   }

   public Momentum computeCentroidalMomentum()
   {
      Momentum momentum = new Momentum(centerOfMassFrame);
      momentumCalculator.computeAndPack(momentum);

      return momentum;
   }

   /**
    * @param centerOfMassVelocity
    * @param capturePoint
    * @param momentum
    */
   private void doMomentumBasedControl(FramePoint2d capturePoint, Momentum momentum)
   {
      ReferenceFrame frame = worldFrame;
      RobotSide supportLeg = highLevelHumanoidController.getSupportLeg();
      FramePoint2d desiredCapturePoint = new FramePoint2d(worldFrame);
      highLevelHumanoidController.packDesiredICP(desiredCapturePoint);
      FrameVector2d desiredCapturePointVelocity = new FrameVector2d(worldFrame);
      highLevelHumanoidController.packDesiredICPVelocity(desiredCapturePointVelocity);

      desiredCoPAndCMPControlModule.compute(capturePoint, supportLeg, desiredCapturePoint, desiredCapturePointVelocity,
              highLevelHumanoidController.getDesiredPelvisOrientation(), highLevelHumanoidController.getOmega0(), momentum);
      FramePoint2d desiredCoP = new FramePoint2d(worldFrame);
      desiredCoPAndCMPControlModule.packCoP(desiredCoP);
      FramePoint2d desiredCMP = new FramePoint2d(worldFrame);
      desiredCoPAndCMPControlModule.packCMP(desiredCMP);
      highLevelHumanoidController.setPreviousCoP(desiredCoP);

      desiredCoP.changeFrame(frame);
      GeometryTools.projectOntoPolygonAndCheckDistance(desiredCoP, bipedSupportPolygons.getSupportPolygonInMidFeetZUp(), 1e-10);    // fix numerical roundoff

      desiredCMP.changeFrame(frame);
      FrameVector2d desiredDeltaCMP = new FrameVector2d(desiredCMP);
      desiredDeltaCMP.sub(desiredCoP);

      this.fZ.set(computeFz());
      double desiredPelvisYaw = highLevelHumanoidController.getDesiredPelvisOrientation().getYawPitchRoll()[0];
      FrameVector totalgroundReactionMoment = groundReactionMomentControlModule.determineGroundReactionMoment(momentum, desiredPelvisYaw);

      SideDependentList<ContactState> footContactStates = new SideDependentList<ContactState>();
      for (RobotSide robotSide : RobotSide.values())
      {
         footContactStates.put(robotSide, highLevelHumanoidController.getContactState(fullRobotModel.getFoot(robotSide)));
      }

      groundReactionWrenchDistributor.distributeGroundReactionWrench(desiredCoP, desiredDeltaCMP, fZ.getDoubleValue(), totalgroundReactionMoment,
              footContactStates, bipedSupportPolygons, highLevelHumanoidController.getUpcomingSupportLeg());

      HashMap<RigidBody, Wrench> groundReactionWrenches = groundReactionWrenchDistributor.getGroundReactionWrenches();

      setGroundReactionWrenches(groundReactionWrenches, inverseDynamicsCalculator);

      Wrench totalGroundReactionWrench = TotalWrenchCalculator.computeTotalWrench(groundReactionWrenches.values(), centerOfMassFrame);

      SpatialForceVector desiredCentroidalMomentumRate = new SpatialForceVector(totalGroundReactionWrench);
      desiredCentroidalMomentumRate.sub(gravitationalWrench);

      solver.reset();

      Map<InverseDynamicsJoint, DenseMatrix64F> jointAccelerations = highLevelHumanoidController.getJointAccelerations();
      for (InverseDynamicsJoint joint : jointAccelerations.keySet())
      {
         solver.setDesiredJointAcceleration(joint, jointAccelerations.get(joint));
      }

      Map<MechanismGeometricJacobian, Pair<SpatialAccelerationVector, DenseMatrix64F>> taskAccelerations = highLevelHumanoidController.getTaskAccelerations();

      for (MechanismGeometricJacobian jacobian : taskAccelerations.keySet())
      {
         Pair<SpatialAccelerationVector, DenseMatrix64F> pair = taskAccelerations.get(jacobian);
         SpatialAccelerationVector spatialAcceleration = pair.first();

         // TODO: get rid of this:
         for (RobotSide robotSide : RobotSide.values())
         {
            RigidBody foot = fullRobotModel.getFoot(robotSide);
            if (jacobian.getEndEffectorFrame() == foot.getBodyFixedFrame())
            {
               List<FramePoint> footContactPoints = highLevelHumanoidController.getContactPoints(foot);

               if (footContactPoints.size() > 0)
               {
                  footContactPoints = DesiredFootstepCalculatorTools.fixTwoPointsAndCopy(footContactPoints);    // TODO: terrible
                  FrameConvexPolygon2d footPolygon = FrameConvexPolygon2d.constructByProjectionOntoXYPlane(footContactPoints,
                                                        referenceFrames.getSoleFrame(robotSide));
                  FramePoint footCoPOnSole = groundReactionWrenchDistributor.getVirtualToePointsOnSole().get(robotSide);
                  footCoPOnSole.changeFrame(footPolygon.getReferenceFrame());
                  FramePoint2d footCoPOnSole2d = footCoPOnSole.toFramePoint2d();
                  FrameLineSegment2d closestEdge = footPolygon.getClosestEdge(footCoPOnSole2d);
                  double epsilonPointOnEdge = 1e-3;
                  boolean isCoPOnEdge = closestEdge.distance(footCoPOnSole2d) < epsilonPointOnEdge;
                  this.isCoPOnEdge.get(robotSide).set(isCoPOnEdge);

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
            }
         }

         solver.setDesiredSpatialAcceleration(jacobian, pair.first(), pair.second());
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         Wrench handWrench = highLevelHumanoidController.getExternalHandWrench(robotSide);
         inverseDynamicsCalculator.setExternalWrench(fullRobotModel.getHand(robotSide), handWrench);
      }

      solver.compute();

      solver.solve(desiredCentroidalMomentumRate);
   }

   private void setGroundReactionWrenches(HashMap<RigidBody, Wrench> groundReactionWrenches, InverseDynamicsCalculator inverseDynamicsCalculator)
   {
      for (RigidBody rigidBody : groundReactionWrenches.keySet())
      {
         Wrench groundReactionWrench = groundReactionWrenches.get(rigidBody);
         groundReactionWrench.changeFrame(rigidBody.getBodyFixedFrame());
         inverseDynamicsCalculator.setExternalWrench(rigidBody, groundReactionWrench);
      }
   }

   private double computeFz()
   {
      double fZ = totalMass * (gravityZ + highLevelHumanoidController.getDesiredCoMHeightAcceleration());

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
      pelvisJointWrench.changeFrame(centerOfMassFrame);
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
         footContactPoints.put(robotSide, highLevelHumanoidController.getContactPoints(foot));
      }

      bipedSupportPolygons.update(footContactPoints);
   }
}
