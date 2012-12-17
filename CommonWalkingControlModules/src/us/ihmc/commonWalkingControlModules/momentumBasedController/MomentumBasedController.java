package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactState;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.DesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.LegStrengthCalculator;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.VirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.LegStrengthCalculatorTools;
import us.ihmc.commonWalkingControlModules.controlModules.NewGeometricVirtualToePointCalculator;
import us.ihmc.commonWalkingControlModules.controlModules.SacrificeDeltaCMPDesiredCoPAndCMPControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.TeeterTotterLegStrengthCalculator;
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
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameLine2d;
import us.ihmc.utilities.math.geometry.FrameLineSegment2d;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.GeometryTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RotationFunctions;
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
   private final VirtualToePointCalculator virtualToePointCalculator;
   private final LegStrengthCalculator legStrengthCalculator;

   private final ReferenceFrame midFeetZUp;
   private final double gravityZ;
   private final double totalMass;

   private final HighLevelHumanoidController highLevelHumanoidController;
   private final MomentumSolver solver;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FullRobotModel fullRobotModel;
   private final CommonWalkingReferenceFrames referenceFrames;
   private final TwistCalculator twistCalculator;

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

   private final DoubleYoVariable kAngularMomentumZ = new DoubleYoVariable("kAngularMomentumZ", registry);
   private final DoubleYoVariable kPelvisYaw = new DoubleYoVariable("kPelvisYaw", registry);
   private final HashMap<RevoluteJoint, DoubleYoVariable> desiredAccelerationYoVariables = new HashMap<RevoluteJoint, DoubleYoVariable>();

   private final SideDependentList<Double> lambdas = new SideDependentList<Double>();
   private final SideDependentList<DoubleYoVariable> kValues = new SideDependentList<DoubleYoVariable>();

   private final DoubleYoVariable fZ = new DoubleYoVariable("fZ", registry);

// private final BooleanYoVariable leftInSingularRegion = new BooleanYoVariable("leftInSingularRegion", registry);
// private final BooleanYoVariable rightInSingularRegion = new BooleanYoVariable("rightInSingularRegion", registry);
// private final SideDependentList<BooleanYoVariable> inSingularRegions = new SideDependentList<BooleanYoVariable>(leftInSingularRegion, rightInSingularRegion);

   private final SpatialForceVector gravitationalWrench;

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

      this.twistCalculator = twistCalculator;
      this.momentumCalculator = new MomentumCalculator(twistCalculator);
      this.highLevelHumanoidController = highLevelHumanoidController;

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

      midFeetZUp = referenceFrames.getMidFeetZUpFrame();

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

      virtualToePointCalculator = new NewGeometricVirtualToePointCalculator(referenceFrames, registry, dynamicGraphicObjectsListRegistry, 0.95);

      this.legStrengthCalculator = new TeeterTotterLegStrengthCalculator(registry);

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

      for (RobotSide robotSide : RobotSide.values())
      {
         kValues.put(robotSide, new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "KValue", registry));
      }

      kAngularMomentumZ.set(10.0);    // 50.0); // 10.0);
      kPelvisYaw.set(100.0);    // was 0.0 for M3 movie
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
      fixDesiredCoPNumericalRoundoff(desiredCoP, bipedSupportPolygons.getSupportPolygonInMidFeetZUp());

      desiredCMP.changeFrame(frame);
      FrameVector2d desiredDeltaCMP = new FrameVector2d(desiredCMP);
      desiredDeltaCMP.sub(desiredCoP);

      this.fZ.set(computeFz());

      SideDependentList<FramePoint2d> virtualToePoints = computeVirtualToePoints(supportLeg, desiredCoP);
      legStrengthCalculator.packLegStrengths(lambdas, virtualToePoints, desiredCoP);
      SideDependentList<FramePoint> virtualToePointsOnSole = projectOntoSole(virtualToePoints);

      FramePoint centerOfMass = new FramePoint(centerOfMassFrame);
      double omega0 = computeOmega0(centerOfMass, virtualToePointsOnSole);
      highLevelHumanoidController.setOmega0(omega0);

      double k1PlusK2 = MathTools.square(omega0) * totalMass;
      for (RobotSide robotSide : RobotSide.values())
      {
         double k = lambdas.get(robotSide) * k1PlusK2;
         kValues.get(robotSide).set(k);
      }

      FrameVector totalgroundReactionMoment = determineGroundReactionMoment(momentum);

      HashMap<RigidBody, Wrench> groundReactionWrenches = computeGroundReactionWrenches(frame, desiredDeltaCMP, virtualToePointsOnSole,
                                                             totalgroundReactionMoment);

      setGroundReactionWrenches(groundReactionWrenches, inverseDynamicsCalculator);

      Wrench totalGroundReactionWrench = computeTotalGroundReactionWrench(groundReactionWrenches);

      SpatialForceVector desiredCentroidalMomentumRate = new SpatialForceVector(totalGroundReactionWrench);
      desiredCentroidalMomentumRate.sub(gravitationalWrench);

//    for (RobotSide robotSide : RobotSide.values())
//    {
//     // TODO: get rid of this
//     boolean isSwingLeg = supportLeg == robotSide.getOppositeSide();
//     double maxKneeAngle = 0.4;
//     boolean leavingKneeLockRegion = optimizer.leavingSingularRegion(robotSide, LimbName.LEG)
//                                     && (fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE).getQ() < maxKneeAngle);    // TODO: hack
//     boolean trajectoryInitialized = highLevelHumanoidController.trajectoryInitialized(robotSide);
//
//     BooleanYoVariable inSingularRegion = inSingularRegions.get(robotSide);
//     inSingularRegion.set(optimizer.inSingularRegion(robotSide, LimbName.LEG));
//
//     // if ((supportLeg == robotSide.getOppositeSide()) &&!optimizer.inSingularRegion(robotSide) &&!stateMachine.trajectoryInitialized(robotSide))
//     if (isSwingLeg && (leavingKneeLockRegion || (!inSingularRegion.getBooleanValue() &&!trajectoryInitialized)))
//     {
//        SpatialAccelerationVector taskSpaceAcceleration = new SpatialAccelerationVector();
//
////      optimizer.computeMatchingNondegenerateTaskSpaceAcceleration(robotSide, LimbName.LEG, taskSpaceAcceleration);
//        highLevelHumanoidController.initializeTrajectory(robotSide, taskSpaceAcceleration);
//     }
//    }

      solver.reset();

      HashMap<InverseDynamicsJoint, DenseMatrix64F> jointAccelerations = highLevelHumanoidController.getJointAccelerations();
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
                  FramePoint footCoPOnSole = virtualToePointsOnSole.get(robotSide);
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

         solver.setDesiredSpatialAcceleration(jacobian, pair);
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         Wrench handWrench = highLevelHumanoidController.getExternalHandWrench(robotSide);
         inverseDynamicsCalculator.setExternalWrench(fullRobotModel.getHand(robotSide), handWrench);
      }

      solver.compute();


      // TODO
//    if (supportLeg != null)
//    {
////       solver.solve(desiredCentroidalMomentumRate);
//       
//       SpatialAccelerationVector desiredRootJointAcceleration = new SpatialAccelerationVector();
//       fullRobotModel.getRootJoint().packDesiredJointAcceleration(desiredRootJointAcceleration);
//       
//       DenseMatrix64F momentumSubspace = new DenseMatrix64F(SpatialForceVector.SIZE, 3);
////       momentumSubspace.set(2, 0, 1.0);
//       momentumSubspace.set(3, 0, 1.0);
//       momentumSubspace.set(4, 1, 1.0);
//       momentumSubspace.set(5, 2, 1.0);
//       
//       DenseMatrix64F momentumMultipliers = new DenseMatrix64F(3, 1);
////       Vector3d angularPart = desiredCentroidalMomentumRate.getAngularPartCopy();
////       momentumMultipliers.set(0, 0, angularPart.getZ());        
////       MatrixTools.setDenseMatrixFromTuple3d(momentumMultipliers, desiredCentroidalMomentumRate.getLinearPartCopy(), 1, 0);
//       MatrixTools.setDenseMatrixFromTuple3d(momentumMultipliers, desiredCentroidalMomentumRate.getLinearPartCopy(), 0, 0);
//       
//       DenseMatrix64F accelerationSubspace = new DenseMatrix64F(SpatialMotionVector.SIZE, 3);
//       accelerationSubspace.set(0, 0, 1.0);
//       accelerationSubspace.set(1, 1, 1.0);
//       accelerationSubspace.set(2, 2, 1.0);
//       
//       DenseMatrix64F accelerationMultipliers = new DenseMatrix64F(3, 1);
//       
//       Twist rootJointTwist = new Twist();
//       twistCalculator.packTwistOfBody(rootJointTwist, fullRobotModel.getRootJoint().getSuccessor());
//       ReferenceFrame pelvisFrame = fullRobotModel.getRootJoint().getFrameAfterJoint();
//       rootJointTwist.changeFrame(pelvisFrame);
//
//       FrameOrientation desiredPelvisOrientation = new FrameOrientation(worldFrame);
//       desiredPelvisOrientation.changeFrame(pelvisFrame);
//       FrameVector desiredPelvisAngularAcceleration = new FrameVector(pelvisFrame);
//       FrameVector desiredAngularVelocity = new FrameVector(pelvisFrame);
//       FrameVector currentAngularVelocity = new FrameVector(rootJointTwist.getExpressedInFrame(), rootJointTwist.getAngularPartCopy());
//       FrameVector feedForward = new FrameVector(pelvisFrame);
//       pelvisOrientationController.compute(desiredPelvisAngularAcceleration, desiredPelvisOrientation, desiredAngularVelocity, currentAngularVelocity, feedForward);
////       accelerationMultipliers.set(0, 0, desiredPelvisAngularAcceleration.getX());
////       accelerationMultipliers.set(1, 0, desiredPelvisAngularAcceleration.getY());
//       
//       MatrixTools.setDenseMatrixFromTuple3d(accelerationMultipliers, desiredPelvisAngularAcceleration.getVector(), 0, 0);
////       MatrixTools.setDenseMatrixFromTuple3d(accelerationMultipliers, desiredRootJointAcceleration.getAngularPartCopy(), 0, 0);
//
//       solver.solve(accelerationSubspace, accelerationMultipliers, momentumSubspace, momentumMultipliers);
//       
//       RigidBody foot = fullRobotModel.getFoot(supportLeg);
//       Wrench footWrench = new Wrench(foot.getBodyFixedFrame(), centerOfMassFrame);
//       solver.getRateOfChangeOfMomentum(footWrench);
//       footWrench.add(gravitationalWrench);
//       footWrench.changeFrame(foot.getBodyFixedFrame());
//
//       inverseDynamicsCalculator.setExternalWrench(foot, footWrench);
//    }
//    else
//    {
//       solver.solve(desiredCentroidalMomentumRate);
//    }
      solver.solve(desiredCentroidalMomentumRate);
   }

   private Wrench computeTotalGroundReactionWrench(HashMap<RigidBody, Wrench> groundReactionWrenches)
   {
      Wrench totalGroundReactionWrench = new Wrench(centerOfMassFrame, centerOfMassFrame);

      Wrench temporaryWrench = new Wrench();
      for (RigidBody rigidBody : groundReactionWrenches.keySet())
      {
         temporaryWrench.set(groundReactionWrenches.get(rigidBody));
         temporaryWrench.changeFrame(centerOfMassFrame);
         temporaryWrench.changeBodyFrameAttachedToSameBody(centerOfMassFrame);
         totalGroundReactionWrench.add(temporaryWrench);
      }

      return totalGroundReactionWrench;
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

   // SPECIFIC
   private HashMap<RigidBody, Wrench> computeGroundReactionWrenches(ReferenceFrame frame, FrameVector2d desiredDeltaCMP,
           SideDependentList<FramePoint> virtualToePointsOnSole, FrameVector totalgroundReactionMoment)
   {
      SideDependentList<Double> momentWeightings = new SideDependentList<Double>();
      for (RobotSide robotSide : RobotSide.values())
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ContactState contactState = highLevelHumanoidController.getContactState(foot);
         FramePoint virtualToePoint = virtualToePointsOnSole.get(robotSide);
         virtualToePoint.changeFrame(contactState.getBodyFrame());
         List<FramePoint> contactPoints = contactState.getContactPoints();
         double momentWeighting;
         if (contactPoints.size() > 0)
         {
            double minDistance = GeometryTools.minimumDistance(virtualToePoint, contactPoints);
            momentWeighting = minDistance * lambdas.get(robotSide);
         }
         else
            momentWeighting = 0.0;

         momentWeightings.put(robotSide, momentWeighting);
      }

      LegStrengthCalculatorTools.normalize(momentWeightings);

      HashMap<RigidBody, Wrench> groundReactionWrenches = new HashMap<RigidBody, Wrench>();
      for (RobotSide robotSide : RobotSide.values())
      {
         FramePoint groundReactionForceTerminalPoint = new FramePoint(centerOfMassFrame);
         groundReactionForceTerminalPoint.changeFrame(frame);
         desiredDeltaCMP.changeFrame(frame);
         groundReactionForceTerminalPoint.setX(groundReactionForceTerminalPoint.getX() - desiredDeltaCMP.getX());
         groundReactionForceTerminalPoint.setY(groundReactionForceTerminalPoint.getY() - desiredDeltaCMP.getY());
         FrameVector groundReactionForce = new FrameVector(groundReactionForceTerminalPoint);

         FramePoint virtualToePoint = virtualToePointsOnSole.get(robotSide);
         virtualToePoint.changeFrame(frame);
         groundReactionForce.sub(virtualToePoint);
         groundReactionForce.scale(kValues.get(robotSide).getDoubleValue());

         FrameVector groundReactionMoment = new FrameVector(totalgroundReactionMoment);

         // TODO: base on contact situation.
         groundReactionMoment.scale(lambdas.get(robotSide));    // momentWeightings.get(robotSide));


         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ReferenceFrame footCoMFrame = foot.getBodyFixedFrame();
         FrameVector torque = new FrameVector(frame);
         torque.cross(virtualToePoint, groundReactionForce);
         Wrench groundReactionWrench = new Wrench(footCoMFrame, frame, groundReactionForce.getVector(), torque.getVector());
         groundReactionWrench.addAngularPart(groundReactionMoment.getVector());
         groundReactionWrenches.put(foot, groundReactionWrench);
      }

      return groundReactionWrenches;
   }

   // SPECIFIC
   private double computeOmega0(FramePoint centerOfMass, SideDependentList<FramePoint> virtualToePointsOnSole)
   {
      FrameLine2d vtpToVTPLine = new FrameLine2d(virtualToePointsOnSole.get(RobotSide.LEFT).toFramePoint2d(),
                                    virtualToePointsOnSole.get(RobotSide.RIGHT).toFramePoint2d());

      FramePoint r1 = virtualToePointsOnSole.get(RobotSide.LEFT);
      FramePoint2d r12d = r1.toFramePoint2d();
      vtpToVTPLine.orthogonalProjection(r12d);    // not sure if necessary.
      double x1 = vtpToVTPLine.getParameterGivenPointEpsilon(r12d, 1e-12);
      double z1 = r1.getZ();

      FramePoint r2 = virtualToePointsOnSole.get(RobotSide.RIGHT);
      FramePoint2d r22d = r2.toFramePoint2d();
      vtpToVTPLine.orthogonalProjection(r22d);    // not sure if necessary.
      double x2 = vtpToVTPLine.getParameterGivenPointEpsilon(r22d, 1e-12);
      double z2 = r2.getZ();
      centerOfMass.changeFrame(worldFrame);
      double z = centerOfMass.getZ();

      double omega0Squared = (fZ.getDoubleValue() * (x1 - x2))
                             / (totalMass
                                * (x1 * (z - lambdas.get(RobotSide.LEFT) * z1 + (-1 + lambdas.get(RobotSide.LEFT)) * z2)
                                   + x2 * (-z + z1 - lambdas.get(RobotSide.RIGHT) * z1 + lambdas.get(RobotSide.RIGHT) * z2)));

      if (omega0Squared <= 0.0)
         throw new RuntimeException("omega0Squared <= 0.0. omega0Squared = " + omega0Squared);

      double omega0 = Math.sqrt(omega0Squared);

      return omega0;
   }

   // SPECIFIC
   private SideDependentList<FramePoint2d> computeVirtualToePoints(RobotSide supportLeg, FramePoint2d desiredCoP)
   {
      SideDependentList<FramePoint2d> virtualToePoints = new SideDependentList<FramePoint2d>();
      if (supportLeg == null)
      {
         virtualToePointCalculator.packVirtualToePoints(virtualToePoints, bipedSupportPolygons, desiredCoP,
                 highLevelHumanoidController.getUpcomingSupportLeg());
      }
      else
      {
         virtualToePointCalculator.hideVisualizationGraphics();
         FrameConvexPolygon2d footPolygonInAnkleZUp = bipedSupportPolygons.getFootPolygonInAnkleZUp(supportLeg);
         fixDesiredCoPNumericalRoundoff(desiredCoP, footPolygonInAnkleZUp);

         virtualToePoints.put(supportLeg, desiredCoP);
         virtualToePoints.put(supportLeg.getOppositeSide(), new FramePoint2d(desiredCoP.getReferenceFrame()));
      }

      return virtualToePoints;
   }


   private SideDependentList<FramePoint> projectOntoSole(SideDependentList<FramePoint2d> virtualToePoints)
   {
      SideDependentList<FramePoint> virtualToePointsOnSole = new SideDependentList<FramePoint>();
      for (RobotSide robotSide : RobotSide.values())
      {
         virtualToePoints.get(robotSide).changeFrame(worldFrame);
         FramePoint virtualToePoint = virtualToePoints.get(robotSide).toFramePoint();
         virtualToePoint = projectPointOntoSole(robotSide, virtualToePoint);
         virtualToePoint.changeFrame(worldFrame);
         virtualToePointsOnSole.put(robotSide, virtualToePoint);
      }

      return virtualToePointsOnSole;
   }

   private void fixDesiredCoPNumericalRoundoff(FramePoint2d desiredCoP, FrameConvexPolygon2d polygon)
   {
      ReferenceFrame originalReferenceFrame = desiredCoP.getReferenceFrame();
      double epsilon = 1e-10;
      desiredCoP.changeFrame(polygon.getReferenceFrame());
      FramePoint2d originalDesiredCoP = new FramePoint2d(desiredCoP);
      polygon.orthogonalProjection(desiredCoP);
      double distance = originalDesiredCoP.distance(desiredCoP);
      if (distance > epsilon)
         throw new RuntimeException("desired CoP outside polygon by " + distance);
      desiredCoP.changeFrame(originalReferenceFrame);
   }

   private double computeFz()
   {
      double fZ = totalMass * (gravityZ + highLevelHumanoidController.getDesiredCoMHeightAcceleration());

      return fZ;
   }

   private FramePoint projectPointOntoSole(RobotSide robotSide, FramePoint virtualToePoint)
   {
      ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
      FramePoint pointOnPlane = new FramePoint(soleFrame);
      FrameVector planeNormal = new FrameVector(soleFrame, 0.0, 0.0, 1.0);
      FramePoint lineStart = virtualToePoint.changeFrameCopy(soleFrame);
      FramePoint lineEnd = new FramePoint(virtualToePoint);    // start at VTP
      lineEnd.setZ(lineEnd.getZ() - 1.0);    // down an arbitrary amount in the frame in which the VTP is expressed
      lineEnd.changeFrame(soleFrame);    // then change frame to sole frame

      return GeometryTools.getIntersectionBetweenLineAndPlane(pointOnPlane, planeNormal, lineStart, lineEnd);
   }

   private FrameVector determineGroundReactionMoment(Momentum momentum)
   {
      FrameVector ret = new FrameVector(midFeetZUp);
      FrameVector angularMomentum = new FrameVector(momentum.getExpressedInFrame(), momentum.getAngularPartCopy());
      angularMomentum.changeFrame(midFeetZUp);

      Matrix3d pelvisToWorld = new Matrix3d();
      fullRobotModel.getPelvis().getBodyFixedFrame().getTransformToDesiredFrame(worldFrame).get(pelvisToWorld);
      double pelvisYaw = RotationFunctions.getYaw(pelvisToWorld);
      double desiredPelvisYaw = highLevelHumanoidController.getDesiredPelvisOrientation().getYawPitchRoll()[0];

      double error = AngleTools.computeAngleDifferenceMinusPiToPi(desiredPelvisYaw, pelvisYaw);
      ret.setZ(-kAngularMomentumZ.getDoubleValue() * angularMomentum.getZ() + kPelvisYaw.getDoubleValue() * error);

      return ret;
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
