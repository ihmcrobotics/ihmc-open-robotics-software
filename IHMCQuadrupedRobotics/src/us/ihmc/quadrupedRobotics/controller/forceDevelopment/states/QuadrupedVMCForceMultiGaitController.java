package us.ihmc.quadrupedRobotics.controller.forceDevelopment.states;

import java.awt.Color;
import java.util.ArrayList;
import java.util.EnumMap;

import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.YoQuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.planning.gait.QuadrupedGaitCycle;
import us.ihmc.quadrupedRobotics.planning.gait.QuadrupedSupportConfiguration;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.math.frames.YoTwist;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.FootSwitchInterface;

public class QuadrupedVMCForceMultiGaitController implements QuadrupedController
{
   // Constants
   private static final double GRAVITY = 9.81;
   private static final double SIMULATION_TO_ROBOT_MODEL_Z_DIFFERNCE = 0.08;
   private static final double ESTIMATED_MASS = 62.493; // TODO PDControl this when z-vel=0
   private static final double ESTIMATED_ROTATIONAL_INERTIA = 5.0; // TODO PDControl this when z-vel=0
   private static final int ORDER_OF_SWING_Z_POLYNOMIAL = 4;
   private static final double ICP_BOUNDS_MARGIN = 0.1;

   // Controller Options
   public static final boolean USE_COPX_AND_COPY = true;
   public static final boolean CREATE_VISUALIZATIONS = true;
   public static final double VISUALIZATION_BALLS_PER_SWING = 20.0;

   // Basis Vectors
   private static final double COEFFICIENT_OF_FRICTION = 0.7;
   private static final int NUMBER_OF_BASIS_VECTORS = 4;
   private static final double ANGLE_BETWEEN_BASIS_VECTORS = 2.0 * Math.PI / (double) NUMBER_OF_BASIS_VECTORS;
   private static final double BASIS_VECTOR_HEIGHT = 1.0;

   // Inherited Variables
   private final double dt;
   private double initialStanceHeight;
   private final DoubleYoVariable yoTime;
   private final YoVariableRegistry registry = new YoVariableRegistry("TrotWalkController");
   private final QuadrupedReferenceFrames referenceFrames;
   private final FullRobotModel fullRobotModel;
   private final QuadrantDependentList<ArrayList<OneDoFJoint>> oneDofJoints = new QuadrantDependentList<>();
   private boolean hasInitializedInheritedYoVariables = false;
   private DoubleYoVariable q_z;

   // PD Controllers
   private final DoubleYoVariable kp_x = new DoubleYoVariable("kp_x", registry);
   private final DoubleYoVariable kp_y = new DoubleYoVariable("kp_y", registry);
   private final DoubleYoVariable kp_z = new DoubleYoVariable("kp_z", registry);
   private final DoubleYoVariable kp_roll = new DoubleYoVariable("kp_roll", registry);
   private final DoubleYoVariable kp_pitch = new DoubleYoVariable("kp_pitch", registry);
   private final DoubleYoVariable kp_yaw = new DoubleYoVariable("kp_yaw", registry);
   private final DoubleYoVariable kp_icp = new DoubleYoVariable("kp_icp", registry);
   private final DoubleYoVariable kd_x = new DoubleYoVariable("kd_x", registry);
   private final DoubleYoVariable kd_y = new DoubleYoVariable("kd_y", registry);
   private final DoubleYoVariable kd_z = new DoubleYoVariable("kd_z", registry);
   private final DoubleYoVariable kd_roll = new DoubleYoVariable("kd_roll", registry);
   private final DoubleYoVariable kd_pitch = new DoubleYoVariable("kd_pitch", registry);
   private final DoubleYoVariable kd_yaw = new DoubleYoVariable("kd_yaw", registry);
   private final DoubleYoVariable kd_icp = new DoubleYoVariable("kd_icp", registry);

   private final QuadrantDependentList<YoFramePoint> footPositions = new QuadrantDependentList<YoFramePoint>();
   private final QuadrantDependentList<YoFramePoint> shoulderPositions = new QuadrantDependentList<YoFramePoint>();
   private final QuadrantDependentList<YoFramePoint> shoulderPositionsOnGroundForVis = new QuadrantDependentList<YoFramePoint>();
   private final QuadrantDependentList<YoFrameVector> footVelocities = new QuadrantDependentList<YoFrameVector>();
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final YoFramePoint centerOfMass = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector centerOfMassVelocity = new YoFrameVector("centerOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfMassXYProjection = new YoFramePoint("centerOfMassXYProjection", ReferenceFrame.getWorldFrame(), registry);

   // Balancing
   private final YoFramePoint2d icp = new YoFramePoint2d("icp", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector2d icpVelocity = new YoFrameVector2d("icpVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d desiredICP = new YoFramePoint2d("desiredICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector2d desiredICPVelocity = new YoFrameVector2d("desiredICPVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector2d desiredRobotVelocity = new YoFrameVector2d("desiredRobotVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final DoubleYoVariable desiredRobotAngularVelocityZ = new DoubleYoVariable("desiredRobotAngularVelocityZ", registry);
   private final YoFramePoint centerOfPressure = new YoFramePoint("centerOfPressure", ReferenceFrame.getWorldFrame(), registry);
   /** ICP Action vector is a vector opposite and equal to the vector from the ICP to the center of pressure */
   private final YoFrameVector2d icpActionVector = new YoFrameVector2d("icpActionVector", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d desiredCenterOfPressure = new YoFramePoint2d("desiredCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d snappedDesiredCenterOfPressure = new YoFramePoint2d("snappedDesiredCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final QuadrantDependentList<YoFrameVector[]> basisForceVectors = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> basisCoPVectors = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector[]> basisTorqueVectors = new QuadrantDependentList<>();
   private final QuadrantDependentList<double[]> rhoScalars = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> footToCoMVectors = new QuadrantDependentList<>();
   private final DenseMatrix64F bodyWrenchMatrix = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F basisMatrix = new DenseMatrix64F(6, 16);
   private final DenseMatrix64F rhoMatrix = new DenseMatrix64F(16, 1);
   private final SolvePseudoInverseSvd solver = new SolvePseudoInverseSvd();
   private final QuadrantDependentList<YoFrameVector> vmcFootForces = new QuadrantDependentList<>();
   private final YoFramePose bodyPoseWorld = new YoFramePose("body", ReferenceFrame.getWorldFrame(), registry);
   private final PoseReferenceFrame bodyPoseReferenceFrame = new PoseReferenceFrame("bodyPoseReferenceFrame", ReferenceFrame.getWorldFrame());
   private final YoTwist bodyTwist;
   private final YoFramePose stancePose = new YoFramePose("stance", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint stancePoseHigherForVis = new YoFramePoint("stancePoseHigherForVis", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredStancePoseHigherForVis = new YoFramePoint("desiredStancePoseHigherForVis", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose desiredStancePose = new YoFramePose("desiredStance", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose desiredStancePoseOffset = new YoFramePose("desiredStanceOffset", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredBodyForceOriginForVis = new YoFramePoint("desiredBodyForceOriginForVis", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredBodyTorqueOriginForVis = new YoFramePoint("desiredBodyTorqueOriginForVis", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector bodyLinearAcceleration = new YoFrameVector("bodyLinearAcceleration", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector bodyAngularAcceleration = new YoFrameVector("bodyAngularAcceleration", ReferenceFrame.getWorldFrame(), registry);
   private final YoWrench desiredBodyWrench;
   private final YoTwist desiredBodyTwist;

   // Jacobian
   private final FramePoint jointPosition = new FramePoint();
   private final FrameVector jointToFootVector = new FrameVector();
   private final FrameVector vmcRequestedTorqueFromJoint = new FrameVector();
   private final FrameVector jointAxis = new FrameVector();

   // Walking
   private final EnumYoVariable<QuadrupedGaitCycle> desiredGait = new EnumYoVariable<>("desiredGait", registry, QuadrupedGaitCycle.class);
   private final EnumYoVariable<QuadrupedGaitCycle> nextGait = new EnumYoVariable<>("nextGait", registry, QuadrupedGaitCycle.class);
   private final EnumYoVariable<QuadrupedGaitCycle> currentGait = new EnumYoVariable<>("currentGait", registry, QuadrupedGaitCycle.class);
   private final QuadrantDependentList<YoPolynomial> swingZTrajectories = new QuadrantDependentList<>();
   private final QuadrantDependentList<DoubleYoVariable> swingInitialZHeights = new QuadrantDependentList<>();
   private final QuadrantDependentList<DoubleYoVariable> gaitCompletionAtStartOfSwing = new QuadrantDependentList<>();
   private final DoubleYoVariable currentGaitPhaseStartTime = new DoubleYoVariable("currentPhaseStartTime", registry);
   private final DoubleYoVariable currentGaitStartTime = new DoubleYoVariable("currentGaitStartTime", registry);
   private final DoubleYoVariable currentGaitPhaseDuration = new DoubleYoVariable("currentPhaseDuration", registry);
   private final QuadrantDependentList<DoubleYoVariable> swingDurations = new QuadrantDependentList<>();
   private final QuadrantDependentList<DoubleYoVariable> currentSwingCompletions = new QuadrantDependentList<>();
   private final EnumYoVariable<QuadrupedSupportConfiguration> previousGaitPhase = new EnumYoVariable<>("previousGaitPhase", registry, QuadrupedSupportConfiguration.class, false);
   private final EnumYoVariable<QuadrupedSupportConfiguration> currentGaitPhase = new EnumYoVariable<>("currentGaitPhase", registry, QuadrupedSupportConfiguration.class, false);
   private final EnumYoVariable<QuadrupedSupportConfiguration> nextGaitPhase = new EnumYoVariable<>("nextGaitPhase", registry, QuadrupedSupportConfiguration.class, false);
   private final DoubleYoVariable swingZHeight = new DoubleYoVariable("swingZHeight", registry);
   private final DoubleYoVariable desiredGaitPeriod = new DoubleYoVariable("desiredGaitPeriod", registry);
   private final DoubleYoVariable currentGaitCompletion = new DoubleYoVariable("currentGaitCompletion", registry);
   private final DoubleYoVariable currentGaitPhaseCompletion = new DoubleYoVariable("currentPhaseCompletion", registry);
   private final BooleanYoVariable gaitCompleted = new BooleanYoVariable("gaitCompleted", registry);
   private final DoubleYoVariable swingImpactVelocityZ = new DoubleYoVariable("swingImpactVelocityZ", registry);
   private final YoFramePoint centroid = new YoFramePoint("centroid", ReferenceFrame.getWorldFrame(), registry);
   private final YoQuadrupedSupportPolygon targetSupportPolygon = new YoQuadrupedSupportPolygon("targetSupportPolygon", registry);
   private final YoQuadrupedSupportPolygon previousSupportPolygon = new YoQuadrupedSupportPolygon("previousSupportPolygon", registry);
   private final YoQuadrupedSupportPolygon currentSupportPolygon = new YoQuadrupedSupportPolygon("currentSupportPolygon", registry);
   private final YoQuadrupedSupportPolygon currentShrunkenSupportPolygon = new YoQuadrupedSupportPolygon("currentShrunkenSupportPolygon", registry);
   private final YoQuadrupedSupportPolygon nextSupportPolygon = new YoQuadrupedSupportPolygon("nextSupportPolygon", registry);
   private final YoFrameConvexPolygon2d targetYoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("targetYoFrameConvexPolygon2d", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d previousYoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("previousYoFrameConvexPolygon2d", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d currentYoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("currentYoFrameConvexPolygon2d", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d currentShrunkenYoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("currentShrunkenYoFrameConvexPolygon2d", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d nextYoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("nextYoFrameConvexPolygon2d", ReferenceFrame.getWorldFrame(), 4, registry);
   private final VelocityConstrainedPositionTrajectoryGenerator icpTrajectory = new VelocityConstrainedPositionTrajectoryGenerator("icpTrajectory", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d icpTrajectory2dInitialPosition = new YoFramePoint2d("icpTrajectory2dInitialPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d icpTrajectory2dFinalPosition = new YoFramePoint2d("icpTrajectory2dFinalPosition", ReferenceFrame.getWorldFrame(), registry);
   private final QuadrantDependentList<BagOfBalls> swingTrajectoryBagsOfBalls = new QuadrantDependentList<>();

   // Experimental Trot Stuff
   private final FramePoint rightMidpoint = new FramePoint();
   private final FramePoint leftMidpoint = new FramePoint();
   private final FramePoint frontMidpoint = new FramePoint();
   private final FramePoint hindMidpoint = new FramePoint();
   private final FrameVector2d frontDirection = new FrameVector2d();
   private final FrameLine2d sidewaysMidLine = new FrameLine2d(ReferenceFrame.getWorldFrame());
   private final FrameLine2d lengthwiseMidLine = new FrameLine2d(ReferenceFrame.getWorldFrame());
   private final FrameLine2d lineForFindingClosestLineSegment = new FrameLine2d(ReferenceFrame.getWorldFrame());
   private final FrameLine2d rightTrotLine = new FrameLine2d(ReferenceFrame.getWorldFrame());
   private final FrameLine2d leftTrotLine = new FrameLine2d(ReferenceFrame.getWorldFrame());
   private final FrameLineSegment2d lineSegmentLeftTrot = new FrameLineSegment2d(ReferenceFrame.getWorldFrame());
   private final FrameLineSegment2d lineSegmentRightTrot = new FrameLineSegment2d(ReferenceFrame.getWorldFrame());
   private final YoFramePoint2d closestIntersection = new YoFramePoint2d("closestIntersection", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d secondClosestIntersection = new YoFramePoint2d("secondClosestIntersection", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d midPointOfIntersections = new YoFramePoint2d("midPointOfIntersections", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint2d trotCrossPoint = new FramePoint2d();
   private final YoFramePoint2d innerCenterOfPressure = new YoFramePoint2d("innerCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final FrameVector2d awayFromCentroidToClosestIntersection = new FrameVector2d();
   private final YoFramePoint2d outerCenterOfPressure = new YoFramePoint2d("outerCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private double ratioFromMidToClosest;
   private final BooleanYoVariable isInFrontOfLeftTrotLine = new BooleanYoVariable("isInFrontOfLeftTrotLine", registry);
   private final BooleanYoVariable isInFrontOfRightTrotLine = new BooleanYoVariable("isInFrontOfRightTrotLine", registry);

   // Swing PD Controllers
   private final YoFrameVector kp_swing = new YoFrameVector("kp_swing_", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector kd_swing = new YoFrameVector("kd_swing_", ReferenceFrame.getWorldFrame(), registry);
   private final QuadrantDependentList<YoFramePoint> desiredFootPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> desiredFootVelocities = new QuadrantDependentList<>();

   private static final EnumMap<QuadrupedGaitCycle, Double> gaitCyclePeriodMap = new EnumMap<>(QuadrupedGaitCycle.class);
   {
      gaitCyclePeriodMap.put(QuadrupedGaitCycle.STAND, 0.5);
      gaitCyclePeriodMap.put(QuadrupedGaitCycle.SAFE_WALK, 15.0);
      gaitCyclePeriodMap.put(QuadrupedGaitCycle.PERFECT_TROT, 0.8); // 0.30);
      gaitCyclePeriodMap.put(QuadrupedGaitCycle.WALKING_TROT, 1.0); // 0.30);
   }

   public QuadrupedVMCForceMultiGaitController(QuadrupedPhysicalProperties physicalProperties, FullQuadrupedRobotModel fullRobotModel, QuadrantDependentList<FootSwitchInterface> footSwitches, double DT,
         DoubleYoVariable yoTime, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      this.dt = DT;
      this.yoTime = yoTime;

      centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ArrayList<OneDoFJoint> jointsToControl = new ArrayList<OneDoFJoint>();
         OneDoFJoint oneDoFJointBeforeFoot = fullRobotModel.getOneDoFJointBeforeFoot(robotQuadrant);
         fullRobotModel.getOneDoFJointsFromRootToHere(oneDoFJointBeforeFoot, jointsToControl);
         oneDofJoints.set(robotQuadrant, jointsToControl);
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         vmcFootForces.set(robotQuadrant, new YoFrameVector("vmcFootForces" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footPositions.set(robotQuadrant, new YoFramePoint("footPosition" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
         shoulderPositions.set(robotQuadrant, new YoFramePoint("shoulderPosition" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
         shoulderPositionsOnGroundForVis.set(robotQuadrant, new YoFramePoint("shoulderPositionOnGroundForVis" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
         footVelocities.set(robotQuadrant, new YoFrameVector("footVelocity" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         basisForceVectors.set(robotQuadrant, new YoFrameVector[NUMBER_OF_BASIS_VECTORS]);
         basisCoPVectors.set(robotQuadrant, new YoFrameVector("basisCoPVector" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
         basisTorqueVectors.set(robotQuadrant, new YoFrameVector[NUMBER_OF_BASIS_VECTORS]);
         rhoScalars.set(robotQuadrant, new double[NUMBER_OF_BASIS_VECTORS]);
         footToCoMVectors.set(robotQuadrant, new YoFrameVector("footToCoMVector" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));

         for (int i = 0; i < NUMBER_OF_BASIS_VECTORS; i++)
         {
            basisForceVectors.get(robotQuadrant)[i] = new YoFrameVector("basisForceVector" + robotQuadrant.getPascalCaseName() + i, ReferenceFrame.getWorldFrame(), registry);
            basisTorqueVectors.get(robotQuadrant)[i] = new YoFrameVector("basisTorqueVector" + robotQuadrant.getPascalCaseName() + i, ReferenceFrame.getWorldFrame(), registry);
         }
      }

      desiredBodyWrench = new YoWrench("desiredBodyWrench", referenceFrames.getBodyZUpFrame(), ReferenceFrame.getWorldFrame(), registry);
      desiredBodyTwist = new YoTwist("desiredBodyTwist", referenceFrames.getBodyFrame(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), registry);
      bodyTwist = new YoTwist("bodyTwist", referenceFrames.getBodyFrame(), ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame(), registry);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingZTrajectories.set(robotQuadrant, new YoPolynomial("swingZTrajectory" + robotQuadrant.getPascalCaseName(), ORDER_OF_SWING_Z_POLYNOMIAL, registry));
         swingInitialZHeights.set(robotQuadrant, new DoubleYoVariable("swingInitialZHeights" + robotQuadrant.getPascalCaseName(), registry));
         gaitCompletionAtStartOfSwing.set(robotQuadrant, new DoubleYoVariable("swingStartTime" + robotQuadrant.getPascalCaseName(), registry));
         swingDurations.set(robotQuadrant, new DoubleYoVariable("swingDurations" + robotQuadrant.getPascalCaseName(), registry));
         currentSwingCompletions.set(robotQuadrant, new DoubleYoVariable("currentSwingCompletions" + robotQuadrant.getPascalCaseName(), registry));
         desiredFootPositions.set(robotQuadrant, new YoFramePoint("desiredFootPosition" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
         desiredFootVelocities.set(robotQuadrant, new YoFrameVector("desiredFootVelocity" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
      }

      yoGraphicsListRegistry.hideYoGraphics();
      yoGraphicsListRegistry.hideArtifacts();

      if (CREATE_VISUALIZATIONS)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            swingTrajectoryBagsOfBalls.set(robotQuadrant, new BagOfBalls((int) VISUALIZATION_BALLS_PER_SWING, 0.01, "swingTrajectoryBagOfBalls" + robotQuadrant.getPascalCaseName(), YoAppearance.White(), registry, yoGraphicsListRegistry));

            yoGraphicsListRegistry.registerYoGraphic(getName() + "FootForces", new YoGraphicVector("vmcFootForce" + robotQuadrant.getPascalCaseName(), footPositions.get(robotQuadrant), vmcFootForces.get(robotQuadrant), 0.007, YoAppearance.Yellow(), true, 0.01));
            yoGraphicsListRegistry.registerArtifact(getName() + "FootPositions", new YoArtifactPosition("footPosition" + robotQuadrant.getPascalCaseName() + "Vis", footPositions.get(robotQuadrant).getYoX(), footPositions.get(robotQuadrant).getYoY(), GraphicType.BALL_WITH_CROSS, YoAppearance.Color(robotQuadrant.getColor()).getAwtColor(), 0.02));
            yoGraphicsListRegistry.registerArtifact(getName() + "ShoulderPositions", new YoArtifactPosition("shoulderPosition" + robotQuadrant.getPascalCaseName() + "Vis", shoulderPositions.get(robotQuadrant).getYoX(), shoulderPositions.get(robotQuadrant).getYoY(), GraphicType.BALL_WITH_CROSS, YoAppearance.Chartreuse().getAwtColor(), 0.02));
            yoGraphicsListRegistry.registerYoGraphic(getName() + "ShoulderPositionsOnGroundForVis", new YoGraphicPosition("shoulderPositionOnGroundForVis" + robotQuadrant.getPascalCaseName() + "Vis", shoulderPositionsOnGroundForVis.get(robotQuadrant), 0.01, YoAppearance.Chartreuse()));

            yoGraphicsListRegistry.registerYoGraphic(getName() + "BasisVectors", new YoGraphicVector("basisCoPYoGraphicVectors" + robotQuadrant.getPascalCaseName(), footPositions.get(robotQuadrant), basisCoPVectors.get(robotQuadrant), 500, YoAppearance.DarkBlue(), true, 0.01));
            for (int i = 0; i < NUMBER_OF_BASIS_VECTORS; i++)
            {
               yoGraphicsListRegistry.registerYoGraphic(getName() + "BasisVectors", new YoGraphicVector("basisForceYoGraphicVectors" + robotQuadrant.getPascalCaseName() + i, footPositions.get(robotQuadrant), basisForceVectors.get(robotQuadrant)[i], 0.007, YoAppearance.Red(), true, 0.01));
            }
         }
         yoGraphicsListRegistry.registerArtifact(getName() + "IcpVis", new YoArtifactPosition("icpVis", icp.getYoX(), icp.getYoY(), GraphicType.SQUARE, YoAppearance.DarkSlateBlue().getAwtColor(), 0.01));
         yoGraphicsListRegistry.registerArtifact(getName() + "DesiredICPVis", new YoArtifactPosition("desiredICPVis", desiredICP.getYoX(), desiredICP.getYoY(), GraphicType.SQUARE, YoAppearance.Green().getAwtColor(), 0.01));
         yoGraphicsListRegistry.registerArtifact(getName() + "CenterOfMassVis", new YoArtifactPosition("centerOfMassVis", centerOfMass.getYoX(), centerOfMass.getYoY(), GraphicType.BALL_WITH_CROSS, YoAppearance.Black().getAwtColor(), 0.02));
         yoGraphicsListRegistry.registerArtifact(getName() + "CentroidVis", new YoArtifactPosition("centroidVis", centroid.getYoX(), centroid.getYoY(), GraphicType.CROSS, YoAppearance.Black().getAwtColor(), 0.01));
         yoGraphicsListRegistry.registerArtifact(getName() + "CenterOfPressureVis", new YoArtifactPosition("centerOfPressureVis", centerOfPressure.getYoX(), centerOfPressure.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.Lime().getAwtColor(), 0.01));
         yoGraphicsListRegistry.registerArtifact(getName() + "DesiredCenterOfPressureVis", new YoArtifactPosition("desiredCenterOfPressureVis", desiredCenterOfPressure.getYoX(), desiredCenterOfPressure.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.DarkSlateBlue().getAwtColor(), 0.01));
         yoGraphicsListRegistry.registerArtifact(getName() + "SnappedDesiredCenterOfPressureVis", new YoArtifactPosition("snappedDesiredCenterOfPressureVis", snappedDesiredCenterOfPressure.getYoX(), snappedDesiredCenterOfPressure.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.Red().getAwtColor(), 0.02));
         yoGraphicsListRegistry.registerArtifact(getName() + "IcpTrajectory", new YoArtifactLineSegment2d("icpTrajectory", icpTrajectory2dInitialPosition, icpTrajectory2dFinalPosition, Color.BLUE));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotWalkPolygons", new YoArtifactPolygon("targetYoArtifactPolygon", targetYoFrameConvexPolygon2d, Color.RED, false, 1));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotWalkPolygons", new YoArtifactPolygon("previousYoArtifactPolygon", previousYoFrameConvexPolygon2d, Color.GRAY, false, 3));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotWalkPolygons", new YoArtifactPolygon("currentYoArtifactPolygon", currentYoFrameConvexPolygon2d, Color.BLUE, false, 2));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotWalkPolygons", new YoArtifactPolygon("nextYoArtifactPolygon", nextYoFrameConvexPolygon2d, new Color(0, 100, 0), false, 1));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotWalkPolygons", new YoArtifactPolygon("currentShrunkenYoArtifactPolygon", currentShrunkenYoFrameConvexPolygon2d, Color.DARK_GRAY, false, 1));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactLineSegment2d("midPointOfIntersectionsToClosestIntersection", midPointOfIntersections, closestIntersection, Color.GREEN));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactPosition("closestIntersectionVis", closestIntersection.getYoX(), closestIntersection.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.DarkRed().getAwtColor(), 0.003));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactPosition("secondClosestIntersectionVis", secondClosestIntersection.getYoX(), secondClosestIntersection.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.DarkSlateBlue().getAwtColor(), 0.003));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactPosition("innerCenterOfPressureVis", innerCenterOfPressure.getYoX(), innerCenterOfPressure.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.DarkMagenta().getAwtColor(), 0.003));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactPosition("outerCenterOfPressureVis", outerCenterOfPressure.getYoX(), outerCenterOfPressure.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.DarkBlue().getAwtColor(), 0.003));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactLineSegment2d("innerToOuterCenterOfPressure", innerCenterOfPressure, outerCenterOfPressure, Color.BLUE));
         yoGraphicsListRegistry.registerYoGraphic(getName() + "BodyPose", new YoGraphicPosition("stancePoseHigherForVis", stancePoseHigherForVis, 0.03, YoAppearance.Blue()));
         yoGraphicsListRegistry.registerYoGraphic(getName() + "BodyPose", new YoGraphicPosition("desiredStancePoseHigherForVis", desiredStancePoseHigherForVis, 0.03, YoAppearance.Green()));
         yoGraphicsListRegistry.registerYoGraphic(getName() + "BodyWrench", new YoGraphicVector("desiredBodyForceForVis", desiredBodyForceOriginForVis, bodyLinearAcceleration, 0.1, YoAppearance.Cyan(), true, 0.01));
         yoGraphicsListRegistry.registerYoGraphic(getName() + "BodyWrench", new YoGraphicVector("desiredBodyTorqueForVis", desiredBodyTorqueOriginForVis, bodyAngularAcceleration, 0.1, YoAppearance.Orange(), true, 0.01));
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
      initializeInheritedVariables();
      initialize();
   }

   public void initialize()
   {
      currentSupportPolygon.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         currentSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      targetSupportPolygon.setWithoutChecks(currentSupportPolygon);

      currentGait.set(QuadrupedGaitCycle.STAND);
      currentGaitStartTime.set(0.0);
      desiredGaitPeriod.set(gaitCyclePeriodMap.get(currentGait.getEnumValue()));
      desiredGait.set(currentGait.getEnumValue());
      nextGait.set(currentGait.getEnumValue());
      swingZHeight.set(0.1); //0.04);
      swingImpactVelocityZ.set(0.0);
      previousSupportPolygon.setWithoutChecks(currentSupportPolygon);
      previousGaitPhase.set(QuadrupedSupportConfiguration.ALL_FOURS);
      currentGaitPhase.set(QuadrupedSupportConfiguration.ALL_FOURS);
      currentGaitCompletion.set(0.0);

      updatePreGaitCheckEstimates();

      initialStanceHeight = stancePose.getZ();
      desiredStancePose.set(stancePose);
      desiredStancePoseOffset.setToZero();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ArrayList<OneDoFJoint> legJoints = oneDofJoints.get(robotQuadrant);
         for (int i = 0; i < legJoints.size(); i++)
         {
            legJoints.get(i).setUnderPositionControl(false);
         }
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         desiredFootPositions.get(robotQuadrant).set(footPositions.get(robotQuadrant));
      }

      handlePhaseChange();

      updatePostGaitCheckEstimates();

      targetSupportPolygon.setWithoutChecks(currentSupportPolygon);

      kp_icp.set(3.0);
      kd_icp.set(0.1);

      kp_x.set(50.0);
      kd_x.set(10.0);

      kp_y.set(50.0);
      kd_y.set(10.0);

      kp_roll.set(-700.0);
      kd_roll.set(-20.0);

      kp_pitch.set(-700.0);
      kd_pitch.set(-30.0);

      kp_yaw.set(-700.0);
      kd_yaw.set(-100.0);

      kp_z.set(300.0);
      kd_z.set(25.0);

      kp_swing.setX(-5000.0);
      kd_swing.setX(-100.0);
      kp_swing.setY(-5000.0);
      kd_swing.setY(-100.0);
      kp_swing.setZ(-5000.0);
      kd_swing.setZ(-100.0);
   }

   @Override
   public ControllerEvent process()
   {
      updatePreGaitCheckEstimates();
      checkGaitTransitionConditions();
      updatePostGaitCheckEstimates();

      computeSwingTrajectories();

      doICPControl();
      doVirtualModelControl();
      distributeForcesToFeet();

      computeStanceJacobians();

      return null;
   }

   private void initializeInheritedVariables()
   {
      if (!hasInitializedInheritedYoVariables)
      {
         hasInitializedInheritedYoVariables = true;

         YoVariableRegistry rootRegistry = registry;
         while (rootRegistry.getParent() != null)
         {
            rootRegistry = rootRegistry.getParent();
         }

         q_z = (DoubleYoVariable) rootRegistry.getVariable("root.babyBeastSimple", "q_z");
      }
   }

   private void updatePreGaitCheckEstimates()
   {
      //update frames
      referenceFrames.updateFrames();

      //update feet locations
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footVelocities.get(robotQuadrant).set(footPositions.get(robotQuadrant));
         footPositions.get(robotQuadrant).setFromReferenceFrame(referenceFrames.getFootFrame(robotQuadrant));
         shoulderPositions.get(robotQuadrant).setFromReferenceFrame(referenceFrames.getLegAttachmentFrame(robotQuadrant));
         shoulderPositionsOnGroundForVis.get(robotQuadrant).set(shoulderPositions.get(robotQuadrant));
         shoulderPositionsOnGroundForVis.get(robotQuadrant).setZ(0.0);
         footVelocities.get(robotQuadrant).sub(footPositions.get(robotQuadrant));
         footVelocities.get(robotQuadrant).scale(-1.0 / dt);
      }

      bodyPoseWorld.setFromReferenceFrame(referenceFrames.getBodyFrame());
      if (q_z != null)
      {
         bodyPoseWorld.setZ(q_z.getDoubleValue() + SIMULATION_TO_ROBOT_MODEL_Z_DIFFERNCE);
      }
      bodyPoseReferenceFrame.setPoseAndUpdate(bodyPoseWorld.getPosition().getFrameTuple(), bodyPoseWorld.getOrientation().getFrameOrientation());

      bodyTwist.setLinearPart(fullRobotModel.getRootJoint().getLinearVelocityForReading());
      bodyTwist.setAngularPart(fullRobotModel.getRootJoint().getAngularVelocityForReading());

      stancePose.set(bodyPoseWorld);

      stancePoseHigherForVis.set(stancePose.getPosition());
      stancePoseHigherForVis.add(0.0, 0.0, 0.3);

      // compute center of mass position and velocity
      centerOfMass.setFromReferenceFrame(referenceFrames.getCenterOfMassZUpFrame());
      centerOfMassXYProjection.set(centerOfMass.getReferenceFrame(), centerOfMass.getX(), centerOfMass.getY(), 0.0);

      centerOfMassJacobian.compute();

      FrameVector tempVector = centerOfMassVelocity.getFrameTuple();
      centerOfMassJacobian.getCenterOfMassVelocity(tempVector);
      tempVector.changeFrame(centerOfMassVelocity.getReferenceFrame());
      centerOfMassVelocity.setWithoutChecks(tempVector);

      currentGaitCompletion.set((yoTime.getDoubleValue() - currentGaitStartTime.getDoubleValue()) / desiredGaitPeriod.getDoubleValue());
      if (currentGaitCompletion.getDoubleValue() >= 1.0)
      {
         gaitCompleted.set(true);
         currentGaitCompletion.set(currentGaitCompletion.getDoubleValue() % 1.0);
      }
   }

   private void checkGaitTransitionConditions()
   {
      if (gaitCompleted.getBooleanValue())
      {
         handleGaitChange();
         handlePhaseChange();
      }
      else if (currentGait.getEnumValue().getGaitPhase(currentGaitCompletion.getDoubleValue()) != currentGaitPhase.getEnumValue())
      {
         handlePhaseChange();
      }
   }

   private void handleGaitChange()
   {
      gaitCompleted.set(false);
      currentGaitStartTime.set(yoTime.getDoubleValue());
      currentGait.set(nextGait.getEnumValue());
      desiredGaitPeriod.set(gaitCyclePeriodMap.get(currentGait.getEnumValue()));
   }

   private void handlePhaseChange()
   {
      // Update phases
      currentGaitPhaseStartTime.set(currentGaitCompletion.getDoubleValue());
      currentGaitPhaseDuration.set(currentGait.getEnumValue().getRemainingPhaseDuration(currentGaitCompletion.getDoubleValue()));
      previousGaitPhase.set(currentGaitPhase.getEnumValue());
      currentGaitPhase.set(currentGait.getEnumValue().getGaitPhase(currentGaitCompletion.getDoubleValue()));
      if (currentGait.getEnumValue().isLastPhase(currentGaitCompletion.getDoubleValue()))
      {
         nextGait.set(desiredGait.getEnumValue());
         nextGaitPhase.set(nextGait.getEnumValue().getGaitPhase(0.0));
      }
      else
      {
         nextGaitPhase.set(currentGait.getEnumValue().getNextGaitPhase(currentGaitCompletion.getDoubleValue()));
      }

      // Update support polygons
      previousSupportPolygon.setWithoutChecks(currentSupportPolygon);
      nextSupportPolygon.clear();
      for (RobotQuadrant robotQuadrant : nextGaitPhase.getEnumValue().supportQuadrants())
      {
         nextSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
//         nextSupportPolygon.reviveFootstep(robotQuadrant);
         nextSupportPolygon.getFootstep(robotQuadrant).setX(shoulderPositions.get(robotQuadrant).getX());
         nextSupportPolygon.getFootstep(robotQuadrant).setY(shoulderPositions.get(robotQuadrant).getY());
//         nextSupportPolygon.getFootstep(robotQuadrant).setZ(0.0);
      }

      // Update Swing Parameters
      for (RobotQuadrant robotQuadrant : previousGaitPhase.getEnumValue().supportQuadrants())
      {
         swingInitialZHeights.get(robotQuadrant).set(footPositions.get(robotQuadrant).getZ());
         gaitCompletionAtStartOfSwing.get(robotQuadrant).set(currentGaitCompletion.getDoubleValue());

         // Measure durations of legs switching to swing
         if (currentGaitPhase.getEnumValue().isSwingQuadrant(robotQuadrant))
         {
            swingDurations.get(robotQuadrant).set(currentGait.getEnumValue().getRemainingSwingDuration(robotQuadrant, currentGaitCompletion.getDoubleValue()));

            // Create trajectory visualization
            if (CREATE_VISUALIZATIONS)
            {
               for (int i = 0; i < VISUALIZATION_BALLS_PER_SWING; i++)
               {
                  double timeInSwing = i * (swingDurations.get(robotQuadrant).getDoubleValue() / VISUALIZATION_BALLS_PER_SWING);
                  double virtualPhaseCompletion = (currentGaitCompletion.getDoubleValue() + timeInSwing
                        - gaitCompletionAtStartOfSwing.get(robotQuadrant).getDoubleValue()) / swingDurations.get(robotQuadrant).getDoubleValue();
                  computeSwingTrajectoryForQuadrant(robotQuadrant, virtualPhaseCompletion);
               }
            }
         }
      }
   }

   private void updatePostGaitCheckEstimates()
   {
      currentGaitPhaseCompletion.set((currentGaitCompletion.getDoubleValue() - currentGaitPhaseStartTime.getDoubleValue()) / currentGaitPhaseDuration.getDoubleValue());
      for (RobotQuadrant robotQuadrant : currentGaitPhase.getEnumValue().swingQuadrants())
      {
         currentSwingCompletions.get(robotQuadrant).set((currentGaitCompletion.getDoubleValue() - gaitCompletionAtStartOfSwing.get(robotQuadrant).getDoubleValue()) / swingDurations.get(robotQuadrant).getDoubleValue());
      }

      currentSupportPolygon.clear();
      for (RobotQuadrant robotQuadrant : currentGaitPhase.getEnumValue().supportQuadrants())
      {
         currentSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      currentShrunkenSupportPolygon.setWithoutChecks(currentSupportPolygon);
      currentShrunkenSupportPolygon.shrinkPolygon2d(ICP_BOUNDS_MARGIN);

      targetSupportPolygon.packYoFrameConvexPolygon2d(targetYoFrameConvexPolygon2d);
      previousSupportPolygon.packYoFrameConvexPolygon2d(previousYoFrameConvexPolygon2d);
      currentSupportPolygon.packYoFrameConvexPolygon2d(currentYoFrameConvexPolygon2d);
      currentShrunkenSupportPolygon.packYoFrameConvexPolygon2d(currentYoFrameConvexPolygon2d);
      nextSupportPolygon.packYoFrameConvexPolygon2d(nextYoFrameConvexPolygon2d);

      currentSupportPolygon.getCentroid(centroid);

      // compute instantaneous capture point
      double zDelta = centerOfMass.getZ() - centroid.getZ();
      double omega = Math.sqrt(GRAVITY / zDelta);
      icpVelocity.set(icp);
      icp.setX(centerOfMass.getX() + centerOfMassVelocity.getX() / omega);
      icp.setY(centerOfMass.getY() + centerOfMassVelocity.getY() / omega);
      icpVelocity.sub(icp);
      icpVelocity.scale(-1.0 / dt);

      // update CoP
      double fzTotal = 0.0;
      centerOfPressure.setToZero();
      for (RobotQuadrant robotQuadrant : currentSupportPolygon.getSupportingQuadrantsInOrder())
      {
         centerOfPressure.add(footPositions.get(robotQuadrant).getX() * vmcFootForces.get(robotQuadrant).getZ(), 0.0, 0.0);
         centerOfPressure.add(0.0, footPositions.get(robotQuadrant).getY() * vmcFootForces.get(robotQuadrant).getZ(), 0.0);
         centerOfPressure.add(0.0, 0.0, footPositions.get(robotQuadrant).getZ() * vmcFootForces.get(robotQuadrant).getZ());
         fzTotal += vmcFootForces.get(robotQuadrant).getZ();
      }
      if (fzTotal < 1e-14)
      {
         centerOfPressure.set(Double.NaN, Double.NaN, Double.NaN);
      }
      else
      {
         centerOfPressure.scale(1.0 / fzTotal);
      }

      for (RobotQuadrant robotQuadrant : currentSupportPolygon.getSupportingQuadrantsInOrder())
      {
         footToCoMVectors.get(robotQuadrant).set(centerOfMass);
         footToCoMVectors.get(robotQuadrant).sub(footPositions.get(robotQuadrant));
         calculateBasisVectors(robotQuadrant, footToCoMVectors.get(robotQuadrant));
      }

      if (currentGait.getEnumValue() != QuadrupedGaitCycle.STAND)
      {
         targetSupportPolygon.translateForward(desiredRobotVelocity.getX() * dt);
         targetSupportPolygon.translateSideways(desiredRobotVelocity.getY() * dt);
         targetSupportPolygon.yawAboutCentroid(desiredRobotAngularVelocityZ.getDoubleValue() * dt);
      }
   }

   private void calculateBasisVectors(RobotQuadrant robotQuadrant, YoFrameVector footToCoMVector)
   {
      for (int i = 0; i < NUMBER_OF_BASIS_VECTORS; i++)
      {
         basisForceVectors.get(robotQuadrant)[i].setX(Math.cos(ANGLE_BETWEEN_BASIS_VECTORS * i + bodyPoseWorld.getYaw()));
         basisForceVectors.get(robotQuadrant)[i].setY(Math.sin(ANGLE_BETWEEN_BASIS_VECTORS * i + bodyPoseWorld.getYaw()));
         basisForceVectors.get(robotQuadrant)[i].scale(COEFFICIENT_OF_FRICTION);
         basisForceVectors.get(robotQuadrant)[i].setZ(BASIS_VECTOR_HEIGHT);
         basisForceVectors.get(robotQuadrant)[i].normalize();


         basisTorqueVectors.get(robotQuadrant)[i].cross(footToCoMVector, basisForceVectors.get(robotQuadrant)[i]);
         basisTorqueVectors.get(robotQuadrant)[i].normalize();
      }

      basisCoPVectors.get(robotQuadrant).sub(footPositions.get(robotQuadrant), bodyPoseWorld.getPosition());
      basisCoPVectors.get(robotQuadrant).setZ(0.0);
      basisCoPVectors.get(robotQuadrant).scale(1.0 / desiredBodyWrench.getLinearPartZ());
   }

   private void computeSwingTrajectories()
   {
      for (RobotQuadrant robotQuadrant : currentGaitPhase.getEnumValue().swingQuadrants())
      {
//         computeSwingTrajectoryForQuadrant(robotQuadrant, currentSwingCompletions.get(robotQuadrant).getDoubleValue());
         computeSwingTrajectoryForQuadrant(robotQuadrant, currentGaitPhaseCompletion.getDoubleValue());
      }
   }

   private void computeSwingTrajectoryForQuadrant(RobotQuadrant robotQuadrant, double currentPhaseCompletion)
   {
      // Swing Trajectory Z
      double timeStart = 0.0;
      double timePeak = 0.5;
      double timeEnd = 1.0;
      double zStart = swingInitialZHeights.get(robotQuadrant).getDoubleValue();
      double zMid = swingZHeight.getDoubleValue();
      double zEnd = zStart;
      double zVelocityFinal = swingImpactVelocityZ.getDoubleValue();
      swingZTrajectories.get(robotQuadrant).setCubicWithIntermediatePositionAndFinalVelocityConstraint(timeStart, timePeak, timeEnd, zStart, zMid, zEnd, zVelocityFinal);
      swingZTrajectories.get(robotQuadrant).compute(currentPhaseCompletion);

      desiredFootPositions.get(robotQuadrant).setZ(swingZTrajectories.get(robotQuadrant).getPosition());
      desiredFootVelocities.get(robotQuadrant).setZ(swingZTrajectories.get(robotQuadrant).getVelocity());

      // Swing Trajectory X and Y
      double startX = previousSupportPolygon.getFootstep(robotQuadrant).getX();
      double startY = previousSupportPolygon.getFootstep(robotQuadrant).getY();
//      double endX = nextSupportPolygon.getFootstep(robotQuadrant).getX();
//      double endY = nextSupportPolygon.getFootstep(robotQuadrant).getY();
      double endX = targetSupportPolygon.getFootstep(robotQuadrant).getX();
      double endY = targetSupportPolygon.getFootstep(robotQuadrant).getY();

      desiredFootPositions.get(robotQuadrant).setX((endX - startX) * currentPhaseCompletion + startX);
      desiredFootPositions.get(robotQuadrant).setY((endY - startY) * currentPhaseCompletion + startY);

      int ballIndex = MathTools.clipToMinMax((int) Math.floor(currentPhaseCompletion * 20.0), 0, 20);
      swingTrajectoryBagsOfBalls.get(robotQuadrant).setBall(desiredFootPositions.get(robotQuadrant).getFrameTuple(), YoAppearance.White(), ballIndex);
   }

   private void doICPControl()
   {
      // Update desired ICP trajectory
      icpTrajectory.setTrajectoryTime(1.0);
      if (currentGaitPhase.getEnumValue() == QuadrupedSupportConfiguration.ALL_FOURS)
      {
         previousSupportPolygon.getCentroid(icpTrajectory.getInitialPosition());
         nextSupportPolygon.getCentroid(icpTrajectory.getFinalPosition());
      }
      else
      {
         currentSupportPolygon.getCentroid(icpTrajectory.getInitialPosition());
         currentSupportPolygon.getCentroid(icpTrajectory.getFinalPosition());
      }
      icpTrajectory.getInitialVelocity().setToZero();
      icpTrajectory.getFinalVelocity().setToZero();
      icpTrajectory.initialize();
      icpTrajectory.compute(currentGaitPhaseCompletion.getDoubleValue());
      icpTrajectory.getProjectedOntoXYPlane(desiredICP);
      icpTrajectory2dInitialPosition.setByProjectionOntoXYPlane(icpTrajectory.getInitialPosition());
      icpTrajectory2dFinalPosition.setByProjectionOntoXYPlane(icpTrajectory.getFinalPosition());

      targetSupportPolygon.getCentroid2d(desiredICP);
      desiredICP.add(desiredStancePoseOffset.getX(), desiredStancePoseOffset.getY());

      desiredICPVelocity.setToZero();
      desiredICPVelocity.add(desiredRobotVelocity);

      icpActionVector.setToZero();
      icpActionVector.getYoX().add(kp_icp.getDoubleValue() * (desiredICP.getX() - icp.getX()));
      icpActionVector.getYoY().add(kp_icp.getDoubleValue() * (desiredICP.getY() - icp.getY()));
      icpActionVector.getYoX().add(kd_icp.getDoubleValue() * (desiredICPVelocity.getX() - icpVelocity.getX()));
      icpActionVector.getYoY().add(kd_icp.getDoubleValue() * (desiredICPVelocity.getY() - icpVelocity.getY()));

      desiredCenterOfPressure.set(icp);
      desiredCenterOfPressure.sub(icpActionVector);

      if (currentSupportPolygon.size() < 3)
      {
         snappedDesiredCenterOfPressure.set(findTrotSpecificCenterOfPressure());
      }
      else
      {
         snappedDesiredCenterOfPressure.set(desiredCenterOfPressure);
      }

      currentShrunkenSupportPolygon.snapPointToClosestEdgeOfPolygonIfOutside2d(snappedDesiredCenterOfPressure);
   }

   private YoFramePoint2d findTrotSpecificCenterOfPressure()
   {
      targetSupportPolygon.getFrontMidpoint(frontMidpoint);
      targetSupportPolygon.getRightMidpoint(rightMidpoint);
      targetSupportPolygon.getHindMidpoint(hindMidpoint);
      targetSupportPolygon.getLeftMidpoint(leftMidpoint);

      frontDirection.setByProjectionOntoXYPlane(frontMidpoint);
      frontDirection.sub(hindMidpoint.getX(), hindMidpoint.getY());

      sidewaysMidLine.setByProjectionOntoXYPlane(leftMidpoint, rightMidpoint);
      lengthwiseMidLine.setByProjectionOntoXYPlane(frontMidpoint, hindMidpoint);

      lineSegmentLeftTrot.setByProjectionOntoXYPlane(footPositions.get(RobotQuadrant.HIND_RIGHT).getFrameTuple(), footPositions.get(RobotQuadrant.FRONT_LEFT).getFrameTuple());
      lineSegmentRightTrot.setByProjectionOntoXYPlane(footPositions.get(RobotQuadrant.HIND_LEFT).getFrameTuple(), footPositions.get(RobotQuadrant.FRONT_RIGHT).getFrameTuple());

      leftTrotLine.setByProjectionOntoXYPlane(footPositions.get(RobotQuadrant.HIND_RIGHT).getFrameTuple(), footPositions.get(RobotQuadrant.FRONT_LEFT).getFrameTuple());
      rightTrotLine.setByProjectionOntoXYPlane(footPositions.get(RobotQuadrant.HIND_LEFT).getFrameTuple(), footPositions.get(RobotQuadrant.FRONT_RIGHT).getFrameTuple());

      leftTrotLine.getIntersectionWithLine(rightTrotLine, trotCrossPoint);

      isInFrontOfLeftTrotLine.set(leftTrotLine.isPointInFrontOfLine(frontDirection, desiredCenterOfPressure.getFrameTuple2d()));
      isInFrontOfRightTrotLine.set(rightTrotLine.isPointInFrontOfLine(frontDirection, desiredCenterOfPressure.getFrameTuple2d()));

      lineForFindingClosestLineSegment.setPoint(desiredCenterOfPressure.getFrameTuple2d());

      if (isInFrontOfLeftTrotLine.getBooleanValue() == isInFrontOfRightTrotLine.getBooleanValue())
      {
         lineForFindingClosestLineSegment.getLine2d().getNormalizedVector().set(sidewaysMidLine.getLine2d().getNormalizedVector());
      }
      else
      {
         lineForFindingClosestLineSegment.getLine2d().getNormalizedVector().set(lengthwiseMidLine.getLine2d().getNormalizedVector());
      }

      FramePoint2d closestIntersectionFrameTuple = closestIntersection.getFrameTuple2d();
      leftTrotLine.getIntersectionWithLine(lineForFindingClosestLineSegment, closestIntersectionFrameTuple);
      closestIntersection.setWithoutChecks(closestIntersectionFrameTuple);

      double distanceUpward = closestIntersection.distance(desiredCenterOfPressure.getFrameTuple2d());

      FramePoint2d secondClosestIntersectionFramePoint = secondClosestIntersection.getFrameTuple2d();
      rightTrotLine.getIntersectionWithLine(lineForFindingClosestLineSegment, secondClosestIntersectionFramePoint);
      secondClosestIntersection.setWithoutChecks(secondClosestIntersectionFramePoint);

      double distanceDownward = secondClosestIntersection.distance(desiredCenterOfPressure.getFrameTuple2d());

      if (distanceUpward > distanceDownward)
      {
         double x = secondClosestIntersection.getFrameTuple2d().getX();
         double y = secondClosestIntersection.getFrameTuple2d().getY();
         secondClosestIntersection.set(closestIntersection.getFrameTuple2d());
         closestIntersection.set(x, y);
      }

      midPointOfIntersections.interpolate(closestIntersection.getFrameTuple2d(), secondClosestIntersection.getFrameTuple2d(), 0.5);

      double midPointOfIntersectionsToDesiredCenterOfPressure = midPointOfIntersections.distance(desiredCenterOfPressure.getFrameTuple2d());
      double midPointOfIntersectionsToClosestIntersection = midPointOfIntersections.distance(closestIntersection.getFrameTuple2d());

      ratioFromMidToClosest = midPointOfIntersectionsToDesiredCenterOfPressure / midPointOfIntersectionsToClosestIntersection;

      innerCenterOfPressure.interpolate(secondClosestIntersection.getFrameTuple2d(), trotCrossPoint, ratioFromMidToClosest);

      awayFromCentroidToClosestIntersection.sub(closestIntersection.getFrameTuple2d(), trotCrossPoint);
      awayFromCentroidToClosestIntersection.scale(ratioFromMidToClosest);

      outerCenterOfPressure.set(closestIntersection.getFrameTuple2d());
      outerCenterOfPressure.add(awayFromCentroidToClosestIntersection);

      double distanceFromInnerCenterOfPressureToPolygon = currentSupportPolygon.getDistanceInside2d(innerCenterOfPressure.getFrameTuple2d());
      double distanceFromOuterCenterOfPressureToPolygon = currentSupportPolygon.getDistanceInside2d(outerCenterOfPressure.getFrameTuple2d());

      if (distanceFromInnerCenterOfPressureToPolygon > distanceFromOuterCenterOfPressureToPolygon)
      {
         return innerCenterOfPressure;
      }
      else
      {
         return outerCenterOfPressure;
      }
   }

   private void doVirtualModelControl()
   {
      desiredStancePose.setToZero();
      desiredStancePose.setX(desiredICP.getX());
      desiredStancePose.setY(desiredICP.getY());
      desiredStancePose.setZ(initialStanceHeight + desiredStancePoseOffset.getZ());
      desiredStancePose.getOrientation().add(desiredStancePoseOffset.getOrientation());
      desiredStancePose.getOrientation().getYaw().add(targetSupportPolygon.getNominalYaw());

      desiredStancePoseHigherForVis.set(desiredStancePose.getPosition());
      desiredStancePoseHigherForVis.add(0.0, 0.0, 0.3);
      desiredBodyForceOriginForVis.set(desiredStancePoseHigherForVis);
      desiredBodyForceOriginForVis.add(Math.cos(stancePose.getYaw()) * 0.3, Math.sin(stancePose.getYaw()) * 0.3, 0.0);
      desiredBodyTorqueOriginForVis.set(desiredStancePoseHigherForVis);
      desiredBodyTorqueOriginForVis.add(Math.cos(stancePose.getYaw()) * -0.3, Math.sin(stancePose.getYaw()) * -0.3, 0.0);

      desiredBodyTwist.setToZero();

      bodyLinearAcceleration.setToZero();
      bodyLinearAcceleration.getYoX().add(kp_x.getDoubleValue() * (desiredStancePose.getX() - stancePose.getX()));
      bodyLinearAcceleration.getYoY().add(kp_y.getDoubleValue() * (desiredStancePose.getY() - stancePose.getY()));
      bodyLinearAcceleration.getYoX().add(kd_x.getDoubleValue() * (desiredBodyTwist.getLinearPartX() - bodyTwist.getLinearPartX()));
      bodyLinearAcceleration.getYoY().add(kd_y.getDoubleValue() * (desiredBodyTwist.getLinearPartY() - bodyTwist.getLinearPartY()));
      bodyLinearAcceleration.getYoZ().add(kp_z.getDoubleValue() * (desiredStancePose.getZ() - stancePose.getZ()));
      bodyLinearAcceleration.getYoZ().add(kd_z.getDoubleValue() * (desiredBodyTwist.getLinearPartZ() - bodyTwist.getLinearPartZ()));

      bodyAngularAcceleration.setToZero();
      bodyAngularAcceleration.getYoX().add(kp_roll.getDoubleValue() * (desiredStancePose.getRoll() - stancePose.getRoll()));
      bodyAngularAcceleration.getYoX().add(kd_roll.getDoubleValue() * (desiredBodyTwist.getAngularPartX() - bodyTwist.getAngularPartX()));
      bodyAngularAcceleration.getYoY().add(kp_pitch.getDoubleValue() * (desiredStancePose.getPitch() - stancePose.getPitch()));
      bodyAngularAcceleration.getYoY().add(kd_pitch.getDoubleValue() * (desiredBodyTwist.getAngularPartY() - bodyTwist.getAngularPartY()));
      bodyAngularAcceleration.getYoZ().add(kp_yaw.getDoubleValue() * (desiredStancePose.getYaw() - stancePose.getYaw()));
      bodyAngularAcceleration.getYoZ().add(kd_yaw.getDoubleValue() * (desiredBodyTwist.getAngularPartZ() - bodyTwist.getAngularPartZ()));

      desiredBodyWrench.setToZero();
      desiredBodyWrench.getYoLinearPart().add(0.0, 0.0, GRAVITY);
      desiredBodyWrench.getYoLinearPart().add(bodyLinearAcceleration);
      desiredBodyWrench.getYoLinearPart().scale(ESTIMATED_MASS);
      desiredBodyWrench.getYoAngularPart().add(bodyAngularAcceleration);
      desiredBodyWrench.getYoAngularPart().scale(ESTIMATED_ROTATIONAL_INERTIA);
   }

   private void distributeForcesToFeet()
   {
      distributeForcesToSupportFeet();
      distributeForcesToSwingFeet();
   }

   private void distributeForcesToSupportFeet()
   {
      if (USE_COPX_AND_COPY)
      {
//         FramePoint2d snappedDesiredCenterOfPressureBodyFrame = snappedDesiredCenterOfPressure.getFrameTuple2d();
//         snappedDesiredCenterOfPressureBodyFrame.changeFrameAndProjectToXYPlane(bodyPoseReferenceFrame);
//
//         bodyWrenchMatrix.set(0, 0, snappedDesiredCenterOfPressureBodyFrame.getX());
//         bodyWrenchMatrix.set(1, 0, snappedDesiredCenterOfPressureBodyFrame.getY());
         bodyWrenchMatrix.set(0, 0, snappedDesiredCenterOfPressure.getX() - bodyPoseWorld.getX());
         bodyWrenchMatrix.set(1, 0, snappedDesiredCenterOfPressure.getY() - bodyPoseWorld.getY());

//         snappedDesiredCenterOfPressureBodyFrame.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      }
      else
      {
         bodyWrenchMatrix.set(0, 0, desiredBodyWrench.getLinearPartX());
         bodyWrenchMatrix.set(1, 0, desiredBodyWrench.getLinearPartY());
      }

      bodyWrenchMatrix.set(2, 0, desiredBodyWrench.getLinearPartZ());
      bodyWrenchMatrix.set(3, 0, desiredBodyWrench.getAngularPartX());
      bodyWrenchMatrix.set(4, 0, desiredBodyWrench.getAngularPartY());
      bodyWrenchMatrix.set(5, 0, desiredBodyWrench.getAngularPartZ());

      basisMatrix.reshape(6, currentSupportPolygon.size() * NUMBER_OF_BASIS_VECTORS);
      rhoMatrix.reshape(currentSupportPolygon.size() * NUMBER_OF_BASIS_VECTORS, 1);

      for (int quadrantIndex = 0; quadrantIndex < currentSupportPolygon.getSupportingQuadrantsInOrder().length; quadrantIndex++)
      {
         RobotQuadrant robotQuadrant = currentSupportPolygon.getSupportingQuadrantsInOrder()[quadrantIndex];
         for (int basisIndex = 0; basisIndex < NUMBER_OF_BASIS_VECTORS; basisIndex++)
         {
            if (USE_COPX_AND_COPY)
            {
               basisMatrix.set(0, quadrantIndex * NUMBER_OF_BASIS_VECTORS + basisIndex, basisCoPVectors.get(robotQuadrant).getX());
               basisMatrix.set(1, quadrantIndex * NUMBER_OF_BASIS_VECTORS + basisIndex, basisCoPVectors.get(robotQuadrant).getY());
            }
            else
            {
               basisMatrix.set(0, quadrantIndex * NUMBER_OF_BASIS_VECTORS + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getX());
               basisMatrix.set(1, quadrantIndex * NUMBER_OF_BASIS_VECTORS + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getY());
            }
            basisMatrix.set(2, quadrantIndex * NUMBER_OF_BASIS_VECTORS + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getZ());
            basisMatrix.set(3, quadrantIndex * NUMBER_OF_BASIS_VECTORS + basisIndex, basisTorqueVectors.get(robotQuadrant)[basisIndex].getX());
            basisMatrix.set(4, quadrantIndex * NUMBER_OF_BASIS_VECTORS + basisIndex, basisTorqueVectors.get(robotQuadrant)[basisIndex].getY());
            basisMatrix.set(5, quadrantIndex * NUMBER_OF_BASIS_VECTORS + basisIndex, basisTorqueVectors.get(robotQuadrant)[basisIndex].getZ());
         }
      }

      solver.setA(basisMatrix);
      solver.solve(bodyWrenchMatrix, rhoMatrix);

      for (int quadrantIndex = 0; quadrantIndex < currentSupportPolygon.getSupportingQuadrantsInOrder().length; quadrantIndex++)
      {
         RobotQuadrant robotQuadrant = currentSupportPolygon.getSupportingQuadrantsInOrder()[quadrantIndex];
         for (int basisIndex = 0; basisIndex < NUMBER_OF_BASIS_VECTORS; basisIndex++)
         {
            rhoScalars.get(robotQuadrant)[basisIndex] = rhoMatrix.get(quadrantIndex * NUMBER_OF_BASIS_VECTORS + basisIndex, 0);
         }
      }

      for (RobotQuadrant robotQuadrant : currentSupportPolygon.getSupportingQuadrantsInOrder())
      {
         vmcFootForces.get(robotQuadrant).setToZero();
         for (int basisIndex = 0; basisIndex < NUMBER_OF_BASIS_VECTORS; basisIndex++)
         {
            rhoScalars.get(robotQuadrant)[basisIndex] = MathTools.clipToMinMax(rhoScalars.get(robotQuadrant)[basisIndex], 400.0); // TODO Don't clip rhos
            basisForceVectors.get(robotQuadrant)[basisIndex].scale(rhoScalars.get(robotQuadrant)[basisIndex]);
            vmcFootForces.get(robotQuadrant).add(basisForceVectors.get(robotQuadrant)[basisIndex]);
         }
      }
   }

   private void distributeForcesToSwingFeet()
   {
      for (RobotQuadrant robotQuadrant : currentGaitPhase.getEnumValue().swingQuadrants())
      {
         vmcFootForces.get(robotQuadrant).setToZero();
         vmcFootForces.get(robotQuadrant).getYoX().add(kp_swing.getX() * (desiredFootPositions.get(robotQuadrant).getX() - footPositions.get(robotQuadrant).getX()));
         vmcFootForces.get(robotQuadrant).getYoX().add(kd_swing.getX() * (desiredFootVelocities.get(robotQuadrant).getX() - footVelocities.get(robotQuadrant).getX()));
         vmcFootForces.get(robotQuadrant).getYoY().add(kp_swing.getY() * (desiredFootPositions.get(robotQuadrant).getY() - footPositions.get(robotQuadrant).getY()));
         vmcFootForces.get(robotQuadrant).getYoY().add(kd_swing.getY() * (desiredFootVelocities.get(robotQuadrant).getY() - footVelocities.get(robotQuadrant).getY()));
         vmcFootForces.get(robotQuadrant).getYoZ().add(kp_swing.getZ() * (desiredFootPositions.get(robotQuadrant).getZ() - footPositions.get(robotQuadrant).getZ()));
         vmcFootForces.get(robotQuadrant).getYoZ().add(kd_swing.getZ() * (desiredFootVelocities.get(robotQuadrant).getZ() - footVelocities.get(robotQuadrant).getZ()));
      }
   }

   private void computeStanceJacobians()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         computeStanceJacobianForLeg(robotQuadrant);
      }
   }

   private void computeStanceJacobianForLeg(RobotQuadrant robotQuadrant)
   {
      for (OneDoFJoint oneDoFJoint : oneDofJoints.get(robotQuadrant))
      {
         jointPosition.setFromReferenceFrame(oneDoFJoint.getFrameBeforeJoint());

         oneDoFJoint.getJointAxis(jointAxis);
         jointAxis.changeFrame(ReferenceFrame.getWorldFrame());

         jointToFootVector.set(footPositions.get(robotQuadrant).getFrameTuple());
         jointToFootVector.sub(jointPosition);

         vmcRequestedTorqueFromJoint.setToZero();
         vmcRequestedTorqueFromJoint.cross(jointToFootVector, vmcFootForces.get(robotQuadrant).getFrameTuple());

         double tau = -jointAxis.dot(vmcRequestedTorqueFromJoint);

         tau = MathTools.clipToMinMax(tau, 100); // TODO Implement actual torque limits

         oneDoFJoint.setTau(tau);
      }
   }

   public String getName()
   {
      return "TrotWalkController";
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getDescription()
   {
      return getName();
   }

   @Override
   public void onExit()
   {

   }
}
