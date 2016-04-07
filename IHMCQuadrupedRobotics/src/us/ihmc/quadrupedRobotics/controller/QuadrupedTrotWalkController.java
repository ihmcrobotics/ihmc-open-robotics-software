package us.ihmc.quadrupedRobotics.controller;

import java.awt.Color;
import java.util.ArrayList;
import java.util.EnumMap;

import org.ejml.alg.dense.linsol.svd.SolvePseudoInverseSvd;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.quadrupedRobotics.gait.QuadrupedGaitCycle;
import us.ihmc.quadrupedRobotics.gait.QuadrupedSupportConfiguration;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.YoQuadrupedSupportPolygon;
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
import us.ihmc.robotics.math.frames.YoTwist;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLine;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPosition;

public class QuadrupedTrotWalkController extends QuadrupedController
{
   // Constants
   private static final double GRAVITY = 9.81;
   private static final double INITIAL_STANCE_HEIGHT = 0.625;
   private static final double SIMULATION_TO_ROBOT_MODEL_Z_DIFFERNCE = 0.08;
   private static final double ESTIMATED_MASS = 63.9; // TODO PDControl this when z-vel=0
   private static final double ESTIMATED_ROTATIONAL_INERTIA = 5.0; // TODO PDControl this when z-vel=0
   private static final double COEFFICIENT_OF_FRICTION = 0.7;
   
   // Controller Options
   public static final boolean USE_COPX_AND_COPY = true;
   public static final boolean CREATE_VISUALIZATIONS = true;
   
   // Inherited Variables
   private final double dt;
   private final DoubleYoVariable yoTime;
   private final YoVariableRegistry registry = new YoVariableRegistry("TrotWalkController");
   private final QuadrupedReferenceFrames referenceFrames;
   private final SDFFullRobotModel fullRobotModel;
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
   private final QuadrantDependentList<YoFrameVector> footVelocities = new QuadrantDependentList<YoFrameVector>();
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final YoFramePoint centerOfMass = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector centerOfMassVelocity = new YoFrameVector("centerOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfMassXYProjection = new YoFramePoint("centerOfMassXYProjection", ReferenceFrame.getWorldFrame(), registry);
   
   // Balancing
   private final YoFramePoint2d icp = new YoFramePoint2d("icp", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d desiredICP = new YoFramePoint2d("desiredICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint centerOfPressure = new YoFramePoint("centerOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d desiredCenterOfPressure = new YoFramePoint2d("desiredCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint2d snappedDesiredCenterOfPressure = new YoFramePoint2d("snappedDesiredCenterOfPressure", ReferenceFrame.getWorldFrame(), registry);
   private final QuadrantDependentList<YoFrameVector[]> basisForceVectors = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector[]> basisTorqueVectors = new QuadrantDependentList<>();
   private final QuadrantDependentList<double[]> rhoScalars = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> footToCoMVectors = new QuadrantDependentList<>();
   private final DenseMatrix64F bodyWrenchMatrix = new DenseMatrix64F(6, 1);
   private final DenseMatrix64F basisMatrix = new DenseMatrix64F(6, 16);
   private final DenseMatrix64F rhoMatrix = new DenseMatrix64F(16, 1);
   private final SolvePseudoInverseSvd solver = new SolvePseudoInverseSvd();
   private final QuadrantDependentList<YoFrameVector> vmcFootForces = new QuadrantDependentList<>();
   private final YoFramePose bodyPoseWorld = new YoFramePose("body", ReferenceFrame.getWorldFrame(), registry);
   private final YoTwist bodyTwist;
   private final YoFramePose stancePose = new YoFramePose("stance", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint stancePoseHigherForVis = new YoFramePoint("stancePoseHigherForVis", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint desiredStancePoseHigherForVis = new YoFramePoint("desiredStancePoseHigherForVis", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose desiredStancePose = new YoFramePose("desiredStance", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePose desiredStancePoseOffset = new YoFramePose("desiredStanceOffset", ReferenceFrame.getWorldFrame(), registry);
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
   private final EnumYoVariable<QuadrupedGaitCycle> currentGait = new EnumYoVariable<>("gait", registry, QuadrupedGaitCycle.class);
   private final QuadrantDependentList<YoPolynomial> swingZTrajectories = new QuadrantDependentList<>();
   private final QuadrantDependentList<DoubleYoVariable> swingInitialZHeights = new QuadrantDependentList<>();
   private final QuadrantDependentList<DoubleYoVariable> gaitCompletionAtStartOfSwing = new QuadrantDependentList<>();
   private final DoubleYoVariable phaseStartTime = new DoubleYoVariable("phaseStartTime", registry);
   private final DoubleYoVariable gaitStartTime = new DoubleYoVariable("gaitStartTime", registry);
   private final QuadrantDependentList<DoubleYoVariable> swingDurations = new QuadrantDependentList<>();
   private final EnumYoVariable<QuadrupedSupportConfiguration> previousGaitPhase = new EnumYoVariable<>("previousGaitPhase", registry, QuadrupedSupportConfiguration.class, false);
   private final EnumYoVariable<QuadrupedSupportConfiguration> currentGaitPhase = new EnumYoVariable<>("currentGaitPhase", registry, QuadrupedSupportConfiguration.class, false);
   private final EnumYoVariable<QuadrupedSupportConfiguration> nextGaitPhase = new EnumYoVariable<>("nextGaitPhase", registry, QuadrupedSupportConfiguration.class, false);
   private final DoubleYoVariable swingZHeight = new DoubleYoVariable("swingZHeight", registry);
   private final DoubleYoVariable desiredGaitPeriod = new DoubleYoVariable("desiredGaitPeriod", registry);
   private final DoubleYoVariable currentGaitCompletion = new DoubleYoVariable("currentGaitCompletion", registry);
   private final BooleanYoVariable gaitCompleted = new BooleanYoVariable("gaitCompleted", registry);
   private final DoubleYoVariable swingImpactVelocityZ = new DoubleYoVariable("impactVelocityZ", registry);
   private final YoFramePoint centroid = new YoFramePoint("centroid", ReferenceFrame.getWorldFrame(), registry);
   private final YoQuadrupedSupportPolygon previousSupportPolygon = new YoQuadrupedSupportPolygon("previousSupportPolygon", registry);
   private final YoQuadrupedSupportPolygon currentSupportPolygon = new YoQuadrupedSupportPolygon("currentSupportPolygon", registry);
   private final YoQuadrupedSupportPolygon nextSupportPolygon = new YoQuadrupedSupportPolygon("nextSupportPolygon", registry);
   private final YoQuadrupedSupportPolygon virtualEffectivePolygon = new YoQuadrupedSupportPolygon("virtualEffectivePolygon", registry);
   private final YoFrameConvexPolygon2d previousYoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("previousYoFrameConvexPolygon2d", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d currentYoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("currentYoFrameConvexPolygon2d", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d nextYoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("nextYoFrameConvexPolygon2d", ReferenceFrame.getWorldFrame(), 4, registry);
   private final VelocityConstrainedPositionTrajectoryGenerator icpTrajectory = new VelocityConstrainedPositionTrajectoryGenerator("icpTrajectory", ReferenceFrame.getWorldFrame(), registry);
   
   // Experimental Trot Stuff
   private final FramePoint rightMidpoint = new FramePoint();
   private final FramePoint leftMidpoint = new FramePoint();
   private final FramePoint frontMidpoint = new FramePoint();
   private final FramePoint hindMidpoint = new FramePoint();
   private final FrameVector2d frontDirection = new FrameVector2d();
   private final FrameLine2d verticalMidLine = new FrameLine2d(ReferenceFrame.getWorldFrame());
   private final FrameLine2d horizontalMidLine = new FrameLine2d(ReferenceFrame.getWorldFrame());
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
   
   // Swing PD Controllers
   private final YoFrameVector kp_swing = new YoFrameVector("kp_swing_", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector kd_swing = new YoFrameVector("kd_swing_", ReferenceFrame.getWorldFrame(), registry);
   private final QuadrantDependentList<YoFramePoint> desiredFootPositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFrameVector> desiredFootVelocities = new QuadrantDependentList<>();
   
   private static final EnumMap<QuadrupedGaitCycle, Double> gaitCyclePeriodMap = new EnumMap<>(QuadrupedGaitCycle.class);
   {
      gaitCyclePeriodMap.put(QuadrupedGaitCycle.STAND, 0.5);
      gaitCyclePeriodMap.put(QuadrupedGaitCycle.SAFE_WALK, 15.0);
      gaitCyclePeriodMap.put(QuadrupedGaitCycle.PERFECT_TROT, 0.7); // 0.30);
   }

   public QuadrupedTrotWalkController(QuadrupedRobotParameters robotParameters, SDFFullRobotModel fullRobotModel, QuadrantDependentList<FootSwitchInterface> footSwitches, double DT,
         DoubleYoVariable yoTime, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(QuadrupedControllerState.TROT_WALK);
      this.fullRobotModel = fullRobotModel;
      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, robotParameters.getJointMap(), robotParameters.getPhysicalProperties());
      this.dt = DT;
      this.yoTime = yoTime;
      
      centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      QuadrupedJointNameMap quadrupedJointMap = robotParameters.getJointMap();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ArrayList<OneDoFJoint> jointsToControl = new ArrayList<OneDoFJoint>();
         String jointBeforeFootName = quadrupedJointMap.getJointBeforeFootName(robotQuadrant);
         OneDoFJoint oneDoFJointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         fullRobotModel.getOneDoFJointsFromRootToHere(oneDoFJointBeforeFoot, jointsToControl);
         oneDofJoints.set(robotQuadrant, jointsToControl);
      }
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         vmcFootForces.set(robotQuadrant, new YoFrameVector("vmcFootForces" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoFramePoint footPosition = new YoFramePoint("footPosition" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry);
         footPositions.set(robotQuadrant, footPosition);
         footVelocities.set(robotQuadrant, new YoFrameVector("footVelocity" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
      }
      
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         basisForceVectors.set(robotQuadrant, new YoFrameVector[4]);
         basisTorqueVectors.set(robotQuadrant, new YoFrameVector[4]);
         rhoScalars.set(robotQuadrant, new double[4]);
         footToCoMVectors.set(robotQuadrant, new YoFrameVector("footToCoMVector" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
         
         for (int i = 0; i < 4; i++)
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
         swingZTrajectories.set(robotQuadrant, new YoPolynomial("swingZTrajectory" + robotQuadrant.getPascalCaseName(), 4, registry));
         swingInitialZHeights.set(robotQuadrant, new DoubleYoVariable("swingInitialZHeights" + robotQuadrant.getPascalCaseName(), registry));
         gaitCompletionAtStartOfSwing.set(robotQuadrant, new DoubleYoVariable("swingStartTime" + robotQuadrant.getPascalCaseName(), registry));
         swingDurations.set(robotQuadrant, new DoubleYoVariable("swingDurations" + robotQuadrant.getPascalCaseName(), registry));
         desiredFootPositions.set(robotQuadrant, new YoFramePoint("desiredFootPosition" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
         desiredFootVelocities.set(robotQuadrant, new YoFrameVector("desiredFootVelocity" + robotQuadrant.getPascalCaseName(), ReferenceFrame.getWorldFrame(), registry));
      }
      
      yoGraphicsListRegistry.hideYoGraphics();
      yoGraphicsListRegistry.hideArtifacts();
      
      if (CREATE_VISUALIZATIONS)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            yoGraphicsListRegistry.registerYoGraphic(getName() + "FootForces", new YoGraphicVector("vmcFootForce" + robotQuadrant.getPascalCaseName(), footPositions.get(robotQuadrant), vmcFootForces.get(robotQuadrant), 0.007, YoAppearance.Yellow(), true, 0.01));
            yoGraphicsListRegistry.registerArtifact(getName() + "FootPositions", new YoArtifactPosition("footPosition" + robotQuadrant.getPascalCaseName() + "Vis", footPositions.get(robotQuadrant).getYoX(), footPositions.get(robotQuadrant).getYoY(), GraphicType.BALL_WITH_CROSS, YoAppearance.Color(robotQuadrant.getColor()).getAwtColor(), 0.02));
            
            for (int i = 0; i < 4; i++)
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
         yoGraphicsListRegistry.registerArtifact(getName() + "IcpTrajectory", new YoArtifactLine("icpTrajectory", icpTrajectory.getInitialPosition(), icpTrajectory.getFinalPosition(), Color.BLUE));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotWalkPolygons", new YoArtifactPolygon("previousYoArtifactPolygon", previousYoFrameConvexPolygon2d, Color.GRAY, false, 3));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotWalkPolygons", new YoArtifactPolygon("currentYoArtifactPolygon", currentYoFrameConvexPolygon2d, Color.BLUE, false, 2));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotWalkPolygons", new YoArtifactPolygon("nextYoArtifactPolygon", nextYoFrameConvexPolygon2d, new Color(0, 100, 0), false, 1));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactLineSegment2d("midPointOfIntersectionsToClosestIntersection", midPointOfIntersections, closestIntersection, Color.GREEN));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactPosition("closestIntersectionVis", closestIntersection.getYoX(), closestIntersection.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.DarkRed().getAwtColor(), 0.003));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactPosition("secondClosestIntersectionVis", secondClosestIntersection.getYoX(), secondClosestIntersection.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.DarkSlateBlue().getAwtColor(), 0.003));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactPosition("innerCenterOfPressureVis", innerCenterOfPressure.getYoX(), innerCenterOfPressure.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.DarkMagenta().getAwtColor(), 0.003));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactPosition("outerCenterOfPressureVis", outerCenterOfPressure.getYoX(), outerCenterOfPressure.getYoY(), GraphicType.BALL_WITH_ROTATED_CROSS, YoAppearance.DarkBlue().getAwtColor(), 0.003));
         yoGraphicsListRegistry.registerArtifact(getName() + "TrotGeometry", new YoArtifactLineSegment2d("innerToOuterCenterOfPressure", innerCenterOfPressure, outerCenterOfPressure, Color.BLUE));
         yoGraphicsListRegistry.registerYoGraphic(getName() + "BodyPose", new YoGraphicPosition("stancePoseHigherForVis", stancePoseHigherForVis, 0.03, YoAppearance.Blue()));
         yoGraphicsListRegistry.registerYoGraphic(getName() + "BodyPose", new YoGraphicPosition("desiredStancePoseHigherForVis", desiredStancePoseHigherForVis, 0.03, YoAppearance.Green()));
      }
      
      parentRegistry.addChild(registry);
   }

   @Override
   public void doTransitionIntoAction()
   {
      initialize();
   }

   public void initialize()
   {
      previousSupportPolygon.clear();
      currentSupportPolygon.clear();
      nextSupportPolygon.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         previousSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
         currentSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
         nextSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      
      updateEstimates();
      
      desiredStancePose.set(stancePose);
      desiredStancePoseOffset.setToZero();
      icpTrajectory.getFinalPosition().set(centroid);
      
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
      
      currentGait.set(QuadrupedGaitCycle.STAND);
      desiredGaitPeriod.set(gaitCyclePeriodMap.get(currentGait.getEnumValue()));
      desiredGait.set(currentGait.getEnumValue());
      nextGait.set(currentGait.getEnumValue());
      swingZHeight.set(0.1); //0.04);
      swingImpactVelocityZ.set(0.0);
      previousSupportPolygon.setWithoutChecks(currentSupportPolygon);
      previousGaitPhase.set(QuadrupedSupportConfiguration.ALL_FOURS);
      currentGaitPhase.set(QuadrupedSupportConfiguration.ALL_FOURS);
      currentGaitCompletion.set(0.0);
      
      handlePhaseChange();
   
      kp_icp.set(5.0);
      kd_icp.set(0.0);
      
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
   public void doAction()
   {
      initializeInheritedVariables();
      updateEstimates();
      
      checkGaitTransitionConditions();
      
      doSupportAndSwing();
      
      doControl();
      distributeForcesToFeet();
      computeStanceJacobians();
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
   
   private void updateEstimates()
   {
      //update frames
      referenceFrames.updateFrames();

      //update feet locations
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footVelocities.get(robotQuadrant).set(footPositions.get(robotQuadrant));
         footPositions.get(robotQuadrant).setFromReferenceFrame(referenceFrames.getFootFrame(robotQuadrant));
         footVelocities.get(robotQuadrant).sub(footPositions.get(robotQuadrant));
         footVelocities.get(robotQuadrant).scale(-1.0 / dt);
      }
      
      for (RobotQuadrant robotQuadrant : currentSupportPolygon.getSupportingQuadrantsInOrder())
      {
         currentSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      for (RobotQuadrant robotQuadrant : nextSupportPolygon.getSupportingQuadrantsInOrder())
      {
         nextSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      for (RobotQuadrant robotQuadrant : previousSupportPolygon.getSupportingQuadrantsInOrder())
      {
         previousSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      for (RobotQuadrant robotQuadrant : virtualEffectivePolygon.getSupportingQuadrantsInOrder())
      {
         virtualEffectivePolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      
      previousSupportPolygon.packYoFrameConvexPolygon2d(previousYoFrameConvexPolygon2d);
      currentSupportPolygon.packYoFrameConvexPolygon2d(currentYoFrameConvexPolygon2d);
      nextSupportPolygon.packYoFrameConvexPolygon2d(nextYoFrameConvexPolygon2d);
      
      bodyPoseWorld.setFromReferenceFrame(referenceFrames.getBodyFrame());
      
      if (q_z != null)
      {
         bodyPoseWorld.setZ(q_z.getDoubleValue() + SIMULATION_TO_ROBOT_MODEL_Z_DIFFERNCE);
      }
      
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
      centerOfMassVelocity.set(tempVector.getX(), tempVector.getY(), tempVector.getZ());
      
      FramePoint frameTuple = centroid.getFrameTuple();
      currentSupportPolygon.getCentroid(frameTuple);
      centroid.set(frameTuple.getX(), frameTuple.getY(), frameTuple.getZ());

      // compute instantaneous capture point
      double zDelta = centerOfMass.getZ() - centroid.getZ();
      double omega = Math.sqrt(GRAVITY / zDelta);
      icp.setX(centerOfMass.getX() + centerOfMassVelocity.getX() / omega);
      icp.setY(centerOfMass.getY() + centerOfMassVelocity.getY() / omega);
      
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
      
      currentGaitCompletion.set((yoTime.getDoubleValue() - gaitStartTime.getDoubleValue()) / desiredGaitPeriod.getDoubleValue());
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
      gaitStartTime.set(yoTime.getDoubleValue());
      currentGait.set(nextGait.getEnumValue());
      desiredGaitPeriod.set(gaitCyclePeriodMap.get(currentGait.getEnumValue()));
   }

   private void handlePhaseChange()
   {
      // Update phases
      phaseStartTime.set(yoTime.getDoubleValue());
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
      
      // Update Z swing parameters
      for (RobotQuadrant robotQuadrant : previousGaitPhase.getEnumValue().supportQuadrants())
      {
         swingInitialZHeights.get(robotQuadrant).set(footPositions.get(robotQuadrant).getZ());
         gaitCompletionAtStartOfSwing.get(robotQuadrant).set(currentGaitCompletion.getDoubleValue());
         
         // Measure durations of legs switching to swing
         if (currentGaitPhase.getEnumValue().isSwingQuadrant(robotQuadrant))
         {
            swingDurations.get(robotQuadrant).set(currentGait.getEnumValue().getRemainingSwingDuration(robotQuadrant, currentGaitCompletion.getDoubleValue()));
         }
      }
      
      // Update support polygons
      previousSupportPolygon.setWithoutChecks(currentSupportPolygon);
      currentSupportPolygon.clear();
      for (RobotQuadrant robotQuadrant : currentGaitPhase.getEnumValue().supportQuadrants())
      {
         currentSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      FramePoint frameTuple = centroid.getFrameTuple();
      currentSupportPolygon.getCentroid(frameTuple);
      centroid.set(frameTuple.getX(), frameTuple.getY(), frameTuple.getZ());
      nextSupportPolygon.clear();
      for (RobotQuadrant robotQuadrant : nextGaitPhase.getEnumValue().supportQuadrants())
      {
         nextSupportPolygon.setFootstep(robotQuadrant, footPositions.get(robotQuadrant).getFrameTuple());
      }
      virtualEffectivePolygon.setWithoutChecks(currentSupportPolygon);
      if (currentSupportPolygon.size() < 3)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (!currentSupportPolygon.containsFootstep(robotQuadrant) && nextSupportPolygon.containsFootstep(robotQuadrant))
            {
               virtualEffectivePolygon.setFootstep(robotQuadrant, nextSupportPolygon.getFootstep(robotQuadrant));
            }
         }
      }
      
      // Update desired ICP trajectory
      icpTrajectory.setTrajectoryTime(currentGait.getEnumValue().getRemainingPhaseDuration(currentGaitCompletion.getDoubleValue()));
      icpTrajectory.getInitialPosition().set(icpTrajectory.getFinalPosition());
      FramePoint nextCentroid = icpTrajectory.getFinalPosition().getFrameTuple();
      if (currentGaitPhase.getEnumValue() == QuadrupedSupportConfiguration.ALL_FOURS)
      {
         nextSupportPolygon.getCentroid(nextCentroid);
      }
      else
      {
         currentSupportPolygon.getCentroid(nextCentroid);
      }
      icpTrajectory.getFinalPosition().set(nextCentroid.getX(), nextCentroid.getY(), 0.0);
      icpTrajectory.getInitialVelocity().setToZero();
      icpTrajectory.getFinalVelocity().setToZero();
      icpTrajectory.initialize();
   }

   private void doSupportAndSwing()
   {
      for (RobotQuadrant robotQuadrant : currentGaitPhase.getEnumValue().swingQuadrants())
      {
         // Set swing Z
         double timeStart = 0.0;
         double timePeak = swingDurations.get(robotQuadrant).getDoubleValue() / 2.0;
         double timeEnd = swingDurations.get(robotQuadrant).getDoubleValue();
         double zStart = swingInitialZHeights.get(robotQuadrant).getDoubleValue();
         double zMid = swingZHeight.getDoubleValue();
         double zEnd = zStart;
         double zVelocityFinal = swingImpactVelocityZ.getDoubleValue();
         double phaseCompletion = currentGaitCompletion.getDoubleValue() - gaitCompletionAtStartOfSwing.get(robotQuadrant).getDoubleValue();
         swingZTrajectories.get(robotQuadrant).setCubicWithIntermediatePositionAndFinalVelocityConstraint(timeStart, timePeak, timeEnd, zStart, zMid, zEnd, zVelocityFinal);
         swingZTrajectories.get(robotQuadrant).compute(phaseCompletion);
         
         desiredFootPositions.get(robotQuadrant).setZ(swingZTrajectories.get(robotQuadrant).getPosition());
         desiredFootVelocities.get(robotQuadrant).setZ(swingZTrajectories.get(robotQuadrant).getVelocity());
         desiredFootVelocities.get(robotQuadrant).setToZero();
         
         double startX = previousSupportPolygon.getFootstep(robotQuadrant).getX();
         double startY = previousSupportPolygon.getFootstep(robotQuadrant).getY();
         double endX = nextSupportPolygon.getFootstep(robotQuadrant).getX();
         double endY = nextSupportPolygon.getFootstep(robotQuadrant).getY();
         
         desiredFootPositions.get(robotQuadrant).setX((endX - startX) * phaseCompletion + startX);
         desiredFootPositions.get(robotQuadrant).setY((endY- startY) * phaseCompletion + startY);
      }
      
      icpTrajectory.compute((yoTime.getDoubleValue() - phaseStartTime.getDoubleValue()) / desiredGaitPeriod.getDoubleValue());
   }
   
   private void findTrotSpecificCenterOfPressure()
   {
      virtualEffectivePolygon.getFrontMidpoint(frontMidpoint);
      virtualEffectivePolygon.getRightMidpoint(rightMidpoint);
      virtualEffectivePolygon.getHindMidpoint(hindMidpoint);
      virtualEffectivePolygon.getLeftMidpoint(leftMidpoint);

      frontDirection.setByProjectionOntoXYPlane(frontMidpoint);
      frontDirection.sub(hindMidpoint.getX(), hindMidpoint.getY());

      verticalMidLine.setByProjectionOntoXYPlane(leftMidpoint, rightMidpoint);
      horizontalMidLine.setByProjectionOntoXYPlane(frontMidpoint, hindMidpoint);

      lineSegmentLeftTrot.setByProjectionOntoXYPlane(footPositions.get(RobotQuadrant.HIND_RIGHT).getFrameTuple(), footPositions.get(RobotQuadrant.FRONT_LEFT).getFrameTuple());
      lineSegmentRightTrot.setByProjectionOntoXYPlane(footPositions.get(RobotQuadrant.HIND_LEFT).getFrameTuple(), footPositions.get(RobotQuadrant.FRONT_RIGHT).getFrameTuple());

      leftTrotLine.setByProjectionOntoXYPlane(footPositions.get(RobotQuadrant.HIND_RIGHT).getFrameTuple(), footPositions.get(RobotQuadrant.FRONT_LEFT).getFrameTuple());
      rightTrotLine.setByProjectionOntoXYPlane(footPositions.get(RobotQuadrant.HIND_LEFT).getFrameTuple(), footPositions.get(RobotQuadrant.FRONT_RIGHT).getFrameTuple());
      
      leftTrotLine.getIntersectionWithLine(rightTrotLine, trotCrossPoint);

      boolean isDesiredCoPInFrontOfLeftTrotLine = leftTrotLine.isPointInFrontOfLine(frontDirection, desiredCenterOfPressure.getFrameTuple2d());
      boolean isDesiredCoPInFrontOfRightTrotLine = rightTrotLine.isPointInFrontOfLine(frontDirection, desiredCenterOfPressure.getFrameTuple2d());

      lineForFindingClosestLineSegment.setOrigin(desiredCenterOfPressure.getFrameTuple2d());

      if (isDesiredCoPInFrontOfLeftTrotLine == isDesiredCoPInFrontOfRightTrotLine)
      {
         lineForFindingClosestLineSegment.getLine2d().getNormalizedVector().set(verticalMidLine.getLine2d().getNormalizedVector());
      }
      else
      {
         lineForFindingClosestLineSegment.getLine2d().getNormalizedVector().set(horizontalMidLine.getLine2d().getNormalizedVector());
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
         snappedDesiredCenterOfPressure.set(innerCenterOfPressure);
      }
      else
      {
         snappedDesiredCenterOfPressure.set(outerCenterOfPressure);
      }
      
      currentSupportPolygon.snapPointToClosestEdgeOfPolygonIfOutside2d(snappedDesiredCenterOfPressure);
   }

   private void doControl()
   {
      icpTrajectory.getProjectedOntoXYPlane(desiredICP);
      
      desiredICP.add(desiredStancePoseOffset.getX(), desiredStancePoseOffset.getY());

      desiredCenterOfPressure.setToZero();
      desiredCenterOfPressure.set(icp);
      desiredCenterOfPressure.sub(desiredICP);
      desiredCenterOfPressure.scale(kp_icp.getDoubleValue());
      desiredCenterOfPressure.add(icp);
      
      if (currentGait.getEnumValue() == QuadrupedGaitCycle.PERFECT_TROT)
      {
         findTrotSpecificCenterOfPressure();
      }
      else
      {
         snappedDesiredCenterOfPressure.set(desiredCenterOfPressure);
      }

      desiredStancePose.setToZero();
      desiredStancePose.setX(desiredICP.getX());
      desiredStancePose.setY(desiredICP.getY());
      desiredStancePose.setZ(INITIAL_STANCE_HEIGHT + desiredStancePoseOffset.getZ());
      desiredStancePose.getOrientation().add(desiredStancePoseOffset.getOrientation());
      
      desiredStancePoseHigherForVis.set(desiredStancePose.getPosition());
      desiredStancePoseHigherForVis.add(0.0, 0.0, 0.3);

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

   private void calculateBasisVectors(RobotQuadrant robotQuadrant, YoFrameVector footToCoMVector)
   {
      basisForceVectors.get(robotQuadrant)[0].set(COEFFICIENT_OF_FRICTION, 0.0, 1.0);
      basisForceVectors.get(robotQuadrant)[0].normalize();
      basisForceVectors.get(robotQuadrant)[1].set(0.0, COEFFICIENT_OF_FRICTION, 1.0);
      basisForceVectors.get(robotQuadrant)[1].normalize();
      basisForceVectors.get(robotQuadrant)[2].set(-COEFFICIENT_OF_FRICTION, 0.0, 1.0);
      basisForceVectors.get(robotQuadrant)[2].normalize();
      basisForceVectors.get(robotQuadrant)[3].set(0.0, -COEFFICIENT_OF_FRICTION, 1.0);
      basisForceVectors.get(robotQuadrant)[3].normalize();
      
      basisTorqueVectors.get(robotQuadrant)[0].cross(footToCoMVector, basisForceVectors.get(robotQuadrant)[0]);
      basisTorqueVectors.get(robotQuadrant)[0].normalize();
      basisTorqueVectors.get(robotQuadrant)[1].cross(footToCoMVector, basisForceVectors.get(robotQuadrant)[1]);
      basisTorqueVectors.get(robotQuadrant)[1].normalize();
      basisTorqueVectors.get(robotQuadrant)[2].cross(footToCoMVector, basisForceVectors.get(robotQuadrant)[2]);
      basisTorqueVectors.get(robotQuadrant)[2].normalize();
      basisTorqueVectors.get(robotQuadrant)[3].cross(footToCoMVector, basisForceVectors.get(robotQuadrant)[3]);
      basisTorqueVectors.get(robotQuadrant)[3].normalize();
   }
   
   private void distributeForcesToFeet()
   {
      clearFootForces();
      distributeForcesToSupportFeet();
      distributeForcesToSwingFeet();
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
   
   private void distributeForcesToSupportFeet()
   {
      // New QP stuff here
      if (USE_COPX_AND_COPY)
      {
         bodyWrenchMatrix.set(0, 0, snappedDesiredCenterOfPressure.getX());
         bodyWrenchMatrix.set(1, 0, snappedDesiredCenterOfPressure.getY());
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
      
      basisMatrix.reshape(6, currentSupportPolygon.size() * 4);
      rhoMatrix.reshape(currentSupportPolygon.size() * 4, 1);
      
      for (int quadrantIndex = 0; quadrantIndex < currentSupportPolygon.getSupportingQuadrantsInOrder().length; quadrantIndex++)
      {
         RobotQuadrant robotQuadrant = currentSupportPolygon.getSupportingQuadrantsInOrder()[quadrantIndex];
         for (int basisIndex = 0; basisIndex < 4; basisIndex++)
         {
            if (USE_COPX_AND_COPY)
            {
               basisMatrix.set(0, quadrantIndex * 4 + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getZ() * footPositions.get(robotQuadrant).getX() / desiredBodyWrench.getLinearPartZ());
               basisMatrix.set(1, quadrantIndex * 4 + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getZ() * footPositions.get(robotQuadrant).getY() / desiredBodyWrench.getLinearPartZ());
            }
            else
            {
               basisMatrix.set(0, quadrantIndex * 4 + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getX());
               basisMatrix.set(1, quadrantIndex * 4 + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getY());
            }
            basisMatrix.set(2, quadrantIndex * 4 + basisIndex, basisForceVectors.get(robotQuadrant)[basisIndex].getZ());
            basisMatrix.set(3, quadrantIndex * 4 + basisIndex, basisTorqueVectors.get(robotQuadrant)[basisIndex].getX());
            basisMatrix.set(4, quadrantIndex * 4 + basisIndex, basisTorqueVectors.get(robotQuadrant)[basisIndex].getY());
            basisMatrix.set(5, quadrantIndex * 4 + basisIndex, basisTorqueVectors.get(robotQuadrant)[basisIndex].getZ());
         }
      }
      
      solver.setA(basisMatrix);
      solver.solve(bodyWrenchMatrix, rhoMatrix);
      
      for (int quadrantIndex = 0; quadrantIndex < currentSupportPolygon.getSupportingQuadrantsInOrder().length; quadrantIndex++)
      {
         RobotQuadrant robotQuadrant = currentSupportPolygon.getSupportingQuadrantsInOrder()[quadrantIndex];
         for (int basisIndex = 0; basisIndex < 4; basisIndex++)
         {
            rhoScalars.get(robotQuadrant)[basisIndex] = rhoMatrix.get(quadrantIndex * 4 + basisIndex, 0);
         }
      }
      
      for (RobotQuadrant robotQuadrant : currentSupportPolygon.getSupportingQuadrantsInOrder())
      {
         for (int basisIndex = 0; basisIndex < 4; basisIndex++)
         {
            rhoScalars.get(robotQuadrant)[basisIndex] = MathTools.clipToMinMax(rhoScalars.get(robotQuadrant)[basisIndex], 400.0); // TODO Don't clip rhos
            basisForceVectors.get(robotQuadrant)[basisIndex].scale(rhoScalars.get(robotQuadrant)[basisIndex]);
            vmcFootForces.get(robotQuadrant).add(basisForceVectors.get(robotQuadrant)[basisIndex]);
         }
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

   private void clearFootForces()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         vmcFootForces.get(robotQuadrant).setToZero();
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
   public void doTransitionOutOfAction()
   {

   }

   @Override
   public RobotMotionStatus getMotionStatus()
   {
      return RobotMotionStatus.IN_MOTION;
   }
}
