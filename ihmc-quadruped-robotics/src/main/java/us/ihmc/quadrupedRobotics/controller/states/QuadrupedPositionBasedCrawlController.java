package us.ihmc.quadrupedRobotics.controller.states;

import java.awt.Color;
import java.util.Random;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.mechanics.inverseKinematics.QuadrupedInverseKinematicsCalculators;
import us.ihmc.quadrupedRobotics.mechanics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.quadrupedRobotics.model.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.MidFootZUpSwingTargetGenerator;
import us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser.SwingTargetGenerator;
import us.ihmc.quadrupedRobotics.planning.chooser.swingLegChooser.DefaultGaitSwingLegChooser;
import us.ihmc.quadrupedRobotics.planning.chooser.swingLegChooser.NextSwingLegChooser;
import us.ihmc.quadrupedRobotics.planning.trajectory.QuadrupedSwingTrajectoryGenerator;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPlanarVelocityInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProvider;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.filters.AlphaFilteredWrappingYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RecyclingQuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoFrameLineSegment2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;
import us.ihmc.yoVariables.variable.YoVariable;

public class QuadrupedPositionBasedCrawlController implements QuadrupedController
{
   private static final double MAX_YAW_IN_PLACE = 0.4;

   private static final double DEFAULT_HEADING_CORRECTION_BREAK_FREQUENCY = 1.0;
   private static final double DEFAULT_YAW_IN_PLACE_RATE_LIMIT = 0.2;
   private static final double DEFAULT_COM_PITCH_FILTER_BREAK_FREQUENCY = 0.5;
   private static final double DEFAULT_COM_ROLL_FILTER_BREAK_FREQUENCY = 0.5;
   private static final double DEFAULT_COM_HEIGHT_Z_FILTER_BREAK_FREQUENCY = 0.5;
   private static final double DEFAULT_TIME_TO_STAY_IN_DOUBLE_SUPPORT = 0.01;

   private final double dt;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoDouble robotTimestamp;

   public enum CrawlGateWalkingState
   {
      QUADRUPLE_SUPPORT, TRIPLE_SUPPORT, ALPHA_FILTERING_DESIREDS
   }

   private enum SafeStartingShiftMode
   {
      COMMON_TRIANGLE, CENTROID, TTR, COM_INCIRCLE, TROTLINE_MIDPOINT
   }

   ;

   private final YoEnum<SafeStartingShiftMode> safeToShiftMode = new YoEnum<>("safeStartingShiftMode", registry, SafeStartingShiftMode.class);

   {
      safeToShiftMode.set(SafeStartingShiftMode.TROTLINE_MIDPOINT);
   }

   private final FullQuadrupedRobotModel feedForwardFullRobotModel;
   private final QuadrupedReferenceFrames feedForwardReferenceFrames;

   private final StateMachine<CrawlGateWalkingState, State> walkingStateMachine;
   private final QuadrupleSupportState quadrupleSupportState;
   private final FilterDesiredsToMatchCrawlControllerState filterDesiredsToMatchCrawlControllerOnTransitionIn;
   private final QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators;
   private final NextSwingLegChooser nextSwingLegChooser;
   private final SwingTargetGenerator swingTargetGenerator;
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;
   private final FullQuadrupedRobotModel actualFullRobotModel;
   private final FloatingInverseDynamicsJoint actualRobotRootJoint;

   private final QuadrupedReferenceFrames referenceFrames;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final CenterOfMassJacobian feedForwardCenterOfMassJacobian;
   private final FramePoint3D feedForwardCoMPosition = new FramePoint3D();
   private final FramePoint3D tempCoMPosition = new FramePoint3D();
   private final ReferenceFrame feedForwardBodyFrame;
   private final ReferenceFrame comFrame;
   private final YoDouble feedForwardCenterOfMassOffsetAlpha;
   private final YoFramePoint3D feedForwardCenterOfMassOffset;
   private final AlphaFilteredYoFramePoint filteredFeedForwardCenterOfMassOffset;

   private final TranslationReferenceFrame feedForwardCenterOfMassFrame;
   private final PoseReferenceFrame desiredCoMPoseReferenceFrame = new PoseReferenceFrame("desiredCoMPoseReferenceFrame", ReferenceFrame.getWorldFrame());
   private final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D desiredCoMVelocity = new YoFrameVector3D("desiredCoMVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final ExecutionTimer calculateNextThreeFootstepsTimer = new ExecutionTimer("calculateNextThreeFootstepsTimer", registry);
   private final ExecutionTimer doTransitionIntoQuadrupleSupportTimer = new ExecutionTimer("doTransitionIntoQuadrupleSupportTimer", registry);
   private final ExecutionTimer doTransitionIntoTripleSupportTimer = new ExecutionTimer("doTransitionIntoTripleSupportTimer", registry);
   private final ExecutionTimer doActionTripleSupportTimer = new ExecutionTimer("doActionTripleSupportTimer", registry);
   private final ExecutionTimer doActionQuadrupleSupportTimer = new ExecutionTimer("doActionQuadrupleSupportTimer", registry);
   private final ExecutionTimer doActionTimer = new ExecutionTimer("doActionTimer", registry);
   private final ExecutionTimer calculateNextCoMTargetTimer = new ExecutionTimer("calculateNextCoMTargetTimer", registry);
   private final ExecutionTimer calculateNextThreeFootStepsOneOffTimer = new ExecutionTimer("calculateNextThreeFootStepsOneOffTimer", registry);
   private boolean oneOffHappened = false;

   private final OneDoFJoint[] oneDoFJointsFeedforward;
   private final OneDoFJoint[] oneDoFJointsActual;

   private final FramePoint3D desiredCoMFramePosition = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final FramePose3D desiredCoMFramePose = new FramePose3D(ReferenceFrame.getWorldFrame());

   private final FrameVector3D tempComTrajComputedVelocity = new FrameVector3D();
   private final FrameVector3D tempDesiredVelocityVector = new FrameVector3D();

   private final YoBoolean runOpenLoop = new YoBoolean("runOpenLoop",
                                                       "If true, runs in open loop mode. The leg motions will not depend on any feedback signals.", registry);

   private final YoFramePoint2D desiredCoMOffset;
   private final YoDouble distanceInsideSupportPolygonBeforeSwingingLeg = new YoDouble("distanceInsideSupportPolygonBeforeSwingingLeg", registry);

   private final YoDouble filteredDesiredCoMYawAlphaBreakFrequency = new YoDouble("filteredDesiredCoMYawAlphaBreakFrequency", registry);
   private final YoDouble filteredDesiredCoMYawAlpha = new YoDouble("filteredDesiredCoMYawAlpha", registry);

   private final YoDouble filteredDesiredCoMPitchAlphaBreakFrequency = new YoDouble("filteredDesiredCoMPitchAlphaBreakFrequency", registry);
   private final YoDouble filteredDesiredCoMPitchAlpha = new YoDouble("filteredDesiredCoMOrientationAlpha", registry);

   private final YoDouble filteredDesiredCoMRollAlphaBreakFrequency = new YoDouble("filteredDesiredCoMRollAlphaBreakFrequency", registry);
   private final YoDouble filteredDesiredCoMRollAlpha = new YoDouble("filteredDesiredCoMRollAlpha", registry);

   private final YoDouble desiredCoMHeight = new YoDouble("desiredCoMHeight", registry);
   private final YoDouble filteredDesiredCoMHeightAlphaBreakFrequency = new YoDouble("filteredDesiredCoMHeightAlphaBreakFrequency", registry);
   private final YoDouble filteredDesiredCoMHeightAlpha = new YoDouble("filteredDesiredCoMHeightAlpha", registry);
   private final AlphaFilteredYoVariable filteredDesiredCoMHeight = new AlphaFilteredYoVariable("filteredDesiredCoMHeight", registry,
                                                                                                filteredDesiredCoMHeightAlpha, desiredCoMHeight);

   private final YoFrameYawPitchRoll desiredCoMOrientation = new YoFrameYawPitchRoll("desiredCoMOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final AlphaFilteredWrappingYoVariable filteredDesiredCoMYaw = new AlphaFilteredWrappingYoVariable("filteredDesiredCoMYaw", "", registry, desiredCoMOrientation.getYaw(), filteredDesiredCoMYawAlpha, -Math.PI, Math.PI);
   private final AlphaFilteredWrappingYoVariable filteredDesiredCoMPitch = new AlphaFilteredWrappingYoVariable("filteredDesiredCoMPitch", "", registry, desiredCoMOrientation.getPitch(), filteredDesiredCoMPitchAlpha, -Math.PI, Math.PI);
   private final AlphaFilteredWrappingYoVariable filteredDesiredCoMRoll = new AlphaFilteredWrappingYoVariable("filteredDesiredCoMRoll", "", registry, desiredCoMOrientation.getRoll(), filteredDesiredCoMRollAlpha, -Math.PI, Math.PI);
   private final YoDouble desiredCoMYaw = new YoDouble("desiredCoMYaw", registry);
   private final YoDouble desiredCoMPitch = new YoDouble("desiredCoMPitch", registry);
   private final YoDouble desiredCoMRoll = new YoDouble("desiredCoMRoll", registry);
   private final YoDouble actualYaw = new YoDouble("actualYaw", registry);
   private final YoDouble actualPitch = new YoDouble("actualPitch", registry);
   private final YoDouble actualRoll = new YoDouble("actualRoll", registry);
   private final PIDController pitchPidController = new PIDController("pitchPidController", registry);
   private final PIDController rollPidController = new PIDController("rollPidController", registry);
//   private final YoFrameOrientation filteredDesiredCoMOrientation = new YoFrameOrientation(filteredDesiredCoMYaw, filteredDesiredCoMPitch, filteredDesiredCoMRoll, ReferenceFrame.getWorldFrame());
   private final YoFrameYawPitchRoll filteredDesiredCoMOrientation = new YoFrameYawPitchRoll(desiredCoMYaw, desiredCoMPitch, desiredCoMRoll, ReferenceFrame.getWorldFrame());
   private final YoFramePoseUsingYawPitchRoll desiredCoMPose = new YoFramePoseUsingYawPitchRoll(desiredCoMPosition, filteredDesiredCoMOrientation);
   private final YoBoolean useImuFeedback = new YoBoolean("useImuFeedback", registry);

   private final YoEnum<RobotQuadrant> swingLeg = new YoEnum<RobotQuadrant>("swingLeg", registry, RobotQuadrant.class, true);
   private final YoFrameVector3D desiredVelocity;
   private final YoFrameVector3D lastDesiredVelocity;
   private final FrameVector3D desiredBodyVelocity = new FrameVector3D();
   private final YoDouble maxYawRate = new YoDouble("maxYawRate", registry);
   private final YoDouble minYawRate = new YoDouble("minYawRate", registry);
   private final YoDouble desiredYawRate = new YoDouble("desiredYawRate", registry);
   private final YoDouble lastDesiredYawRate = new YoDouble("lastDesiredYawRate", registry);

   private final YoDouble shrunkenPolygonSize = new YoDouble("shrunkenPolygonSize", registry);

   private final YoDouble nominalYaw = new YoDouble("nominalYaw", registry);

   private final YoDouble desiredYawInPlace = new YoDouble("desiredYawInPlace", registry);
   private final YoDouble desiredYawInPlaceRateLimit = new YoDouble("desiredYawInPlaceRateLimit", registry);
   private final RateLimitedYoVariable desiredYawInPlaceRateLimited;

   private final YoFrameLineSegment2D nominalYawLineSegment = new YoFrameLineSegment2D("nominalYawLineSegment", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoArtifactLineSegment2d nominalYawArtifact = new YoArtifactLineSegment2d("nominalYawArtifact", nominalYawLineSegment, Color.YELLOW, 0.02, 0.02);
   private final FramePoint2D endPoint2d = new FramePoint2D();
   private final FramePoint3D centroidFramePoint = new FramePoint3D();
   private final FramePoint2D centroidFramePoint2d = new FramePoint2D();

   private final QuadrupedSupportPolygon safeToStepSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon currentSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon commonSupportPolygon = new QuadrupedSupportPolygon();
   private final RecyclingQuadrantDependentList<FramePoint3D> tempSupportPolygonFramePointHolder = new RecyclingQuadrantDependentList<FramePoint3D>(
         FramePoint3D.class);
   private final QuadrupedSupportPolygon tempCommonShrunkenPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon tempPolygonForCommonShrunken = new QuadrupedSupportPolygon();

   private final QuadrantDependentList<QuadrupedSwingTrajectoryGenerator> swingTrajectoryGenerators = new QuadrantDependentList<>();
   private final YoDouble swingDuration = new YoDouble("swingDuration", registry);
   private final YoDouble swingHeight = new YoDouble("swingHeight", registry);
   private final YoDouble swingTimeRemaining = new YoDouble("swingTimeRemaining", registry);

   private final YoDouble distanceInside = new YoDouble("distanceInside", registry);

   private final QuadrantDependentList<ReferenceFrame> legAttachmentFrames = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint3D> actualFeetLocations = new QuadrantDependentList<YoFramePoint3D>();
   private final QuadrantDependentList<YoFramePoint3D> desiredFeetLocations = new QuadrantDependentList<YoFramePoint3D>();
   private final FramePoint3D desiredFootPosition = new FramePoint3D();
   private final FramePoint3D desiredFootPositionInBody = new FramePoint3D();

   private final QuadrantDependentList<YoFrameVector3D> desiredFeetPositionsInLegAttachmentFrame = new QuadrantDependentList<YoFrameVector3D>();
   private final QuadrantDependentList<YoFrameVector3D> actualFeetPositionsInLegAttachmentFrame = new QuadrantDependentList<YoFrameVector3D>();
   private final Vector3D desiredFootPositionForInverseKinematics = new Vector3D();

   private final YoFrameConvexPolygon2D supportPolygon = new YoFrameConvexPolygon2D("quadPolygon", "", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2D currentTriplePolygon = new YoFrameConvexPolygon2D("currentTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameConvexPolygon2D upcomingTriplePolygon = new YoFrameConvexPolygon2D("upcomingTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameConvexPolygon2D commonTriplePolygon = new YoFrameConvexPolygon2D("commonTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);

   private final YoFrameConvexPolygon2D commonTriplePolygonLeft = new YoFrameConvexPolygon2D("commonTriplePolygonLeft", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameConvexPolygon2D commonTriplePolygonRight = new YoFrameConvexPolygon2D("commonTriplePolygonRight", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final SideDependentList<YoFrameConvexPolygon2D> commonTriplePolygons = new SideDependentList<>(commonTriplePolygonLeft, commonTriplePolygonRight);
//   private final YoFrameConvexPolygon2d[] tripleSupportPolygons = new YoFrameConvexPolygon2d[6];
//   private final YoArtifactPolygon[] tripleSupportArtifactPolygons = new YoArtifactPolygon[6];

   private final QuadrantDependentList<YoFrameConvexPolygon2D> tripleSupportPolygons = new QuadrantDependentList<YoFrameConvexPolygon2D>();
   private final QuadrantDependentList<YoArtifactPolygon> tripleSupportArtifactPolygons = new QuadrantDependentList<YoArtifactPolygon>();

   private final YoFramePoint3D circleCenter = new YoFramePoint3D("circleCenter", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint2D circleCenter2d = new FramePoint2D();
   private final FramePoint3D circleCenter3d = new FramePoint3D();
   private final YoGraphicPosition circleCenterGraphic = new YoGraphicPosition("circleCenterGraphic", circleCenter, 0.005, YoAppearance.Green());

   private final YoDouble inscribedCircleRadius = new YoDouble("inscribedCircleRadius", registry);
   private final YoArtifactOval inscribedCircle = new YoArtifactOval("inscribedCircle", circleCenter, inscribedCircleRadius, Color.BLACK);

   private final YoBoolean useSubCircleForBodyShiftTarget = new YoBoolean("useSubCircleForBodyShiftTarget", registry);
   private final YoDouble subCircleRadius = new YoDouble("subCircleRadius", registry);
   private final YoDouble comCloseRadius = new YoDouble("comCloseRadius", "Distance check from final desired circle to CoM for transitioning into swing state",
                                                        registry);

   private final YoFrameVector3D yoVectorToSubtract = new YoFrameVector3D("yoVectorToSubtract", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint3D centerOfMassInBody = new FramePoint3D();
   private final FramePoint3D desiredRootJointPosition = new FramePoint3D();
   private final FrameVector3D vectorToSubtractHolder = new FrameVector3D();
   private final Vector3D linearVelocityHolder = new Vector3D();

   private final YoBoolean isCoMInsideTriangleForSwingLeg = new YoBoolean("isCoMInsideTriangleForSwingLeg", registry);
   private final YoBoolean isCoMCloseToFinalDesired = new YoBoolean("isCoMCloseToFinalDesired", registry);
   private final YoBoolean useCommonTriangleForSwingTransition = new YoBoolean("useCommonTriangleForSwingTransition", registry);

   private final YoFrameVector3D feedForwardCenterOfMassVelocity = new YoFrameVector3D("feedForwardCenterOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D centerOfMassPosition = new YoFramePoint3D("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D centerOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final FramePoint3D centerOfMassFramePoint = new FramePoint3D();
   private final FramePoint2D centerOfMassPoint2d = new FramePoint2D();
   private final YoGraphicPosition centerOfMassViz = new YoGraphicPosition("centerOfMass", centerOfMassPosition, 0.02, YoAppearance.Black(),
                                                                           GraphicType.BALL_WITH_CROSS);

   private final YoFramePoint3D currentSwingTarget = new YoFramePoint3D("currentSwingTarget", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D finalSwingTarget = new YoFramePoint3D("finalSwingTarget", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition currentSwingTargetViz = new YoGraphicPosition("currentSwingTarget", currentSwingTarget, 0.01, YoAppearance.Red());
   private final YoGraphicPosition finalSwingTargetViz = new YoGraphicPosition("finalSwingTarget", finalSwingTarget, 0.01, YoAppearance.Purple());

   private final YoFramePoint3D desiredCoMTarget = new YoFramePoint3D("desiredCoMTarget", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition desiredCoMTargetViz = new YoGraphicPosition("desiredCoMTargetViz", desiredCoMTarget, 0.01, YoAppearance.Turquoise());

   private final YoFramePoint3D desiredCoM = new YoFramePoint3D("desiredCoM", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition desiredCoMViz = new YoGraphicPosition("desiredCoMViz", desiredCoM, 0.01, YoAppearance.HotPink());

   private final YoFramePoint3D currentICP = new YoFramePoint3D("currentICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition currentICPViz = new YoGraphicPosition("currentICPViz", currentICP, 0.01, YoAppearance.DarkSlateBlue());

   private final YoFramePoint3D feedForwardICP = new YoFramePoint3D("feedForwardICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition feedForwardICPViz = new YoGraphicPosition("feedForwardICPViz", feedForwardICP, 0.01, YoAppearance.DarkSlateBlue());

   private final YoGraphicReferenceFrame desiredCoMPoseYoGraphic = new YoGraphicReferenceFrame(desiredCoMPoseReferenceFrame, registry, 0.45);
   private final YoGraphicReferenceFrame comPoseYoGraphic, feedForwardCoMPoseYoGraphic;

   private final YoGraphicReferenceFrame centroidWithNominal;
   private final YoGraphicReferenceFrame centroidZUpWithNominal;
   private final QuadrantDependentList<YoGraphicReferenceFrame> tripleSupportFrames = new QuadrantDependentList<>();

   public final YoBoolean isVelocityNegative = new YoBoolean("isVelocityNegative", registry);
   public final YoDouble velocitySign = new YoDouble("velocitySign", registry);

   private final QuadrantDependentList<YoGraphicReferenceFrame> desiredAttachmentFrames = new QuadrantDependentList<YoGraphicReferenceFrame>();
   private final QuadrantDependentList<YoGraphicReferenceFrame> actualAttachmentFrames = new QuadrantDependentList<YoGraphicReferenceFrame>();

   private final YoDouble timeToFilterDesiredAtCrawlStart = new YoDouble("timeToFilterDesiredAtCrawlStart", registry);

   /** body sway trajectory **/
   private final YoBoolean comTrajectoryGeneratorRequiresReInitailization = new YoBoolean("comTrajectoryGeneratorRequiresReInitailization", registry);
   private final VelocityConstrainedPositionTrajectoryGenerator comTrajectoryGenerator = new VelocityConstrainedPositionTrajectoryGenerator("comTraj", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D initialCoMPosition = new YoFramePoint3D("initialCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D initialCoMVelocity = new YoFrameVector3D("initialCoMVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoDouble comTrajectoryTimeStart = new YoDouble("comTrajectoryTimeStart", registry);
   private final YoDouble comTrajectoryTimeCurrent = new YoDouble("comTrajectoryTimeCurrent", registry);
   private final YoDouble comTrajectoryTimeDesired = new YoDouble("comTrajectoryTimeDesired", registry);
   private final YoDouble comTrajectoryTimeLastCalled = new YoDouble("comTrajectoryTimeLastCalled", registry);
   private final YoDouble comTrajectoryTimeScaleFactor = new YoDouble("comTrajectoryTimeScaleFactor", registry);
   private final YoDouble distanceToTrotLine = new YoDouble("distanceToTrotLine", registry);
   private final YoDouble distanceTravelAtEndOfSwing = new YoDouble("distanceTravelAtEndOfSwing", registry);
   private final YoBoolean slowBodyDown = new YoBoolean("slowBodyDown", registry);
   private final YoDouble timeToSlowTo = new YoDouble("timeToSlowTo", registry);

   private final YoDouble turnInPlaceCoMTrajectoryBuffer = new YoDouble("turnInPlaceCoMTrajectoryBuffer", registry);
   private final YoDouble comTrajectoryDuration = new YoDouble("comTrajectoryDuration", registry);
   private final YoDouble maximumCoMTrajectoryDuration = new YoDouble("maximumCoMTrajectoryDuration", registry);
   private final YoDouble minimumCoMTrajectoryDuration = new YoDouble("minimumCoMTrajectoryDuration", registry);

   private final QuadrupedPostureInputProviderInterface postureProvider;
   private final QuadrupedPlanarVelocityInputProvider planarVelocityProvider;

   private final Twist bodyTwist = new Twist();

   private final JointDesiredOutputList jointDesiredOutputList;

   public QuadrupedPositionBasedCrawlController(QuadrupedRuntimeEnvironment environment, FullQuadrupedRobotModelFactory modelFactory,
                                                QuadrupedPhysicalProperties physicalProperties,
                                                QuadrupedPositionBasedCrawlControllerParameters crawlControllerParameters)
   {
      this.jointDesiredOutputList = environment.getJointDesiredOutputList();
      this.referenceFrames = new QuadrupedReferenceFrames(environment.getFullRobotModel(), physicalProperties);
      this.inverseKinematicsCalculators = new QuadrupedInverseKinematicsCalculators(modelFactory, jointDesiredOutputList, physicalProperties,
                                                                                    environment.getFullRobotModel(), referenceFrames, registry,
                                                                                    environment.getGraphicsListRegistry());

      swingDuration.set(crawlControllerParameters.getDefaultSwingDuration());
      swingHeight.set(crawlControllerParameters.getDefaultSwingHeight());
      subCircleRadius.set(crawlControllerParameters.getDefaultSubCircleRadius());
      comCloseRadius.set(crawlControllerParameters.getDefaultCoMCloseToFinalDesiredTransitionRadius());
      minYawRate.set(crawlControllerParameters.getMaxYawRate() * -1.0);
      maxYawRate.set(crawlControllerParameters.getMaxYawRate());
      distanceInsideSupportPolygonBeforeSwingingLeg.set(0.02);
      turnInPlaceCoMTrajectoryBuffer.set(0.5);

      useImuFeedback.set(false);
      pitchPidController.setProportionalGain(10.0);
      pitchPidController.setDerivativeGain(0.4);
      pitchPidController.setIntegralGain(0.4);
      pitchPidController.setMaxIntegralError(0.1);

      rollPidController.setProportionalGain(10.0);
      rollPidController.setDerivativeGain(0.4);
      rollPidController.setIntegralGain(0.1);
      rollPidController.setMaxIntegralError(0.1);

      useSubCircleForBodyShiftTarget.set(true);
      swingLeg.set(RobotQuadrant.HIND_LEFT);

      runOpenLoop.set(true);

      this.postureProvider = new QuadrupedPostureInputProvider(physicalProperties, environment.getGlobalDataProducer(), registry);
      this.planarVelocityProvider = new QuadrupedPlanarVelocityInputProvider(environment.getGlobalDataProducer(), registry);
      this.robotTimestamp = environment.getRobotTimestamp();
      this.dt = environment.getControlDT();
      this.actualFullRobotModel = environment.getFullRobotModel();
      this.centerOfMassJacobian = new CenterOfMassJacobian(environment.getFullRobotModel().getElevator());

      actualRobotRootJoint = actualFullRobotModel.getRootJoint();
      referenceFrames.updateFrames();
      comFrame = referenceFrames.getCenterOfMassFrame();

      feedForwardFullRobotModel = modelFactory.createFullRobotModel();
      this.feedForwardCenterOfMassJacobian = new CenterOfMassJacobian(feedForwardFullRobotModel.getElevator());
      feedForwardReferenceFrames = new QuadrupedReferenceFrames(feedForwardFullRobotModel, physicalProperties);
      feedForwardCenterOfMassFrame = new TranslationReferenceFrame("offsetFeedForwardCenterOfMassFrame", feedForwardReferenceFrames.getCenterOfMassFrame());
      feedForwardReferenceFrames.updateFrames();
      feedForwardBodyFrame = feedForwardReferenceFrames.getBodyFrame();
      //      this.nextSwingLegChooser = new QuadrupedGaitSwingLegChooser(feedForwardReferenceFrames, registry, yoGraphicsListRegistry);
      this.nextSwingLegChooser = new DefaultGaitSwingLegChooser();

      oneDoFJointsFeedforward = feedForwardFullRobotModel.getOneDoFJoints();
      oneDoFJointsActual = actualFullRobotModel.getOneDoFJoints();

      desiredCoMOffset = new YoFramePoint2D("desiredCoMOffset", feedForwardReferenceFrames.getBodyZUpFrame(), registry);
      desiredCoMOffset.set(crawlControllerParameters.getDefaultDesiredCoMOffset());

      updateFeedForwardModelAndFrames();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         YoGraphicReferenceFrame desiredAttachmentFrame = new YoGraphicReferenceFrame("ffLegAttachment",
                                                                                      feedForwardReferenceFrames.getLegAttachmentFrame(robotQuadrant), registry,
                                                                                      0.25, YoAppearance.Purple());
         desiredAttachmentFrames.set(robotQuadrant, desiredAttachmentFrame);

         YoGraphicReferenceFrame actualAttachmentFrame = new YoGraphicReferenceFrame("legAttachment", referenceFrames.getLegAttachmentFrame(robotQuadrant),
                                                                                     registry, 0.25, YoAppearance.Green());
         actualAttachmentFrames.set(robotQuadrant, actualAttachmentFrame);

         environment.getGraphicsListRegistry().registerYoGraphic("AttachementFrames", desiredAttachmentFrame);
         environment.getGraphicsListRegistry().registerYoGraphic("AttachementFrames", actualAttachmentFrame);
      }

      this.swingTargetGenerator = new MidFootZUpSwingTargetGenerator(crawlControllerParameters, feedForwardReferenceFrames, registry);
      this.footSwitches = environment.getFootSwitches();

      desiredVelocity = new YoFrameVector3D("desiredVelocity", feedForwardBodyFrame, registry);
      lastDesiredVelocity = new YoFrameVector3D("lastDesiredVelocity", feedForwardBodyFrame, registry);

      desiredVelocity.setX(0.0);
      comTrajectoryTimeDesired.set(1.0);
      maximumCoMTrajectoryDuration.set(6.0);
      minimumCoMTrajectoryDuration.set(0.01);
      comTrajectoryTimeScaleFactor.set(1.0);

      shrunkenPolygonSize.set(0.02);

      timeToFilterDesiredAtCrawlStart.set(4.0);

      comPoseYoGraphic = new YoGraphicReferenceFrame("rasta_", comFrame, registry, 0.25, YoAppearance.Green());
      feedForwardCoMPoseYoGraphic = new YoGraphicReferenceFrame("feedForwardRasta_", feedForwardCenterOfMassFrame, registry, 0.25, YoAppearance.Purple());

      feedForwardCenterOfMassOffsetAlpha = new YoDouble("feedForwardCenterOfMassOffsetAlpha", registry);
      feedForwardCenterOfMassOffsetAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0, dt));
      feedForwardCenterOfMassOffset = new YoFramePoint3D("feedForwardCenterOfMassOffset", feedForwardCenterOfMassFrame, registry);
      filteredFeedForwardCenterOfMassOffset = AlphaFilteredYoFramePoint.createAlphaFilteredYoFramePoint("filteredFeedForwardCenterOfMassOffset", "", registry, feedForwardCenterOfMassOffsetAlpha, feedForwardCenterOfMassOffset);


      centroidWithNominal = new YoGraphicReferenceFrame(referenceFrames.getCenterOfFeetFrameAveragingLowestZHeightsAcrossEnds(), registry, 0.1);
      centroidZUpWithNominal = new YoGraphicReferenceFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds(), registry, 0.1);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         tripleSupportFrames.set(robotQuadrant,
                                 new YoGraphicReferenceFrame(referenceFrames.getTripleSupportFrameAveragingLowestZHeightsAcrossEnds(robotQuadrant), registry,
                                                             0.1));
      }

      filteredDesiredCoMYawAlphaBreakFrequency.set(DEFAULT_HEADING_CORRECTION_BREAK_FREQUENCY);
      filteredDesiredCoMYawAlpha
            .set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMYawAlphaBreakFrequency.getDoubleValue(), dt));
      filteredDesiredCoMYawAlphaBreakFrequency
            .addVariableChangedListener(createBreakFrequencyChangeListener(dt, filteredDesiredCoMYawAlphaBreakFrequency, filteredDesiredCoMYawAlpha));

      filteredDesiredCoMPitchAlphaBreakFrequency.set(DEFAULT_COM_PITCH_FILTER_BREAK_FREQUENCY);
      filteredDesiredCoMPitchAlpha
            .set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMPitchAlphaBreakFrequency.getDoubleValue(), dt));
      filteredDesiredCoMPitchAlphaBreakFrequency
            .addVariableChangedListener(createBreakFrequencyChangeListener(dt, filteredDesiredCoMPitchAlphaBreakFrequency, filteredDesiredCoMPitchAlpha));

      filteredDesiredCoMRollAlphaBreakFrequency.set(DEFAULT_COM_ROLL_FILTER_BREAK_FREQUENCY);
      filteredDesiredCoMRollAlpha
            .set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMRollAlphaBreakFrequency.getDoubleValue(), dt));
      filteredDesiredCoMRollAlphaBreakFrequency
            .addVariableChangedListener(createBreakFrequencyChangeListener(dt, filteredDesiredCoMRollAlphaBreakFrequency, filteredDesiredCoMRollAlpha));

      desiredYawInPlaceRateLimited = new RateLimitedYoVariable("desiredYawInPlaceRateLimited", registry, desiredYawInPlaceRateLimit, dt);
      desiredYawInPlaceRateLimit.set(DEFAULT_YAW_IN_PLACE_RATE_LIMIT);

      filteredDesiredCoMHeightAlphaBreakFrequency.set(DEFAULT_COM_HEIGHT_Z_FILTER_BREAK_FREQUENCY);
      filteredDesiredCoMHeightAlpha
            .set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMHeightAlphaBreakFrequency.getDoubleValue(), dt));
      filteredDesiredCoMHeightAlphaBreakFrequency.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            filteredDesiredCoMHeightAlpha
                  .set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMHeightAlphaBreakFrequency.getDoubleValue(), dt));

         }
      });

      referenceFrames.updateFrames();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingTrajectoryGenerators
               .set(robotQuadrant, new QuadrupedSwingTrajectoryGenerator(robotQuadrant, registry, environment.getGraphicsListRegistry(), dt));

         ReferenceFrame footReferenceFrame = referenceFrames.getSoleFrame(robotQuadrant);
         ReferenceFrame legAttachmentFrame = referenceFrames.getLegAttachmentFrame(robotQuadrant);

         legAttachmentFrames.set(robotQuadrant, legAttachmentFrame);

         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();

         YoFramePoint3D actualFootPosition = new YoFramePoint3D(prefix + "actualFootPosition", ReferenceFrame.getWorldFrame(), registry);
         actualFeetLocations.set(robotQuadrant, actualFootPosition);

         YoFramePoint3D desiredFootLocation = new YoFramePoint3D(prefix + "FootDesiredPosition", ReferenceFrame.getWorldFrame(), registry);

         FramePoint3D footPosition = new FramePoint3D(footReferenceFrame);
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         footPosition.setZ(0.0);
         desiredFootLocation.set(footPosition);
         desiredFeetLocations.set(robotQuadrant, desiredFootLocation);

         YoFrameVector3D footPositionInLegAttachementFrame = new YoFrameVector3D(prefix + "FootPositionInLegFrame", referenceFrames.getLegAttachmentFrame(robotQuadrant), registry);
         desiredFeetPositionsInLegAttachmentFrame.set(robotQuadrant, footPositionInLegAttachementFrame);

         YoFrameVector3D actualFootPositionInLegAttachementFrame = new YoFrameVector3D(prefix + "ActualFootPositionInLegFrame", referenceFrames.getLegAttachmentFrame(robotQuadrant), registry);
         actualFeetPositionsInLegAttachmentFrame.set(robotQuadrant, actualFootPositionInLegAttachementFrame);
      }

      //      for (int i = 0; i < tripleSupportPolygons.length; i++)
      //      {
      //         String polygonName = "tripleSupport" + i;
      //         YoFrameConvexPolygon2d yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d(polygonName, "", ReferenceFrame.getWorldFrame(), 3, registry);
      //         tripleSupportPolygons[i] = yoFrameConvexPolygon2d;
      //
      //         float saturation = 0.5f;
      //         float brightness = 0.5f;
      //         float hue = (float) (0.1 * i);
      //
      //         tripleSupportArtifactPolygons[i] = new YoArtifactPolygon(polygonName, yoFrameConvexPolygon2d, Color.getHSBColor(hue, saturation, brightness), false);
      //         yoGraphicsListRegistry.registerArtifact(polygonName, tripleSupportArtifactPolygons[i]);
      //         yoGraphicsListRegistryForDetachedOverhead.registerArtifact(polygonName, tripleSupportArtifactPolygons[i]);
      //      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String polygonName = robotQuadrant.getCamelCaseNameForStartOfExpression() + "TripleSupportPolyon";
         YoFrameConvexPolygon2D yoFrameConvexPolygon2d = new YoFrameConvexPolygon2D(polygonName, "", ReferenceFrame.getWorldFrame(), 3, registry);
         tripleSupportPolygons.set(robotQuadrant, yoFrameConvexPolygon2d);

         float saturation = 0.5f;
         float brightness = 0.5f;
         float hue = (float) (0.1 * robotQuadrant.ordinal());

         YoArtifactPolygon yoArtifactPolygon = new YoArtifactPolygon(polygonName, yoFrameConvexPolygon2d, Color.getHSBColor(hue, saturation, brightness),
                                                                     false);
         tripleSupportArtifactPolygons.set(robotQuadrant, yoArtifactPolygon);
         environment.getGraphicsListRegistry().registerArtifact(polygonName, yoArtifactPolygon);
         environment.getGraphicsListRegistryForDetachedOverhead().registerArtifact(polygonName, yoArtifactPolygon);
      }

      createGraphicsAndArtifacts(environment.getGraphicsListRegistry(), environment.getGraphicsListRegistryForDetachedOverhead());

      referenceFrames.updateFrames();
      updateFeetLocations();

      FramePose3D centerOfMassPose = new FramePose3D(referenceFrames.getCenterOfFourHipsFrame());
      centerOfMassPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredCoMHeight.set(crawlControllerParameters.getInitialCoMHeight());
      filteredDesiredCoMHeight.update();
      centerOfMassPose.setZ(filteredDesiredCoMHeight.getDoubleValue());
      desiredCoMPose.set(centerOfMassPose);
      desiredCoM.set(centerOfMassPose.getPosition());
      desiredCoMPoseReferenceFrame.setPoseAndUpdate(centerOfMassPose);
      updateFeedForwardModelAndFrames();

      quadrupleSupportState = new QuadrupleSupportState(DEFAULT_TIME_TO_STAY_IN_DOUBLE_SUPPORT, 0.2);
      TripleSupportState tripleSupportState = new TripleSupportState();

      filterDesiredsToMatchCrawlControllerOnTransitionIn = new FilterDesiredsToMatchCrawlControllerState();

      StateMachineFactory<CrawlGateWalkingState, State> factory = new StateMachineFactory<>(CrawlGateWalkingState.class);
      factory.setNamePrefix("QuadrupedCrawlState").setRegistry(registry).buildYoClock(robotTimestamp);
      factory.addState(CrawlGateWalkingState.ALPHA_FILTERING_DESIREDS, filterDesiredsToMatchCrawlControllerOnTransitionIn);
      factory.addState(CrawlGateWalkingState.QUADRUPLE_SUPPORT, quadrupleSupportState);
      factory.addState(CrawlGateWalkingState.TRIPLE_SUPPORT, tripleSupportState);

      factory.addTransition(CrawlGateWalkingState.ALPHA_FILTERING_DESIREDS, CrawlGateWalkingState.QUADRUPLE_SUPPORT,
                            new FilterToQuadrupleCondition(filterDesiredsToMatchCrawlControllerOnTransitionIn));
      factory.addTransition(CrawlGateWalkingState.QUADRUPLE_SUPPORT, CrawlGateWalkingState.TRIPLE_SUPPORT,
                            new QuadrupleToTripleCondition(quadrupleSupportState));
      factory.addTransition(CrawlGateWalkingState.TRIPLE_SUPPORT, CrawlGateWalkingState.QUADRUPLE_SUPPORT, new TripleToQuadrupleCondition());

      walkingStateMachine = factory.build(CrawlGateWalkingState.ALPHA_FILTERING_DESIREDS);

      YoBoolean applyJoystickInput = new YoBoolean("applyJoystickInput", registry);
      applyJoystickInput.set(true);

      environment.getParentRegistry().addChild(registry);
   }

   private VariableChangedListener createBreakFrequencyChangeListener(final double dt, final YoDouble breakFrequency, final YoDouble alpha)
   {
      return new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            double newAlpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(breakFrequency.getDoubleValue(), dt);
            alpha.set(newAlpha);
         }
      };
   }

   private void createGraphicsAndArtifacts(YoGraphicsListRegistry yoGraphicsListRegistry, YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead)
   {
      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("quadSupportPolygonArtifact", supportPolygon, Color.BLUE, false);
      YoArtifactPolygon currentTriplePolygonArtifact = new YoArtifactPolygon("currentTriplePolygonArtifact", currentTriplePolygon, Color.GREEN, false);
      YoArtifactPolygon upcomingTriplePolygonArtifact = new YoArtifactPolygon("upcomingTriplePolygonArtifact", upcomingTriplePolygon, Color.yellow, false);
      YoArtifactPolygon commonTriplePolygonArtifact = new YoArtifactPolygon("commonTriplePolygonArtifact", commonTriplePolygon, Color.RED, false);
      YoArtifactPolygon commonTriplePolygonLeftArtifact = new YoArtifactPolygon("commonTriplePolygonLeftArtifact", commonTriplePolygonLeft, Color.pink, false);
      YoArtifactPolygon commonTriplePolygonRightArtifact = new YoArtifactPolygon("commonTriplePolygonRightArtifact", commonTriplePolygonRight, Color.MAGENTA,
                                                                                 false);

      yoGraphicsListRegistry.registerArtifact("supportPolygon", supportPolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("currentTriplePolygon", currentTriplePolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("upcomingTriplePolygon", upcomingTriplePolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("commonTriplePolygon", commonTriplePolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("commonTriplePolygonLeft", commonTriplePolygonLeftArtifact);
      yoGraphicsListRegistry.registerArtifact("commonTriplePolygonRight", commonTriplePolygonRightArtifact);

      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("supportPolygon", supportPolygonArtifact);
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("currentTriplePolygon", currentTriplePolygonArtifact);
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("upcomingTriplePolygon", upcomingTriplePolygonArtifact);
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("commonTriplePolygon", commonTriplePolygonArtifact);
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("commonTriplePolygonLeft", commonTriplePolygonLeftArtifact);
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("commonTriplePolygonRight", commonTriplePolygonRightArtifact);

      yoGraphicsListRegistry.registerArtifact("inscribedCircle", inscribedCircle);
      yoGraphicsListRegistry.registerArtifact("circleCenterViz", circleCenterGraphic.createArtifact());
      yoGraphicsListRegistry.registerArtifact("centerOfMassViz", centerOfMassViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("currentSwingTarget", currentSwingTargetViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("finalSwingTarget", finalSwingTargetViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("desiredCoMTarget", desiredCoMTargetViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("desiredCoMViz", desiredCoMViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("currentICPViz", currentICPViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact("feedForwardICPViz", feedForwardICPViz.createArtifact());

      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("inscribedCircle", inscribedCircle);
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("circleCenterViz", circleCenterGraphic.createArtifact());
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("centerOfMassViz", centerOfMassViz.createArtifact());
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("currentSwingTarget", currentSwingTargetViz.createArtifact());
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("finalSwingTarget", finalSwingTargetViz.createArtifact());
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("desiredCoMTarget", desiredCoMTargetViz.createArtifact());
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("desiredCoMViz", desiredCoMViz.createArtifact());
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("currentICPViz", currentICPViz.createArtifact());

      yoGraphicsListRegistry.registerArtifact("nominalYawArtifact", nominalYawArtifact);
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("nominalYawArtifact", nominalYawArtifact);

      yoGraphicsListRegistry.registerYoGraphic("centerOfMassViz", centerOfMassViz);
      yoGraphicsListRegistry.registerYoGraphic("desiredCoMPoseYoGraphic", desiredCoMPoseYoGraphic);
      yoGraphicsListRegistry.registerYoGraphic("comPoseYoGraphic", comPoseYoGraphic);
      yoGraphicsListRegistry.registerYoGraphic("feedForwardCoMPoseYoGraphic", feedForwardCoMPoseYoGraphic);

      yoGraphicsListRegistry.registerYoGraphic("centroidWithNominal", centroidWithNominal);
      yoGraphicsListRegistry.registerYoGraphic("centroidZUpWithNominal", centroidZUpWithNominal);

      yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("centerOfMassViz", centerOfMassViz);
      yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("desiredCoMPoseYoGraphic", desiredCoMPoseYoGraphic);
      yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("comPoseYoGraphic", comPoseYoGraphic);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();

         YoFramePoint3D footPosition = actualFeetLocations.get(robotQuadrant);
         YoGraphicPosition actualFootPositionViz = new YoGraphicPosition(prefix + "actualFootPositionViz", footPosition, 0.02,
               getYoAppearance(robotQuadrant), GraphicType.BALL_WITH_CROSS);

         yoGraphicsListRegistry.registerYoGraphic("actualFootPosition", actualFootPositionViz);
         yoGraphicsListRegistry.registerArtifact("actualFootPosition", actualFootPositionViz.createArtifact());
         yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("actualFootPosition", actualFootPositionViz);
         yoGraphicsListRegistryForDetachedOverhead.registerArtifact("actualFootPosition", actualFootPositionViz.createArtifact());

         YoFramePoint3D desiredFootPosition = desiredFeetLocations.get(robotQuadrant);
         YoGraphicPosition desiredFootPositionViz = new YoGraphicPosition(prefix + "desiredFootPositionViz", desiredFootPosition, 0.01,
               YoAppearance.Red());
//         desiredFootPositionViz.hideGraphicObject();

         yoGraphicsListRegistry.registerYoGraphic("Desired Feet", desiredFootPositionViz);
         yoGraphicsListRegistry.registerArtifact("Desired Feet", desiredFootPositionViz.createArtifact());
         yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("Desired Feet", desiredFootPositionViz);
         yoGraphicsListRegistryForDetachedOverhead.registerArtifact("Desired Feet", desiredFootPositionViz.createArtifact());

         yoGraphicsListRegistry.registerYoGraphic(prefix + "TripleSupportFrame", tripleSupportFrames.get(robotQuadrant));
      }
   }

   private AppearanceDefinition getYoAppearance(RobotQuadrant robotQuadrant)
   {
      switch (robotQuadrant)
      {
      case FRONT_LEFT:
         return YoAppearance.White();
      case FRONT_RIGHT:
         return YoAppearance.Yellow();
      case HIND_LEFT:
         return YoAppearance.Blue();
      case HIND_RIGHT:
         return YoAppearance.Black();
      default:
         throw new RuntimeException("bad quad");
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      doActionTimer.startMeasurement();

      referenceFrames.updateFrames();
      updateEstimates();
      updateGraphics();
      pollDataProviders();
      checkForReversedVelocity();
      walkingStateMachine.doActionAndTransition();
      updateDesiredCoMTrajectory();
      updateDesiredHeight();
      updateDesiredYaw();
      updateDesiredBodyOrientation();
      updateDesiredCoMPose();
      updateLegsBasedOnDesiredCoM();
      computeDesiredPositionsAndStoreInFullRobotModel(actualFullRobotModel);

      if (walkingStateMachine.getCurrentStateKey() == CrawlGateWalkingState.ALPHA_FILTERING_DESIREDS)
      {
         filterDesiredsToMatchCrawlControllerOnTransitionIn.filterDesireds();
      }

      updateFeedForwardModelAndFrames();

      doActionTimer.stopMeasurement();
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      return null;
   }

   private void checkForReversedVelocity()
   {
      if (desiredVelocity.getX() < 0 && lastDesiredVelocity.getX() >= 0)
      {
         isVelocityNegative.set(true);
         velocitySign.set(-1.0);
      }
      else if (desiredVelocity.getX() >= 0 && lastDesiredVelocity.getX() < 0)
      {
         isVelocityNegative.set(false);
         velocitySign.set(1.0);
      }
   }

   private boolean isDesiredVelocityAndYawRateZero()
   {
      boolean isDesiredVelocityZero = desiredVelocity.length() < 0.1e-3;
      boolean isDesiredYawRateZero = Math.abs(desiredYawRate.getDoubleValue()) < 0.1e-3;
      return isDesiredVelocityZero && isDesiredYawRateZero;
   }

   private final RigidBodyTransform rootJointPose = new RigidBodyTransform();

   private void setFeedForwardToActuals()
   {
      for (int i = 0; i < oneDoFJointsActual.length; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJointsActual[i];
         OneDoFJoint oneDoFJointFeedforward = oneDoFJointsFeedforward[i];

         oneDoFJointFeedforward.setQ(jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint).getDesiredPosition());
      }

      feedForwardReferenceFrames.updateFrames();
      actualFullRobotModel.updateFrames();

      FloatingInverseDynamicsJoint feedForwardRootJoint = feedForwardFullRobotModel.getRootJoint();

      actualRobotRootJoint.getJointTransform3D(rootJointPose);
      feedForwardRootJoint.setPositionAndRotation(rootJointPose);

      feedForwardReferenceFrames.updateFrames();

      centerOfMassFramePoint.setToZero(feedForwardCenterOfMassFrame);
      centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassPosition.set(centerOfMassFramePoint);
      desiredCoMPosition.set(centerOfMassPosition);

      filteredDesiredCoMYaw.reset();
      filteredDesiredCoMPitch.reset();
      filteredDesiredCoMRoll.reset();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ReferenceFrame footFrame = feedForwardReferenceFrames.getSoleFrame(robotQuadrant);
         desiredFootPosition.setToZero(footFrame);
         desiredFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
         fourFootSupportPolygon.setFootstep(robotQuadrant, desiredFootPosition);
      }

      double desiredZInWorld = desiredCoMPosition.getZ();
      double lowestZ = fourFootSupportPolygon.getLowestFootstepZHeight();
      //      double comHeight = desiredZInWorld - lowestZ;
      double comHeight = desiredZInWorld;

      desiredCoMHeight.set(comHeight);
      filteredDesiredCoMHeight.reset();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ReferenceFrame footFrame = referenceFrames.getSoleFrame(robotQuadrant);
         actualFootLocation.setToZero(footFrame);
         actualFootLocation.changeFrame(ReferenceFrame.getWorldFrame());

         YoFramePoint3D yoActualFootLocation = actualFeetLocations.get(robotQuadrant);
         yoActualFootLocation.set(actualFootLocation);

         YoFramePoint3D yoDesiredFootLocation = desiredFeetLocations.get(robotQuadrant);
         yoDesiredFootLocation.set(actualFootLocation);

         fourFootSupportPolygon.setFootstep(robotQuadrant, actualFootLocation);
      }
   }

   private void updateFeedForwardModelAndFrames()
   {
      for (int i = 0; i < oneDoFJointsActual.length; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJointsActual[i];
         OneDoFJoint oneDoFJointFeedforward = oneDoFJointsFeedforward[i];
         JointDesiredOutput jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(oneDoFJoint);

         //		   oneDoFJointFeedforward.setQ(oneDoFJoint.getQ());
         double qd = jointDesiredOutput.getDesiredVelocity() - oneDoFJointFeedforward.getQ();
         oneDoFJointFeedforward.setQd(qd * 1.0 / dt);
         oneDoFJointFeedforward.setQ(jointDesiredOutput.getDesiredPosition());
      }

      FloatingInverseDynamicsJoint feedForwardRootJoint = feedForwardFullRobotModel.getRootJoint();

      feedForwardRootJoint.setRotation(filteredDesiredCoMOrientation.getYaw().getDoubleValue(), filteredDesiredCoMOrientation.getPitch().getDoubleValue(),
                                       filteredDesiredCoMOrientation.getRoll().getDoubleValue());
      feedForwardFullRobotModel.updateFrames();

      //	   Vector3d rootJointPosition = new Vector3d();
      //	   rootJoint.packTranslation(rootJointPosition);
      //	   feedForwardRootJoint.setPosition(rootJointPosition);

      centerOfMassInBody.setIncludingFrame(feedForwardCenterOfMassFrame, 0.0, 0.0, 0.0);
      centerOfMassInBody.changeFrame(feedForwardRootJoint.getFrameAfterJoint());

      vectorToSubtractHolder.setIncludingFrame(feedForwardFullRobotModel.getRootJoint().getFrameAfterJoint(), centerOfMassInBody);
      vectorToSubtractHolder.changeFrame(ReferenceFrame.getWorldFrame());

      yoVectorToSubtract.set(vectorToSubtractHolder);
      //	   System.out.println("VectorToSubtract = " + vectorToSubtract);

      desiredRootJointPosition.setIncludingFrame(desiredCoM);
      desiredRootJointPosition.sub(vectorToSubtractHolder);
      feedForwardRootJoint.getTranslation(linearVelocityHolder);
      feedForwardRootJoint.setPosition(desiredRootJointPosition);
      linearVelocityHolder.sub(desiredRootJointPosition, linearVelocityHolder);
      //	   feedForwardRootJoint.setLinearVelocityInWorld(linearVelocityHolder);

      //	   feedForwardFullRobotModel.updateFrames();
      feedForwardReferenceFrames.updateFrames();
   }

   private double lastProvidedDesiredYawRate = 0.0;
   private final FrameVector3D providedDesiredVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());

   private void pollDataProviders()
   {

      if (postureProvider != null)
      {
         postureProvider.getComVelocityInput();// support z up

         //body velocity
         Vector3D planarVelocityInput = planarVelocityProvider.get();//Frame up to controller - feedForwardBodyFrame
         desiredVelocity.set(planarVelocityInput);
         desiredVelocity.setY(0.0);
         desiredVelocity.setZ(0.0);

         //yaw rate
         double providedDesiredYawRate = planarVelocityInput.getZ();
         providedDesiredYawRate = MathTools.clamp(providedDesiredYawRate, minYawRate.getDoubleValue(), maxYawRate.getDoubleValue());
         if (providedDesiredYawRate != lastProvidedDesiredYawRate)
         {
            desiredYawRate.set(providedDesiredYawRate);
            lastProvidedDesiredYawRate = providedDesiredYawRate;
         }

         //com height
         Point3D comPositionInput = postureProvider.getComPositionInput();// support z up
         desiredCoMHeight.set(comPositionInput.getZ());
      }
   }

   private final double[] yawPitchRollArray = new double[3];
   private final Point3D centerOfMassOffset = new Point3D();
   private final FramePoint3D actualFootLocation = new FramePoint3D();
   private final FrameVector3D tempFrameVector = new FrameVector3D();

   /**
    * uses feedback to update the CoM Velocity, ICP, and Actual Foot Positions
    */
   private void updateEstimates()
   {
      actualFullRobotModel.getBody().getBodyFixedFrame().getTwistOfFrame(bodyTwist);

      actualRobotRootJoint.getRotation(yawPitchRollArray);
      actualYaw.set(yawPitchRollArray[0]);
      actualPitch.set(yawPitchRollArray[1]);
      actualRoll.set(yawPitchRollArray[2]);

      filteredFeedForwardCenterOfMassOffset.update();
      centerOfMassOffset.set(filteredFeedForwardCenterOfMassOffset);
      feedForwardCenterOfMassFrame.updateTranslation(centerOfMassOffset);

      // compute center of mass position and velocity
      feedForwardCoMPosition.setIncludingFrame(feedForwardCenterOfMassFrame, 0.0, 0.0, 0.0);
      feedForwardCoMPosition.changeFrame(ReferenceFrame.getWorldFrame());
      feedForwardCenterOfMassJacobian.compute();
      feedForwardCenterOfMassJacobian.getCenterOfMassVelocity(tempFrameVector);
      tempFrameVector.changeFrame(ReferenceFrame.getWorldFrame());
      feedForwardCenterOfMassVelocity.set(tempFrameVector);

      // compute instantaneous capture point
      double feedForwardZFoot = desiredFeetLocations.get(fourFootSupportPolygon.getLowestFootstep()).getZ();
      double feedForwardZDelta = feedForwardCoMPosition.getZ() - feedForwardZFoot;
      double feedForwardOmega = Math.sqrt(9.81 / feedForwardZDelta);
      feedForwardICP.setX(feedForwardCoMPosition.getX() + feedForwardCenterOfMassVelocity.getX() / feedForwardOmega);
      feedForwardICP.setY(feedForwardCoMPosition.getY() + feedForwardCenterOfMassVelocity.getY() / feedForwardOmega);
      feedForwardICP.setZ(feedForwardZFoot);

      // compute center of mass position and velocity
      tempCoMPosition.setIncludingFrame(comFrame, 0.0, 0.0, 0.0);
      tempCoMPosition.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassJacobian.compute();
      centerOfMassJacobian.getCenterOfMassVelocity(tempFrameVector);
      tempFrameVector.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassVelocity.set(tempFrameVector);

      // compute instantaneous capture point
      double zFoot = actualFeetLocations.get(fourFootSupportPolygon.getLowestFootstep()).getZ();
      double zDelta = tempCoMPosition.getZ() - zFoot;
      double omega = Math.sqrt(9.81 / zDelta);
      currentICP.setX(tempCoMPosition.getX() + centerOfMassVelocity.getX() / omega);
      currentICP.setY(tempCoMPosition.getY() + centerOfMassVelocity.getY() / omega);
      currentICP.setZ(zFoot);

      updateFeetLocations();
   }

   /**
    * update actual feet locations and the four foot polygon, using the desired locations
    */
   private void updateFeetLocations()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ReferenceFrame footFrame = referenceFrames.getSoleFrame(robotQuadrant);
         actualFootLocation.setToZero(footFrame);
         actualFootLocation.changeFrame(ReferenceFrame.getWorldFrame());

         YoFramePoint3D yoFootLocation = actualFeetLocations.get(robotQuadrant);
         yoFootLocation.set(actualFootLocation);

         // Use the desired foot locations instead of the actual locations
         YoFramePoint3D desiredFootLocation = desiredFeetLocations.get(robotQuadrant);
         fourFootSupportPolygon.setFootstep(robotQuadrant, desiredFootLocation);
      }
   }

   FramePoint3D tempCOMTarget = new FramePoint3D(ReferenceFrame.getWorldFrame());

   private double computeDistanceToTrotLine2d(RobotQuadrant swingQuadrant)
   {
      comTrajectoryGenerator.getFinalPosition(tempCOMTarget);
      FramePoint3DReadOnly desiredBodyCurrent = desiredCoMPose.getPosition();

      //do this only if the desiredBodyCurrent != tempCOMTarget
      double distanceToTrotLine;
      if (desiredBodyCurrent.distance(tempCOMTarget) > 1e-3)
      {
         distanceToTrotLine = fourFootSupportPolygon
               .getDistanceFromP1ToTrotLineInDirection2d(swingQuadrant.getAcrossBodyQuadrant(), desiredBodyCurrent, tempCOMTarget);
      }
      else
      {
         //For now, use distance inside. We really want the perpendicular distance to the trot line.
         computeCurrentSupportPolygonAndDistanceInside(swingQuadrant);
         distanceToTrotLine = distanceInside.getDoubleValue();
      }

      return distanceToTrotLine;
   }

   private void computeCurrentSupportPolygonAndDistanceInside(RobotQuadrant swingQuadrant)
   {
      currentSupportPolygon.set(fourFootSupportPolygon);
      if (swingQuadrant != null)
         currentSupportPolygon.removeFootstep(swingQuadrant);
      centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      distanceInside.set(-currentSupportPolygon.signedDistance(centerOfMassFramePoint));
   }

   private void updateGraphics()
   {
      desiredCoMPoseYoGraphic.update();
      desiredCoMViz.update();
      comPoseYoGraphic.update();
      feedForwardCoMPoseYoGraphic.update();
      //      centerOfMassFramePoint.setToZero(comFrame);
      centerOfMassFramePoint.setToZero(desiredCoMPoseReferenceFrame);
      centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassPosition.set(centerOfMassFramePoint);
      drawSupportPolygon(currentSupportPolygon, supportPolygon);
      centroidWithNominal.update();
      centroidZUpWithNominal.update();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         tripleSupportFrames.get(robotQuadrant).update();
         desiredAttachmentFrames.get(robotQuadrant).update();
         actualAttachmentFrames.get(robotQuadrant).update();
      }
   }

   /**
    * sets the desired X and Y CoM positions to the current positions
    * along the trajectory in world and uses the desired CoM height for Z
    */
   private void updateDesiredCoMTrajectory()
   {
      if (!comTrajectoryGenerator.isDone() && !comTrajectoryGeneratorRequiresReInitailization.getBooleanValue() && (
            walkingStateMachine.getCurrentStateKey() == CrawlGateWalkingState.TRIPLE_SUPPORT || !isDesiredVelocityAndYawRateZero()))
      {
         //         double deltaTimeFromLastCall = robotTimestamp.getDoubleValue() - comTrajectoryTimeLastCalled.getDoubleValue();
         double deltaTimeToUse = comTrajectoryTimeScaleFactor.getDoubleValue() * dt;
         comTrajectoryTimeCurrent.set(comTrajectoryTimeCurrent.getDoubleValue() + deltaTimeToUse);
         //comTrajectoryTimeCurrent.set(robotTimestamp.getDoubleValue() - comTrajectoryTimeStart.getDoubleValue());
         comTrajectoryGenerator.compute(comTrajectoryTimeCurrent.getDoubleValue());
         comTrajectoryGenerator.getPosition(desiredCoMFramePosition);
         comTrajectoryGenerator.getVelocity(desiredCoMVelocity);
         desiredCoMFramePosition.setZ(desiredCoMPose.getPosition().getZ());

         desiredCoMPose.setPosition(desiredCoMFramePosition);

         comTrajectoryTimeLastCalled.set(robotTimestamp.getDoubleValue());
      }
   }

   /**
    * yaws the body based on the actual feet
    */
   private void updateDesiredYaw()
   {
      fourFootSupportPolygon.getCentroid(centroidFramePoint);
      nominalYaw.set(fourFootSupportPolygon.getNominalYaw());

      centroidFramePoint2d.setIncludingFrame(centroidFramePoint);
      endPoint2d.set(centroidFramePoint2d);
      endPoint2d.add(0.4, 0.0);
      GeometryTools.yawAboutPoint(endPoint2d, centroidFramePoint2d, nominalYaw.getDoubleValue(), endPoint2d);

      nominalYawLineSegment.set(centroidFramePoint2d, endPoint2d);
      YoDouble desiredYaw = desiredCoMOrientation.getYaw();

      //only let desiredYawInPlace be non zero when not walking
      double maxYawInPlace = MAX_YAW_IN_PLACE;
      if (!isDesiredVelocityAndYawRateZero())
      {
         desiredYawInPlace.set(0.0);
      }
      else
      {
         if (desiredYawInPlace.getDoubleValue() > maxYawInPlace)
            desiredYawInPlace.set(maxYawInPlace);
         else if (desiredYawInPlace.getDoubleValue() < -maxYawInPlace)
            desiredYawInPlace.set(-maxYawInPlace);
      }

      desiredYawInPlaceRateLimited.update(desiredYawInPlace.getDoubleValue());
      desiredYaw.set(nominalYaw.getDoubleValue() + desiredYawInPlaceRateLimited.getDoubleValue());
   }

   /**
    * desired CoM height in world
    */
   private void updateDesiredHeight()
   {
      filteredDesiredCoMHeight.update();
      double zHeight = filteredDesiredCoMHeight.getDoubleValue();
      //      double zHeight = fourFootSupportPolygon.getLowestFootStepZHeight() + filteredDesiredCoMHeight.getDoubleValue();
      desiredCoMPosition.setZ(zHeight);
   }

   private final Vector3D bodyOrienation = new Vector3D();

   private void updateDesiredBodyOrientation()
   {
      filteredDesiredCoMYaw.update();
      filteredDesiredCoMPitch.update();
      filteredDesiredCoMRoll.update();

      bodyTwist.changeFrame(referenceFrames.getBodyFrame());
      bodyTwist.getAngularPart(bodyOrienation);

      if (useImuFeedback.getBooleanValue())
      {
         double pitchError = pitchPidController.compute(actualPitch.getDoubleValue(), filteredDesiredCoMPitch.getDoubleValue(), bodyOrienation.getY(), 0.0, dt);
         desiredCoMPitch.set(filteredDesiredCoMPitch.getDoubleValue() + pitchError);

         double rollError = rollPidController.compute(actualRoll.getDoubleValue(), filteredDesiredCoMRoll.getDoubleValue(), bodyOrienation.getX(), 0.0, dt);
         desiredCoMRoll.set(filteredDesiredCoMRoll.getDoubleValue() + rollError);
      }
      else
      {
         desiredCoMPitch.set(filteredDesiredCoMPitch.getDoubleValue());
         desiredCoMRoll.set(filteredDesiredCoMRoll.getDoubleValue());
      }
      desiredCoMYaw.set(filteredDesiredCoMYaw.getDoubleValue());
   }

   private void updateDesiredCoMPose()
   {
      desiredCoMPose.getFramePose(desiredCoMFramePose);
      desiredCoM.set(desiredCoMFramePose.getX(), desiredCoMFramePose.getY(), desiredCoMFramePose.getZ());
      desiredCoMPoseReferenceFrame.setPoseAndUpdate(desiredCoMFramePose);
   }

   private FramePoint3D actualFootPositionInLegAttachmentFrame = new FramePoint3D();

   private void updateLegsBasedOnDesiredCoM()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         packFootPositionUsingDesiredBodyToBodyHack(robotQuadrant);

         actualFootPositionInLegAttachmentFrame.setIncludingFrame(referenceFrames.getSoleFrame(robotQuadrant), 0.0, 0.0, 0.0);
         actualFootPositionInLegAttachmentFrame.changeFrame(referenceFrames.getLegAttachmentFrame(robotQuadrant));
         actualFeetPositionsInLegAttachmentFrame.get(robotQuadrant).set(actualFootPositionInLegAttachmentFrame);
      }
   }

   private void computeDesiredPositionsAndStoreInFullRobotModel(FullRobotModel fullRobotModel)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         desiredFootPositionForInverseKinematics.set(desiredFeetPositionsInLegAttachmentFrame.get(robotQuadrant));
         inverseKinematicsCalculators
               .solveForEndEffectorLocationInBodyAndUpdateDesireds(robotQuadrant, desiredFootPositionForInverseKinematics, fullRobotModel);
      }
   }

   /**
    * currently uses the difference between the actual CoM and desired CoM to move the body,
    * This should be a feedforward psuedo actual, integrated from the desired, relating to the desiredBody
    */
   private void packFootPositionUsingDesiredBodyToBodyHack(RobotQuadrant robotQuadrant)
   {
      desiredFootPosition.setIncludingFrame(desiredFeetLocations.get(robotQuadrant));
      desiredFootPosition.changeFrame(desiredCoMPoseReferenceFrame);

      // Fix this for feed forward!!!
      desiredFootPositionInBody.setIncludingFrame(feedForwardCenterOfMassFrame, desiredFootPosition);
      desiredFootPositionInBody.changeFrame(feedForwardReferenceFrames.getLegAttachmentFrame(robotQuadrant));

      desiredFeetPositionsInLegAttachmentFrame.get(robotQuadrant).setMatchingFrame(desiredFootPositionInBody);
   }

   FrameVector3D expectedAverageVelocity = new FrameVector3D();

   /**
    * Calculates the next swing target in world using the actual feet,
    * should create another method for feedforward then handle moving to body frame
    */
   public void calculateSwingTarget(RobotQuadrant swingLeg, FramePoint3D framePointToPack)
   {
      tempComTrajComputedVelocity.setIncludingFrame(desiredCoMVelocity);
      tempDesiredVelocityVector.setIncludingFrame(desiredVelocity);

      tempComTrajComputedVelocity.changeFrame(feedForwardBodyFrame);
      tempDesiredVelocityVector.changeFrame(feedForwardBodyFrame);

      if (tempDesiredVelocityVector.length() < 1e-3)
      {
         expectedAverageVelocity.setIncludingFrame(tempDesiredVelocityVector);
      }
      else if (tempComTrajComputedVelocity.getX() > 0 && tempDesiredVelocityVector.getX() < 0
            || tempComTrajComputedVelocity.getX() < 0 && tempDesiredVelocityVector.getX() > 0)
      {
         expectedAverageVelocity.setIncludingFrame(tempDesiredVelocityVector);
         expectedAverageVelocity.scale(swingDuration.getDoubleValue() / 3.0);
      }
      else
      {
         expectedAverageVelocity.setIncludingFrame(tempDesiredVelocityVector);
         expectedAverageVelocity.sub(tempDesiredVelocityVector, tempComTrajComputedVelocity);
         expectedAverageVelocity.scale(swingDuration.getDoubleValue() / 3.0);
         expectedAverageVelocity.add(tempComTrajComputedVelocity);
      }

      double yawRate = desiredYawRate.getDoubleValue();
      swingTargetGenerator
            .getSwingTarget(swingLeg, feedForwardReferenceFrames.getLegAttachmentFrame(swingLeg), expectedAverageVelocity, swingDuration.getDoubleValue(),
                            framePointToPack, yawRate);
      framePointToPack.setZ(0.0);
   }

   private void drawSupportPolygon(QuadrupedSupportPolygon supportPolygon, YoFrameConvexPolygon2D yoFramePolygon)
   {
      tempSupportPolygonFramePointHolder.clear();
      for (RobotQuadrant quadrant : supportPolygon.getSupportingQuadrantsInOrder())
      {
         tempSupportPolygonFramePointHolder.add(quadrant).setIncludingFrame(supportPolygon.getFootstep(quadrant));
      }

      yoFramePolygon.set(FrameVertex3DSupplier.asFrameVertex3DSupplier(tempSupportPolygonFramePointHolder.values()));
   }

   /**
    * Uses the current CoM position to decide if it's safe to swing
    *
    */
   private class QuadrupleToTripleCondition implements StateTransitionCondition
   {
      private final QuadrupleSupportState quadrupleSupportState;

      public QuadrupleToTripleCondition(QuadrupleSupportState quadSupportState)
      {
         this.quadrupleSupportState = quadSupportState;
      }

      @Override
      public boolean testCondition(double timeInState)
      {
         if (!quadrupleSupportState.isMinimumTimeInQuadSupportElapsed(timeInState))
         {
            return false;
         }

         if (quadrupleSupportState.isTransitioningToSafePosition())
         {
            return false;
         }

         if (swingLeg.getEnumValue().isQuadrantInHind())
         {
            if (isVelocityNegative.getBooleanValue())
            {
               isCoMCloseToFinalDesired.set(true);
            }
            else
            {
               isCoMCloseToFinalDesired.set(quadrupleSupportState.isCoMCloseToFinalDesired(comCloseRadius.getDoubleValue()));
            }
         }
         else
         {
            if (isVelocityNegative.getBooleanValue())
            {
               isCoMCloseToFinalDesired.set(quadrupleSupportState.isCoMCloseToFinalDesired(comCloseRadius.getDoubleValue()));
            }
            else
            {
               isCoMCloseToFinalDesired.set(true);
            }
         }

         if (useCommonTriangleForSwingTransition.getBooleanValue())
         {
            isCoMInsideTriangleForSwingLeg.set(quadrupleSupportState.isCoMInsideCommonTriangleForSwingLeg(swingLeg.getEnumValue()));
         }
         else
         {
            isCoMInsideTriangleForSwingLeg.set(quadrupleSupportState.isCoMInsideTriangleForSwingLeg(swingLeg.getEnumValue()));
         }

         return isCoMCloseToFinalDesired.getBooleanValue() && isCoMInsideTriangleForSwingLeg.getBooleanValue() && (desiredVelocity.length() != 0.0
               || desiredYawRate.getDoubleValue() != 0); //bodyTrajectoryGenerator.isDone() &&
      }
   }

   private class TripleToQuadrupleCondition implements StateTransitionCondition
   {
      @Override
      public boolean testCondition(double timeInState)
      {
         RobotQuadrant swingQuadrant = swingLeg.getEnumValue();
         boolean swingTrajectoryIsDone = swingTrajectoryGenerators.get(swingQuadrant).isDone();
         boolean swingFootHitGround = false;
         boolean inSwingStateLongEnough = timeInState > swingDuration.getDoubleValue() / 3.0;

         if (!runOpenLoop.getBooleanValue() && footSwitches != null)
         {
            FootSwitchInterface footSwitch = footSwitches.get(swingQuadrant);
            swingFootHitGround = footSwitch.hasFootHitGround();
         }

         return ((swingTrajectoryIsDone || swingFootHitGround) && inSwingStateLongEnough);
      }
   }

   private class FilterToQuadrupleCondition implements StateTransitionCondition
   {
      private final FilterDesiredsToMatchCrawlControllerState filterDesiredsToMatchCrawlControllerState;

      public FilterToQuadrupleCondition(FilterDesiredsToMatchCrawlControllerState filterDesiredsToMatchCrawlControllerState)
      {
         this.filterDesiredsToMatchCrawlControllerState = filterDesiredsToMatchCrawlControllerState;
      }

      @Override
      public boolean testCondition(double timeInState)
      {
         return filterDesiredsToMatchCrawlControllerState.isInterpolationFinished();
      }
   }

   private class FilterDesiredsToMatchCrawlControllerState implements State
   {
      private final MinimumJerkTrajectory minimumJerkTrajectory = new MinimumJerkTrajectory();
      private final TDoubleArrayList initialPositions = new TDoubleArrayList();

      public void filterDesireds()
      {
         for (int i = 0; i < oneDoFJointsActual.length; i++)
         {
            OneDoFJoint actualOneDoFJoint = oneDoFJointsActual[i];

            double initialPosition = initialPositions.get(i);

            double alpha = minimumJerkTrajectory.getPosition(); //filterStandPrepDesiredsToWalkingDesireds.getDoubleValue();

            double alphaFilteredQ =
                  (1.0 - alpha) * initialPosition + alpha * jointDesiredOutputList.getJointDesiredOutput(actualOneDoFJoint).getDesiredPosition();

            jointDesiredOutputList.getJointDesiredOutput(actualOneDoFJoint).setDesiredPosition(alphaFilteredQ);
         }
      }

      public boolean isInterpolationFinished()
      {
         return minimumJerkTrajectory.getTimeInMove() >= timeToFilterDesiredAtCrawlStart
               .getDoubleValue(); //filterStandPrepDesiredsToWalkingDesireds.getDoubleValue() >= 0.9999;
      }

      @Override
      public void doAction(double timeInState)
      {
         double newTime = minimumJerkTrajectory.getTimeInMove() + dt;
         minimumJerkTrajectory.computeTrajectory(newTime);
      }

      @Override
      public void onEntry()
      {
         minimumJerkTrajectory.setMoveParameters(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, timeToFilterDesiredAtCrawlStart.getDoubleValue());

         initialPositions.clear();
         for (int i = 0; i < oneDoFJointsActual.length; i++)
         {
            OneDoFJoint actualOneDoFJoint = oneDoFJointsActual[i];
            initialPositions.add(jointDesiredOutputList.getJointDesiredOutput(actualOneDoFJoint).getDesiredPosition());
         }
      }

      @Override
      public void onExit()
      {

      }
   }

   private class QuadrupleSupportState implements State
   {
      private final FramePoint3D swingDesired = new FramePoint3D();
      private final QuadrupedSupportPolygon quadStateAfterFirstStep = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon quadStateAfterSecondStep = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon quadStateAfterThirdStep = new QuadrupedSupportPolygon();

      private final RecyclingQuadrantDependentList<QuadrupedSupportPolygon> upcomingTripleSupportPolygons = new RecyclingQuadrantDependentList<>(
            QuadrupedSupportPolygon.class);
      private final QuadrupedSupportPolygon tripleStateWithFirstStepSwinging = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon tripleStateAfterFirstStepWithSecondSwinging = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon tripleStateAfterSecondStepWithThirdSwinging = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon tripleStateAfterThirdStepWithFourthSwinging = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon temporaryQuadrupedSupportPolygonForCheckingCoMInsideTriangleForSwingLeg = new QuadrupedSupportPolygon();

      private final RecyclingQuadrantDependentList<QuadrupedSupportPolygon> estimatedCommonTriangle = new RecyclingQuadrantDependentList<>(
            QuadrupedSupportPolygon.class);
      private final YoDouble minimumTimeInQuadSupport;
      private final YoDouble minimumTimeInQuadSupportForNormalOperation;
      private final YoDouble minimumTimeInQuadSupportAfterReverseDirection;
      private final YoBoolean minimumTimeInQuadSupportElapsed;
      private final YoBoolean transitioningToSafePosition;
      private final YoBoolean swingLegUpdatedOnTransition;

      public QuadrupleSupportState(double minimumTimeInQuadSupport, double minimumTimeInQuadAfterReverseDirection)
      {
         this.minimumTimeInQuadSupport = new YoDouble("minimumTimeInQuadSupport", registry);
         this.minimumTimeInQuadSupportForNormalOperation = new YoDouble("minimumTimeInQuadSupportForNormalOperation", registry);
         this.minimumTimeInQuadSupportAfterReverseDirection = new YoDouble("minimumTimeInQuadSupportAfterReverseDirection", registry);
         this.minimumTimeInQuadSupportElapsed = new YoBoolean("minimumTimeInQuadSupportElapsed", registry);
         this.transitioningToSafePosition = new YoBoolean("transitioningToSafePosition", registry);
         this.swingLegUpdatedOnTransition = new YoBoolean("swingLegUpdatedOnTransition", registry);

         this.minimumTimeInQuadSupport.set(minimumTimeInQuadSupport);
         this.minimumTimeInQuadSupportForNormalOperation.set(minimumTimeInQuadSupport);
         this.minimumTimeInQuadSupportAfterReverseDirection.set(minimumTimeInQuadAfterReverseDirection);
      }

      @Override
      public void doAction(double timeInState)
      {
         doActionQuadrupleSupportTimer.startMeasurement();

         processVelocityChanges();

         if (isTransitioningToSafePosition() && comTrajectoryGenerator.isDone())
         {
            transitioningToSafePosition.set(false);
         }

         else if (shouldTranistionToTripleButItsNotSafeToStep(timeInState))
         {
            shiftCoMToSafeStartingPosition();
         }

         computeCurrentSupportPolygonAndDistanceInside(null);

         doActionQuadrupleSupportTimer.stopMeasurement();
      }

      /**
       * handles the 4 velocity cases
       * zero to nonZero
       * nonZero to zero
       * same sign change
       * reversing sign
       */
      private void processVelocityChanges()
      {
         if (isDesiredVelocityOrYawRateChanging())
         {
            //going to zero
            if (isDesiredVelocityAndYawRateZero() && !comTrajectoryGenerator.isDone())
            {
               setCoMTrajectoryToCurrentAndSetToDone();
            }

            if (!isDesiredVelocityAndYawRateZero())
            {
               //reversing sign
               if (isDesiredVelocityReversing())
               {
                  minimumTimeInQuadSupport.set(minimumTimeInQuadSupportAfterReverseDirection.getDoubleValue());
                  if (swingLegUpdatedOnTransition.getBooleanValue())
                  {
                     calculateNextSwingLeg();
                  }
                  calculateNextCoMTarget(true);
               }

               //same sign change
               else if (isVelocityChangingButKeepingSign())
               {
                  reinitializeCoMTrajectory();
               }

               //Coming from zero to nonZero
               else
               {
                  calculateNextCoMTarget(true);
               }
            }
         }

         lastDesiredVelocity.set(desiredVelocity);
         lastDesiredYawRate.set(desiredYawRate.getDoubleValue());
      }

      private void calculateNextSwingLeg()
      {
         desiredBodyVelocity.setIncludingFrame(desiredVelocity);
         RobotQuadrant lastSwingLeg = swingLeg.getEnumValue();
         RobotQuadrant newSwingLeg = nextSwingLegChooser
               .chooseNextSwingLeg(fourFootSupportPolygon, lastSwingLeg, desiredBodyVelocity, desiredYawRate.getDoubleValue());
         swingLeg.set(newSwingLeg);
      }

      private void calculateNextCoMTarget(boolean recalculateCurrent)
      {
         desiredBodyVelocity.setIncludingFrame(desiredVelocity);
         RobotQuadrant currentSwingLeg = swingLeg.getEnumValue();
         estimatedCommonTriangle.clear();
         if (recalculateCurrent)
         {
            RobotQuadrant previousSwingLeg = nextSwingLegChooser
                  .chooseNextSwingLeg(fourFootSupportPolygon, currentSwingLeg, lastDesiredVelocity, desiredYawRate.getDoubleValue());
            calculateNextThreeFootSteps(previousSwingLeg);
         }
         else
         {
            calculateNextThreeFootSteps(currentSwingLeg);
         }
         RobotQuadrant nextSwingLeg = nextSwingLegChooser
               .chooseNextSwingLeg(fourFootSupportPolygon, currentSwingLeg, desiredBodyVelocity, desiredYawRate.getDoubleValue());

         QuadrupedSupportPolygon quadrupedSupportPolygon = estimatedCommonTriangle.get(nextSwingLeg);
         if (quadrupedSupportPolygon != null)
         {
            calculateTrajectoryTarget(nextSwingLeg, quadrupedSupportPolygon, circleCenter2d);
            initializeCoMTrajectory(circleCenter2d);
         }
      }

      private void shiftCoMToSafeStartingPosition()
      {
         transitioningToSafePosition.set(true);
         RobotQuadrant currentSwingLeg = swingLeg.getEnumValue();
         RobotQuadrant sameSidQuadrant = currentSwingLeg.getSameSideQuadrant();
         RobotQuadrant diagonalQuadrant = currentSwingLeg.getDiagonalOppositeQuadrant();
         RobotQuadrant acrossBodyQuadrant = currentSwingLeg.getAcrossBodyQuadrant();

         FramePoint3D sameSideFootstep = fourFootSupportPolygon.getFootstep(sameSidQuadrant);
         FramePoint3D diagonalFootstep = fourFootSupportPolygon.getFootstep(diagonalQuadrant);
         FramePoint3D acrossBodyFootstep = fourFootSupportPolygon.getFootstep(acrossBodyQuadrant);

         centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         centerOfMassPoint2d.setIncludingFrame(centerOfMassFramePoint);

         FramePoint2D sameSideFootstep2d = new FramePoint2D(sameSideFootstep);
         FramePoint2D diagonalFootstep2d = new FramePoint2D(diagonalFootstep);
         FramePoint2D acrossBodyFootstep2d = new FramePoint2D(acrossBodyFootstep);
         FrameLineSegment2D lineSegment = new FrameLineSegment2D(ReferenceFrame.getWorldFrame());
         FramePoint2D comProjectionOnOutsideLegs2d = new FramePoint2D(ReferenceFrame.getWorldFrame());
         FramePoint3D comProjectionOnOutsideLegs = new FramePoint3D(ReferenceFrame.getWorldFrame());

         QuadrupedSupportPolygon tripleStateWithoutCurrentSwing = new QuadrupedSupportPolygon();
         fourFootSupportPolygon.getAndRemoveFootstep(tripleStateWithoutCurrentSwing, currentSwingLeg);

         switch (safeToShiftMode.getEnumValue())
         {
         case COMMON_TRIANGLE:
            RobotQuadrant nextSwingLeg = nextSwingLegChooser
                  .chooseNextSwingLeg(fourFootSupportPolygon, currentSwingLeg, desiredBodyVelocity, desiredYawRate.getDoubleValue());
            calculateNextThreeFootSteps(currentSwingLeg);
            QuadrupedSupportPolygon quadrupedSupportPolygon = estimatedCommonTriangle.get(nextSwingLeg);
            if (quadrupedSupportPolygon != null)
            {
               calculateTrajectoryTarget(nextSwingLeg, quadrupedSupportPolygon, circleCenter2d);
               initializeCoMTrajectory(circleCenter2d);
            }
            break;

         case CENTROID:
            tripleStateWithoutCurrentSwing.getCentroid2d(circleCenter2d);
            break;

         case COM_INCIRCLE:
            lineSegment.set(diagonalFootstep2d, acrossBodyFootstep2d);
            lineSegment.orthogonalProjection(centerOfMassPoint2d, comProjectionOnOutsideLegs2d);
            comProjectionOnOutsideLegs.set(comProjectionOnOutsideLegs2d, 0.0);

            safeToStepSupportPolygon.clear();
            safeToStepSupportPolygon.setFootstep(currentSwingLeg, centerOfMassFramePoint);
            safeToStepSupportPolygon.setFootstep(diagonalQuadrant, fourFootSupportPolygon.getFootstep(diagonalQuadrant));
            safeToStepSupportPolygon.setFootstep(acrossBodyQuadrant, comProjectionOnOutsideLegs);
            safeToStepSupportPolygon.getInCircle2d(circleCenter3d);
            circleCenter2d.set(circleCenter3d.getX(), circleCenter3d.getY());
            break;

         case TTR:
            tripleStateWithoutCurrentSwing
                  .getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(currentSwingLeg.getAcrossBodyQuadrant(), 0.1, circleCenter2d);
            break;

         case TROTLINE_MIDPOINT:
            lineSegment.set(sameSideFootstep2d, acrossBodyFootstep2d);

            FramePoint2D midpoint = new FramePoint2D();
            lineSegment.midpoint(midpoint);
            double bisectorLengthDesired = 0.1;
            FrameVector2D perpendicularBisector = new FrameVector2D();
            lineSegment.perpendicular(true, perpendicularBisector);
            perpendicularBisector.scale(-bisectorLengthDesired);
            circleCenter2d.add(midpoint, perpendicularBisector);
            if (!tripleStateWithoutCurrentSwing.isPointInside(circleCenter2d))
            {
               perpendicularBisector.scale(-1.0);
               circleCenter2d.add(midpoint, perpendicularBisector);
            }
            break;
         }

         /**
          * something went wrong!
          */
         if (!tripleStateWithoutCurrentSwing.isPointInside(circleCenter2d))
         {
            System.err.println(safeToShiftMode + " tried to shift outside of the support polygon. Fix this");
            tripleStateWithoutCurrentSwing.getCentroid2d(circleCenter2d);
         }

         initializeCoMTrajectory(circleCenter2d);
      }

      public boolean isTransitioningToSafePosition()
      {
         return transitioningToSafePosition.getBooleanValue();
      }

      private boolean isDesiredVelocityChanging()
      {
         boolean isBodyVelocityXChanging = Math.abs(desiredVelocity.getX() - lastDesiredVelocity.getX()) > 0.1e-3;
         boolean isBodyVelocityYChanging = Math.abs(desiredVelocity.getY() - lastDesiredVelocity.getY()) > 0.1e-3;
         return isBodyVelocityXChanging || isBodyVelocityYChanging;
      }

      private boolean isDesiredYawRateChanging()
      {
         boolean isYawRateChanging = Math.abs(desiredYawRate.getDoubleValue() - lastDesiredYawRate.getDoubleValue()) > 0.1e-3;
         return isYawRateChanging;
      }

      private boolean isDesiredVelocityOrYawRateChanging()
      {
         boolean isDesiredVelocityChanging = isDesiredVelocityChanging();
         boolean isDesiredYawRateChanging = isDesiredYawRateChanging();
         return isDesiredVelocityChanging || isDesiredYawRateChanging;
      }

      private boolean isVelocityChangingButKeepingSign()
      {
         return desiredVelocity.getX() > 0 && lastDesiredVelocity.getX() > 0 || desiredVelocity.getX() < 0 && lastDesiredVelocity.getX() < 0;
      }

      private boolean isDesiredVelocityReversing()
      {
         if (Math.abs(desiredVelocity.getX()) < 1.0e-5)
         {
            return false;
         }

         return desiredVelocity.getX() > 0 && lastDesiredVelocity.getX() < 0 || desiredVelocity.getX() < 0 && lastDesiredVelocity.getX() > 0;
      }

      private boolean shouldTranistionToTripleButItsNotSafeToStep(double timeInState)
      {
         if (!comTrajectoryGenerator.isDone())
         {
            return false;
         }

         if (!isMinimumTimeInQuadSupportElapsed(timeInState))
         {
            return false;
         }

         if (desiredVelocity.length() == 0.0 && desiredYawRate.getDoubleValue() == 0)
         {
            return false;
         }

         return true;
      }

      /**
       * uses actual to calculate the next three footsteps. Can use feedforward based on desireds instead of the fourFootSupportPolygon
       */
      @Override
      public void onEntry()
      {
         doTransitionIntoQuadrupleSupportTimer.startMeasurement();

         if (!isDesiredVelocityAndYawRateZero() && !isDesiredVelocityReversing())
         {
            swingLegUpdatedOnTransition.set(true);
            calculateNextSwingLeg();
            calculateNextCoMTargetTimer.startMeasurement();
            calculateNextCoMTarget(false);
            calculateNextCoMTargetTimer.stopMeasurement();
         }

         doTransitionIntoQuadrupleSupportTimer.stopMeasurement();
      }

      public void calculateNextThreeFootSteps2(RobotQuadrant firstSwingLeg)
      {
         RobotQuadrant secondSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, firstSwingLeg, desiredBodyVelocity, desiredYawRate
               .getDoubleValue());//firstSwingLeg.getNextRegularGaitSwingQuadrant();
         RobotQuadrant thirdSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, secondSwingLeg, desiredBodyVelocity, desiredYawRate
               .getDoubleValue());//secondSwingLeg.getNextRegularGaitSwingQuadrant();
         RobotQuadrant fourthSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, thirdSwingLeg, desiredBodyVelocity, desiredYawRate
               .getDoubleValue());//thirdSwingLeg.getNextRegularGaitSwingQuadrant();

         FrameVector3DReadOnly desiredVelocityVector = desiredVelocity;
         double yawRate = desiredYawRate.getDoubleValue();

         swingDesired.changeFrame(ReferenceFrame.getWorldFrame());
         calculateSwingTarget(firstSwingLeg, swingDesired);
         fourFootSupportPolygon.getAndReplaceFootstep(quadStateAfterFirstStep, firstSwingLeg, swingDesired);

         swingTargetGenerator.getSwingTarget(quadStateAfterFirstStep, secondSwingLeg, desiredVelocityVector, swingDesired, yawRate);
         quadStateAfterFirstStep.getAndReplaceFootstep(quadStateAfterSecondStep, secondSwingLeg, swingDesired);

         swingTargetGenerator.getSwingTarget(quadStateAfterSecondStep, thirdSwingLeg, desiredVelocityVector, swingDesired, yawRate);
         quadStateAfterSecondStep.getAndReplaceFootstep(quadStateAfterThirdStep, thirdSwingLeg, swingDesired);

         quadStateAfterFirstStep.getAndRemoveFootstep(tripleStateWithFirstStepSwinging, secondSwingLeg);
         quadStateAfterFirstStep.getAndRemoveFootstep(tripleStateAfterFirstStepWithSecondSwinging, secondSwingLeg);
         quadStateAfterSecondStep.getAndRemoveFootstep(tripleStateAfterSecondStepWithThirdSwinging, thirdSwingLeg);
         quadStateAfterThirdStep.getAndRemoveFootstep(tripleStateAfterThirdStepWithFourthSwinging, fourthSwingLeg);

         drawSupportPolygon(tripleStateWithFirstStepSwinging, tripleSupportPolygons.get(firstSwingLeg));
         drawSupportPolygon(tripleStateAfterFirstStepWithSecondSwinging, tripleSupportPolygons.get(secondSwingLeg));
         QuadrupedSupportPolygon firstAndSecondCommonPolygon = tempCommonShrunkenPolygon;
         tripleStateWithFirstStepSwinging
               .getShrunkenCommonTriangle2d(tripleStateAfterFirstStepWithSecondSwinging, firstAndSecondCommonPolygon, tempPolygonForCommonShrunken,
                                            firstSwingLeg, shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue(),
                                            shrunkenPolygonSize.getDoubleValue());
         estimatedCommonTriangle.add(firstSwingLeg).set(firstAndSecondCommonPolygon);
         estimatedCommonTriangle.add(firstSwingLeg.getSameSideQuadrant());
         estimatedCommonTriangle.get(firstSwingLeg.getSameSideQuadrant()).clear();
         firstAndSecondCommonPolygon.getAndSwapSameSideFootsteps(estimatedCommonTriangle.get(firstSwingLeg.getSameSideQuadrant()), firstSwingLeg.getSide());
         drawSupportPolygon(firstAndSecondCommonPolygon, commonTriplePolygons.get(firstSwingLeg.getSide()));

         drawSupportPolygon(tripleStateAfterFirstStepWithSecondSwinging, tripleSupportPolygons.get(thirdSwingLeg));
         drawSupportPolygon(tripleStateAfterSecondStepWithThirdSwinging, tripleSupportPolygons.get(fourthSwingLeg));
         QuadrupedSupportPolygon secondAndThirdCommonPolygon = tempCommonShrunkenPolygon;
         tripleStateAfterFirstStepWithSecondSwinging
               .getShrunkenCommonTriangle2d(tripleStateAfterSecondStepWithThirdSwinging, secondAndThirdCommonPolygon, tempPolygonForCommonShrunken,
                                            secondSwingLeg, shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue(),
                                            shrunkenPolygonSize.getDoubleValue());
         estimatedCommonTriangle.add(secondSwingLeg).set(secondAndThirdCommonPolygon);
         estimatedCommonTriangle.add(secondSwingLeg.getSameSideQuadrant());
         estimatedCommonTriangle.get(secondSwingLeg.getSameSideQuadrant()).clear();
         secondAndThirdCommonPolygon.getAndSwapSameSideFootsteps(estimatedCommonTriangle.get(secondSwingLeg.getSameSideQuadrant()), secondSwingLeg.getSide());
         drawSupportPolygon(secondAndThirdCommonPolygon, commonTriplePolygons.get(secondSwingLeg.getSide()));

         drawSupportPolygon(tripleStateAfterSecondStepWithThirdSwinging, tripleSupportPolygons.get(thirdSwingLeg));
         drawSupportPolygon(tripleStateAfterThirdStepWithFourthSwinging, tripleSupportPolygons.get(fourthSwingLeg));
         QuadrupedSupportPolygon thirdAndFourthCommonPolygon = tempCommonShrunkenPolygon;
         tripleStateAfterSecondStepWithThirdSwinging
               .getShrunkenCommonTriangle2d(tripleStateAfterThirdStepWithFourthSwinging, thirdAndFourthCommonPolygon, tempPolygonForCommonShrunken,
                                            thirdSwingLeg, shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue(),
                                            shrunkenPolygonSize.getDoubleValue());
         estimatedCommonTriangle.add(thirdSwingLeg).set(thirdAndFourthCommonPolygon);
         estimatedCommonTriangle.add(thirdSwingLeg.getSameSideQuadrant());
         estimatedCommonTriangle.get(thirdSwingLeg.getSameSideQuadrant()).clear();
         thirdAndFourthCommonPolygon.getAndSwapSameSideFootsteps(estimatedCommonTriangle.get(thirdSwingLeg.getSameSideQuadrant()), thirdSwingLeg.getSide());
         drawSupportPolygon(thirdAndFourthCommonPolygon, commonTriplePolygons.get(thirdSwingLeg.getSide()));
      }

      public void calculateNextThreeFootSteps(RobotQuadrant firstSwingLeg)
      {
         calculateNextThreeFootstepsTimer.startMeasurement();

         // 1.7 MS START
         FrameVector3DReadOnly desiredVelocityVector = desiredVelocity;
         double yawRate = desiredYawRate.getDoubleValue();
         double shrunkenPolygonOffset = shrunkenPolygonSize.getDoubleValue();

         calculateSwingTarget(firstSwingLeg, swingDesired);

         fourFootSupportPolygon.getAndReplaceFootstep(quadStateAfterFirstStep, firstSwingLeg, swingDesired);

         RobotQuadrant secondSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(quadStateAfterFirstStep, firstSwingLeg, desiredVelocityVector, yawRate);
         quadStateAfterSecondStep.set(quadStateAfterFirstStep);
         calculateNextSupportPolygon(quadStateAfterSecondStep, secondSwingLeg, desiredVelocityVector, yawRate, quadStateAfterSecondStep);

         RobotQuadrant thirdSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(quadStateAfterSecondStep, secondSwingLeg, desiredVelocityVector, yawRate);
         quadStateAfterThirdStep.set(quadStateAfterSecondStep);
         calculateNextSupportPolygon(quadStateAfterThirdStep, thirdSwingLeg, desiredVelocityVector, yawRate, quadStateAfterThirdStep);

         RobotQuadrant fourthSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, thirdSwingLeg, desiredVelocityVector, yawRate);
         // 1.7 MS STOP

         upcomingTripleSupportPolygons.add(firstSwingLeg).clear();
         fourFootSupportPolygon.getAndRemoveFootstep(upcomingTripleSupportPolygons.get(firstSwingLeg), firstSwingLeg);
         upcomingTripleSupportPolygons.add(secondSwingLeg).clear();
         fourFootSupportPolygon.getAndRemoveFootstep(upcomingTripleSupportPolygons.get(secondSwingLeg), firstSwingLeg);

         fourFootSupportPolygon.getAndRemoveFootstep(tripleStateWithFirstStepSwinging, secondSwingLeg);
         quadStateAfterFirstStep.getAndRemoveFootstep(tripleStateAfterFirstStepWithSecondSwinging, secondSwingLeg);
         quadStateAfterSecondStep.getAndRemoveFootstep(tripleStateAfterSecondStepWithThirdSwinging, thirdSwingLeg);
         quadStateAfterThirdStep.getAndRemoveFootstep(tripleStateAfterThirdStepWithFourthSwinging, fourthSwingLeg);

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            tripleSupportPolygons.get(robotQuadrant).clear();
         }

         // 5.1/7.2 MS 71% START
         if (!oneOffHappened)
            calculateNextThreeFootStepsOneOffTimer.startMeasurement();
         updateAndDrawCommonTriangle(tripleStateWithFirstStepSwinging, tripleStateAfterFirstStepWithSecondSwinging, firstSwingLeg, secondSwingLeg,
                                     shrunkenPolygonOffset);
         updateAndDrawCommonTriangle(tripleStateAfterFirstStepWithSecondSwinging, tripleStateAfterSecondStepWithThirdSwinging, secondSwingLeg, thirdSwingLeg,
                                     shrunkenPolygonOffset);
         updateAndDrawCommonTriangle(tripleStateAfterSecondStepWithThirdSwinging, tripleStateAfterThirdStepWithFourthSwinging, thirdSwingLeg, fourthSwingLeg,
                                     shrunkenPolygonOffset);
         if (!oneOffHappened)
            calculateNextThreeFootStepsOneOffTimer.stopMeasurement();
         // 5.1/7.2 MS 71% STOP

         calculateNextThreeFootstepsTimer.stopMeasurement();

         oneOffHappened = true;
      }

      private void calculateNextSupportPolygon(QuadrupedSupportPolygon supportPolygon, RobotQuadrant nextSwingLeg, FrameVector3DReadOnly desiredVelocity,
                                               double desiredYawRate, QuadrupedSupportPolygon quadrupedSupportPolygonToPack)
      {
         swingDesired.changeFrame(ReferenceFrame.getWorldFrame());
         swingTargetGenerator.getSwingTarget(supportPolygon, nextSwingLeg, desiredVelocity, swingDesired, desiredYawRate);
         quadrupedSupportPolygonToPack.set(supportPolygon);
         quadrupedSupportPolygonToPack.setFootstep(nextSwingLeg, swingDesired);
      }

      private void updateAndDrawCommonTriangle(QuadrupedSupportPolygon firstTripleSupportPolygon, QuadrupedSupportPolygon secondTripleSupportPolygon,
                                               RobotQuadrant firstSwingLeg, RobotQuadrant secondSwingLeg, double shrunkenPolygonOffset)
      {
         if (firstTripleSupportPolygon.getFirstNonSupportingQuadrant().getSameSideQuadrant() != secondTripleSupportPolygon.getFirstNonSupportingQuadrant())
         {
            return;
         }

         firstTripleSupportPolygon
               .getShrunkenCommonTriangle2d(secondTripleSupportPolygon, tempCommonShrunkenPolygon, tempPolygonForCommonShrunken, firstSwingLeg,
                                            shrunkenPolygonOffset, shrunkenPolygonOffset, shrunkenPolygonOffset);
         estimatedCommonTriangle.add(firstSwingLeg).set(tempCommonShrunkenPolygon);
         estimatedCommonTriangle.add(firstSwingLeg.getSameSideQuadrant()).clear();
         tempCommonShrunkenPolygon.getAndSwapSameSideFootsteps(estimatedCommonTriangle.get(firstSwingLeg.getSameSideQuadrant()), firstSwingLeg.getSide());
         drawSupportPolygon(tempCommonShrunkenPolygon, commonTriplePolygons.get(firstSwingLeg.getSide()));
         YoFrameConvexPolygon2D firstTripleSupportPolygonGraphic = tripleSupportPolygons.get(firstSwingLeg);
         firstTripleSupportPolygonGraphic.clear();
         drawSupportPolygon(firstTripleSupportPolygon, firstTripleSupportPolygonGraphic);
         drawSupportPolygon(secondTripleSupportPolygon, tripleSupportPolygons.get(secondSwingLeg));
      }

      private final FrameVector2D tempFrameVector = new FrameVector2D();

      private void calculateTrajectoryTarget(RobotQuadrant upcomingSwingLeg, QuadrupedSupportPolygon commonTriangle, FramePoint2D comTargetToPack)
      {
         commonSupportPolygon.set(commonTriangle);
         double radius = subCircleRadius.getDoubleValue();
         boolean hasEnoughSides = commonSupportPolygon.getNumberOfVertices() >= 3;
         boolean requestedRadiusLargerThanInCircle = true;
         if (useSubCircleForBodyShiftTarget.getBooleanValue() && hasEnoughSides)
         {
            requestedRadiusLargerThanInCircle = !commonSupportPolygon
                  .getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(upcomingSwingLeg, radius, comTargetToPack);
         }
         if (hasEnoughSides && requestedRadiusLargerThanInCircle)
         {
            radius = commonSupportPolygon.getInCircle2d(circleCenter3d);
            comTargetToPack.set(circleCenter3d.getX(), circleCenter3d.getY());
         }
         inscribedCircleRadius.set(radius);

         tempFrameVector.setIncludingFrame(desiredCoMOffset);
         //         tempFrameVector.scale(velocitySign.getDoubleValue());
         tempFrameVector.changeFrame(ReferenceFrame.getWorldFrame());

         comTargetToPack.add(tempFrameVector);
         circleCenter.set(comTargetToPack, 0.0);
      }

      public void reinitializeCoMTrajectory()
      {
         FramePoint3D finalPosition = new FramePoint3D(ReferenceFrame.getWorldFrame());
         comTrajectoryGenerator.getFinalPosition(finalPosition);
         initializeCoMTrajectory(new FramePoint2D(ReferenceFrame.getWorldFrame(), finalPosition.getX(), finalPosition.getY()));
      }

      public void setCoMTrajectoryToCurrentAndSetToDone()
      {
         desiredCoMVelocity.setToZero();
         FramePoint3D desiredBodyCurrent = new FramePoint3D(desiredCoMPose.getPosition());
         initialCoMPosition.set(desiredBodyCurrent);
         initializeCoMTrajectory(new FramePoint2D(ReferenceFrame.getWorldFrame(), initialCoMPosition.getX(), initialCoMPosition.getY()));
         comTrajectoryGenerator.setToDone();
      }

      Random random = new Random(100L);

      private void initializeCoMTrajectory(FramePoint2D target)
      {
         FramePoint3DReadOnly desiredBodyCurrent = desiredCoMPose.getPosition();
         initialCoMPosition.set(desiredBodyCurrent);

         FrameVector3DReadOnly desiredBodyVelocityCurrent = desiredCoMVelocity;
         initialCoMVelocity.set(desiredBodyVelocityCurrent);

         desiredCoMTarget.set(target, desiredBodyCurrent.getZ());

         double distance = initialCoMPosition.distance(desiredCoMTarget);
         desiredBodyVelocity.setIncludingFrame(desiredVelocity);

         if (!MathTools.epsilonEquals(desiredBodyVelocity.length(), 0.0, 1e-5))
         {
            // Use average velocity to estimate how long it'll take.
            // Otherwise, when going from fast to slow, it'll have a long duration but
            // a large initial velocity and thereby overshoot a lot.
            comTrajectoryDuration.set(distance / (0.5 * (desiredBodyVelocity.length() + initialCoMVelocity.length())));
         }
         else
         {
            comTrajectoryDuration.set(swingDuration.getDoubleValue() * 2 + turnInPlaceCoMTrajectoryBuffer.getDoubleValue());
         }

         if (comTrajectoryDuration.getDoubleValue() > maximumCoMTrajectoryDuration.getDoubleValue())
            comTrajectoryDuration.set(maximumCoMTrajectoryDuration.getDoubleValue());
         if (comTrajectoryDuration.getDoubleValue() < minimumCoMTrajectoryDuration.getDoubleValue())
            comTrajectoryDuration.set(minimumCoMTrajectoryDuration.getDoubleValue());

         //PDN: this is just so I can see when the value changes
         comTrajectoryDuration.set(comTrajectoryDuration.getDoubleValue() + random.nextDouble() / 1000.0);
         comTrajectoryGenerator.setTrajectoryTime(comTrajectoryDuration.getDoubleValue());

         desiredBodyVelocity.changeFrame(ReferenceFrame.getWorldFrame());

         comTrajectoryTimeStart.set(robotTimestamp.getDoubleValue());
         comTrajectoryTimeLastCalled.set(comTrajectoryTimeStart.getDoubleValue());
         comTrajectoryTimeCurrent.set(0.0);
         comTrajectoryGenerator.setInitialConditions(initialCoMPosition, initialCoMVelocity);
         comTrajectoryGenerator.setFinalConditions(desiredCoMTarget, desiredBodyVelocity);
         comTrajectoryGenerator.initialize();
         comTrajectoryGeneratorRequiresReInitailization.set(false);
      }

      public boolean isMinimumTimeInQuadSupportElapsed(double timeInState)
      {
         if (timeInState > minimumTimeInQuadSupport.getDoubleValue())
         {
            minimumTimeInQuadSupportElapsed.set(true);
         }
         return minimumTimeInQuadSupportElapsed.getBooleanValue();
      }

      @Override
      public void onExit()
      {
         minimumTimeInQuadSupportElapsed.set(false);
         minimumTimeInQuadSupport.set(minimumTimeInQuadSupportForNormalOperation.getDoubleValue());
         swingLegUpdatedOnTransition.set(false);
      }

      public boolean isCommonTriangleNull(RobotQuadrant swingLeg)
      {
         return estimatedCommonTriangle.get(swingLeg) == null;
      }

      public boolean isCoMInsideCommonTriangleForSwingLeg(RobotQuadrant swingLeg)
      {
         if (isCommonTriangleNull(swingLeg))
         {
            return false;
         }
         centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         centerOfMassPoint2d.setIncludingFrame(centerOfMassFramePoint);

         return estimatedCommonTriangle.get(swingLeg).isPointInside(centerOfMassFramePoint);
      }

      /**
       * uses actuals
       */
      public boolean isCoMInsideTriangleForSwingLeg(RobotQuadrant swingLeg)
      {
         centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         centerOfMassPoint2d.setIncludingFrame(centerOfMassFramePoint);
         fourFootSupportPolygon.getAndRemoveFootstep(temporaryQuadrupedSupportPolygonForCheckingCoMInsideTriangleForSwingLeg, swingLeg);
         //         return temporaryQuadrupedSupportPolygonForCheckingCoMInsideTriangleForSwingLeg.isInside(centerOfMassFramePoint);
         return -temporaryQuadrupedSupportPolygonForCheckingCoMInsideTriangleForSwingLeg.signedDistance(centerOfMassFramePoint)
               > distanceInsideSupportPolygonBeforeSwingingLeg.getDoubleValue();
      }

      public boolean isCoMCloseToFinalDesired(double distanceToCheck)
      {
         double distanceFromDesiredToTarget = desiredCoMTarget.distance(desiredCoMFramePosition);
         return (distanceFromDesiredToTarget < distanceToCheck);
      }
   }

   /**
    * Nothing fancy, just calculate where to swing the leg and swing it until it's done
    */
   private class TripleSupportState implements State
   {
      private final FramePoint3D swingTarget = new FramePoint3D(ReferenceFrame.getWorldFrame());
      private final FramePoint3D currentDesiredInTrajectory = new FramePoint3D();
      private final FrameVector3D speedMatchVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
      private final YoDouble speedMatchScalar = new YoDouble("speedMatchScalar", registry);

      public TripleSupportState()
      {
         speedMatchScalar.set(0.0);
      }

      private FramePoint3D tempFramePoint = new FramePoint3D(ReferenceFrame.getWorldFrame());

      @Override
      public void doAction(double timeInState)
      {
         doActionTripleSupportTimer.startMeasurement();

         RobotQuadrant swingQuadrant = swingLeg.getEnumValue();

         computeFootPositionAlongSwingTrajectory(swingQuadrant, currentDesiredInTrajectory);
         currentDesiredInTrajectory.changeFrame(ReferenceFrame.getWorldFrame());
         currentSwingTarget.set(currentDesiredInTrajectory);

         desiredFeetLocations.get(swingQuadrant).setMatchingFrame(currentDesiredInTrajectory);

         computeCurrentSupportPolygonAndDistanceInside(swingQuadrant);

         //Only need to slow down if com target is on other side of trot line
         double thresholdDistance = 0.01;
         comTrajectoryGenerator.getFinalPosition(tempFramePoint);

         distanceToTrotLine.set(computeDistanceToTrotLine2d(swingQuadrant));

         doActionTripleSupportTimer.stopMeasurement();
      }

      FramePoint3D tempCOMTarget = new FramePoint3D(ReferenceFrame.getWorldFrame());

      @Override
      public void onEntry()
      {
         doTransitionIntoTripleSupportTimer.startMeasurement();

         RobotQuadrant swingQuadrant = swingLeg.getEnumValue();
         swingTarget.setToZero(ReferenceFrame.getWorldFrame());
         calculateSwingTarget(swingQuadrant, swingTarget);

         YoFramePoint3D yoDesiredFootPosition = desiredFeetLocations.get(swingQuadrant);
         currentSwingTarget.set(swingTarget);
         finalSwingTarget.set(swingTarget);

         initializeSwingTrajectory(swingQuadrant, yoDesiredFootPosition, swingTarget, swingDuration.getDoubleValue());
         doTransitionIntoTripleSupportTimer.stopMeasurement();

         //Determine if we need to slow down body motion
         distanceToTrotLine.set(computeDistanceToTrotLine2d(swingQuadrant));
         distanceTravelAtEndOfSwing.set(distanceToTravelInGivenTime(swingDuration.getDoubleValue()));

         //only need to slow if COM target is outside of triple support polygon
         comTrajectoryGenerator.getFinalPosition(tempCOMTarget);
         boolean targetIsInSupportPolygon = isCOMTargetInsideSupportPolygon(swingQuadrant, tempCOMTarget, 0.01);

         timeToSlowTo.set(swingDuration.getDoubleValue());
         if (targetIsInSupportPolygon)
         {
            slowBodyDown.set(false);
            comTrajectoryTimeScaleFactor.set(1.0);
         }
         else
         {
            double stabilityMargin = distanceInsideSupportPolygonBeforeSwingingLeg.getDoubleValue();
            double distanceOverTravel = distanceToTrotLine.getDoubleValue() - distanceTravelAtEndOfSwing.getDoubleValue() - stabilityMargin;
            if (distanceOverTravel < 0.0)
            {
               slowBodyDown.set(true);
               timeToSlowTo.set(getTimeToTravelDistance(distanceToTrotLine.getDoubleValue() - stabilityMargin, swingDuration.getDoubleValue()));
               comTrajectoryTimeScaleFactor.set(timeToSlowTo.getDoubleValue() / swingDuration.getDoubleValue());
            }
            else
            {
               slowBodyDown.set(false);
               comTrajectoryTimeScaleFactor.set(1.0);
            }
         }
         if (footSwitches != null)
         {
            footSwitches.get(swingQuadrant).setFootContactState(false);
         }
      }

      FramePoint3D tempStartPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
      FramePoint3D tempEndPoint = new FramePoint3D(ReferenceFrame.getWorldFrame());

      private double getTimeToTravelDistance(double distanceToTravel, double swingTime)
      {
         double currentTimeForCOMTrajectoryGenerator = comTrajectoryGenerator.getCurrentTime();
         //recompute just to be safe
         comTrajectoryGenerator.compute(currentTimeForCOMTrajectoryGenerator);
         comTrajectoryGenerator.getPosition(tempStartPoint);

         int maxNumberOfTests = 20;
         double deltaT = swingTime / ((double) maxNumberOfTests);
         double timeToSlowTo = swingTime;
         for (double timeToTest = swingTime; timeToTest > deltaT; timeToTest = timeToTest - deltaT)
         {
            timeToSlowTo = timeToTest;
            comTrajectoryGenerator.compute(timeToTest);
            comTrajectoryGenerator.getPosition(tempEndPoint);
            double distanceTraveled = tempStartPoint.distanceXY(tempEndPoint);
            if (distanceTraveled <= distanceToTravel)
               break;
         }

         //set it back to what it was before
         comTrajectoryGenerator.compute(currentTimeForCOMTrajectoryGenerator);

         return timeToSlowTo;
      }

      private double distanceToTravelInGivenTime(double timeElapsed)
      {
         double currentTimeForCOMTrajectoryGenerator = comTrajectoryGenerator.getCurrentTime();

         //recompute just to be safe
         comTrajectoryGenerator.compute(currentTimeForCOMTrajectoryGenerator);
         comTrajectoryGenerator.getPosition(tempStartPoint);

         //compute position after time in future
         comTrajectoryGenerator.compute(currentTimeForCOMTrajectoryGenerator + timeElapsed);
         comTrajectoryGenerator.getPosition(tempEndPoint);

         double distanceTraveled = tempStartPoint.distanceXY(tempEndPoint);

         //set it back to what it was before
         comTrajectoryGenerator.compute(currentTimeForCOMTrajectoryGenerator);

         return distanceTraveled;
      }

      @Override
      public void onExit()
      {
         RobotQuadrant swingQuadrant = swingLeg.getEnumValue();
         if (footSwitches != null)
         {
            footSwitches.get(swingQuadrant).setFootContactState(true);
         }
         //reset this just in case
         comTrajectoryTimeScaleFactor.set(1.0);
      }

      private boolean isCOMTargetInsideSupportPolygon(RobotQuadrant swingQuadrant, FramePoint3D cOMTarget, double thresholdDistance)
      {
         currentSupportPolygon.set(fourFootSupportPolygon);
         if (swingQuadrant != null)
            currentSupportPolygon.removeFootstep(swingQuadrant);
         cOMTarget.changeFrame(ReferenceFrame.getWorldFrame());
         double distanceInside = -currentSupportPolygon.signedDistance(cOMTarget);
         return distanceInside > thresholdDistance;
      }

      /**
       * tries to do some fancy ground speed matching, not well tuned
       */
      private void initializeSwingTrajectory(RobotQuadrant swingLeg, FramePoint3DReadOnly swingInitial, FramePoint3DReadOnly swingTarget, double swingTime)
      {
         QuadrupedSwingTrajectoryGenerator swingTrajectoryGenerator = swingTrajectoryGenerators.get(swingLeg);
         speedMatchVelocity.setIncludingFrame(desiredBodyVelocity);
         speedMatchVelocity.changeFrame(ReferenceFrame.getWorldFrame());
         speedMatchVelocity.scale(speedMatchScalar.getDoubleValue());
         swingTrajectoryGenerator.initializeSwing(swingTime, swingInitial, swingHeight.getDoubleValue(), swingTarget, speedMatchVelocity);
      }

      private void computeFootPositionAlongSwingTrajectory(RobotQuadrant swingLeg, FramePoint3D framePointToPack)
      {
         QuadrupedSwingTrajectoryGenerator swingTrajectoryGenerator = swingTrajectoryGenerators.get(swingLeg);
         swingTrajectoryGenerator.computeSwing(framePointToPack);

         swingTimeRemaining.set(swingTrajectoryGenerator.getTimeRemaining());
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void onEntry()
   {
      for (OneDoFJoint oneDofJoint : oneDoFJointsActual)
      {
         oneDofJoint.setUnderPositionControl(true);
      }

      actualFullRobotModel.updateFrames();
      referenceFrames.updateFrames();

      //      setFeedForwardToActuals();
      updateEstimates();
      updateFeedForwardModelAndFrames();
      walkingStateMachine.resetToInitialState();
   }

   @Override
   public void onExit()
   {

   }
}
