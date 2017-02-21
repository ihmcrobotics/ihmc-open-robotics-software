package us.ihmc.quadrupedRobotics.controller.position.states;

import java.awt.Color;
import java.util.Random;

import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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
import us.ihmc.quadrupedRobotics.mechanics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
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
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.filters.AlphaFilteredWrappingYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFramePoint;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.VelocityConstrainedPositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RecyclingQuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.robotics.trajectories.MinimumJerkTrajectory;

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
   private final DoubleYoVariable robotTimestamp;

   public enum CrawlGateWalkingState
   {
      QUADRUPLE_SUPPORT, TRIPLE_SUPPORT, ALPHA_FILTERING_DESIREDS
   }

   private enum SafeStartingShiftMode
   {
      COMMON_TRIANGLE, CENTROID, TTR, COM_INCIRCLE, TROTLINE_MIDPOINT
   };

   private final EnumYoVariable<SafeStartingShiftMode> safeToShiftMode = new EnumYoVariable<>("safeStartingShiftMode", registry, SafeStartingShiftMode.class);
   {
      safeToShiftMode.set(SafeStartingShiftMode.TROTLINE_MIDPOINT);
   }

   private final FullQuadrupedRobotModel feedForwardFullRobotModel;
   private final QuadrupedReferenceFrames feedForwardReferenceFrames;

   private final StateMachine<CrawlGateWalkingState> walkingStateMachine;
   private final QuadrupleSupportState quadrupleSupportState;
   private final FilterDesiredsToMatchCrawlControllerState filterDesiredsToMatchCrawlControllerOnTransitionIn;
   private final QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators;
   private final NextSwingLegChooser nextSwingLegChooser;
   private final SwingTargetGenerator swingTargetGenerator;
   private final QuadrantDependentList<FootSwitchInterface> footSwitches;
   private final FullRobotModel actualFullRobotModel;
   private final FloatingInverseDynamicsJoint actualRobotRootJoint;

   private final QuadrupedReferenceFrames referenceFrames;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final CenterOfMassJacobian feedForwardCenterOfMassJacobian;
   private final FramePoint feedForwardCoMPosition = new FramePoint();
   private final FramePoint tempCoMPosition = new FramePoint();
   private final ReferenceFrame feedForwardBodyFrame;
   private final ReferenceFrame comFrame;
   private final DoubleYoVariable feedForwardCenterOfMassOffsetAlpha;
   private final YoFramePoint feedForwardCenterOfMassOffset;
   private final AlphaFilteredYoFramePoint filteredFeedForwardCenterOfMassOffset;

   private final TranslationReferenceFrame feedForwardCenterOfMassFrame;
   private final PoseReferenceFrame desiredCoMPoseReferenceFrame = new PoseReferenceFrame("desiredCoMPoseReferenceFrame", ReferenceFrame.getWorldFrame());
   private final YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector desiredCoMVelocity = new YoFrameVector("desiredCoMVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final GlobalTimer calculateNextThreeFootstepsTimer = new GlobalTimer("calculateNextThreeFootstepsTimer", registry);
   private final GlobalTimer doTransitionIntoQuadrupleSupportTimer = new GlobalTimer("doTransitionIntoQuadrupleSupportTimer", registry);
   private final GlobalTimer doTransitionIntoTripleSupportTimer = new GlobalTimer("doTransitionIntoTripleSupportTimer", registry);
   private final GlobalTimer doActionTripleSupportTimer = new GlobalTimer("doActionTripleSupportTimer", registry);
   private final GlobalTimer doActionQuadrupleSupportTimer = new GlobalTimer("doActionQuadrupleSupportTimer", registry);
   private final GlobalTimer doActionTimer = new GlobalTimer("doActionTimer", registry);
   private final GlobalTimer calculateNextCoMTargetTimer = new GlobalTimer("calculateNextCoMTargetTimer", registry);
   private final GlobalTimer calculateNextThreeFootStepsOneOffTimer = new GlobalTimer("calculateNextThreeFootStepsOneOffTimer", registry);
   private boolean oneOffHappened = false;

   private final OneDoFJoint[] oneDoFJointsFeedforward;
   private final OneDoFJoint[] oneDoFJointsActual;

   private final FramePoint desiredCoMFramePosition = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FramePose desiredCoMFramePose = new FramePose(ReferenceFrame.getWorldFrame());

   private final FrameVector tempComTrajComputedVelocity = new FrameVector();
   private final FrameVector tempDesiredVelocityVector = new FrameVector();

   private final BooleanYoVariable runOpenLoop = new BooleanYoVariable("runOpenLoop", "If true, runs in open loop mode. The leg motions will not depend on any feedback signals.", registry);

   private final YoFramePoint2d desiredCoMOffset;
   private final DoubleYoVariable distanceInsideSupportPolygonBeforeSwingingLeg = new DoubleYoVariable("distanceInsideSupportPolygonBeforeSwingingLeg", registry);

   private final DoubleYoVariable filteredDesiredCoMYawAlphaBreakFrequency = new DoubleYoVariable("filteredDesiredCoMYawAlphaBreakFrequency", registry);
   private final DoubleYoVariable filteredDesiredCoMYawAlpha = new DoubleYoVariable("filteredDesiredCoMYawAlpha", registry);

   private final DoubleYoVariable filteredDesiredCoMPitchAlphaBreakFrequency = new DoubleYoVariable("filteredDesiredCoMPitchAlphaBreakFrequency", registry);
   private final DoubleYoVariable filteredDesiredCoMPitchAlpha = new DoubleYoVariable("filteredDesiredCoMOrientationAlpha", registry);

   private final DoubleYoVariable filteredDesiredCoMRollAlphaBreakFrequency = new DoubleYoVariable("filteredDesiredCoMRollAlphaBreakFrequency", registry);
   private final DoubleYoVariable filteredDesiredCoMRollAlpha = new DoubleYoVariable("filteredDesiredCoMRollAlpha", registry);

   private final DoubleYoVariable desiredCoMHeight = new DoubleYoVariable("desiredCoMHeight", registry);
   private final DoubleYoVariable filteredDesiredCoMHeightAlphaBreakFrequency = new DoubleYoVariable("filteredDesiredCoMHeightAlphaBreakFrequency", registry);
   private final DoubleYoVariable filteredDesiredCoMHeightAlpha = new DoubleYoVariable("filteredDesiredCoMHeightAlpha", registry);
   private final AlphaFilteredYoVariable filteredDesiredCoMHeight = new AlphaFilteredYoVariable("filteredDesiredCoMHeight", registry, filteredDesiredCoMHeightAlpha , desiredCoMHeight );

   private final YoFrameOrientation desiredCoMOrientation = new YoFrameOrientation("desiredCoMOrientation", ReferenceFrame.getWorldFrame(), registry);
   private final AlphaFilteredWrappingYoVariable filteredDesiredCoMYaw = new AlphaFilteredWrappingYoVariable("filteredDesiredCoMYaw", "", registry, desiredCoMOrientation.getYaw(), filteredDesiredCoMYawAlpha, -Math.PI, Math.PI);
   private final AlphaFilteredWrappingYoVariable filteredDesiredCoMPitch = new AlphaFilteredWrappingYoVariable("filteredDesiredCoMPitch", "", registry, desiredCoMOrientation.getPitch(), filteredDesiredCoMPitchAlpha, -Math.PI, Math.PI);
   private final AlphaFilteredWrappingYoVariable filteredDesiredCoMRoll = new AlphaFilteredWrappingYoVariable("filteredDesiredCoMRoll", "", registry, desiredCoMOrientation.getRoll(), filteredDesiredCoMRollAlpha, -Math.PI, Math.PI);
   private final DoubleYoVariable desiredCoMYaw = new DoubleYoVariable("desiredCoMYaw", registry);
   private final DoubleYoVariable desiredCoMPitch = new DoubleYoVariable("desiredCoMPitch", registry);
   private final DoubleYoVariable desiredCoMRoll = new DoubleYoVariable("desiredCoMRoll", registry);
   private final DoubleYoVariable actualYaw = new DoubleYoVariable("actualYaw", registry);
   private final DoubleYoVariable actualPitch = new DoubleYoVariable("actualPitch", registry);
   private final DoubleYoVariable actualRoll = new DoubleYoVariable("actualRoll", registry);
   private final PIDController pitchPidController = new PIDController("pitchPidController", registry);
   private final PIDController rollPidController = new PIDController("rollPidController", registry);
//   private final YoFrameOrientation filteredDesiredCoMOrientation = new YoFrameOrientation(filteredDesiredCoMYaw, filteredDesiredCoMPitch, filteredDesiredCoMRoll, ReferenceFrame.getWorldFrame());
   private final YoFrameOrientation filteredDesiredCoMOrientation = new YoFrameOrientation(desiredCoMYaw, desiredCoMPitch, desiredCoMRoll, ReferenceFrame.getWorldFrame());
   private final YoFramePose desiredCoMPose = new YoFramePose(desiredCoMPosition, filteredDesiredCoMOrientation);
   private final BooleanYoVariable useImuFeedback = new BooleanYoVariable("useImuFeedback", registry);

   private final EnumYoVariable<RobotQuadrant> swingLeg = new EnumYoVariable<RobotQuadrant>("swingLeg", registry, RobotQuadrant.class, true);
   private final YoFrameVector desiredVelocity;
   private final YoFrameVector lastDesiredVelocity;
   private final FrameVector desiredBodyVelocity = new FrameVector();
   private final DoubleYoVariable maxYawRate = new DoubleYoVariable("maxYawRate", registry);
   private final DoubleYoVariable minYawRate = new DoubleYoVariable("minYawRate", registry);
   private final DoubleYoVariable desiredYawRate = new DoubleYoVariable("desiredYawRate", registry);
   private final DoubleYoVariable lastDesiredYawRate = new DoubleYoVariable("lastDesiredYawRate", registry);

   private final DoubleYoVariable shrunkenPolygonSize = new DoubleYoVariable("shrunkenPolygonSize", registry);

   private final DoubleYoVariable nominalYaw = new DoubleYoVariable("nominalYaw", registry);

   private final DoubleYoVariable desiredYawInPlace = new DoubleYoVariable("desiredYawInPlace", registry);
   private final DoubleYoVariable desiredYawInPlaceRateLimit = new DoubleYoVariable("desiredYawInPlaceRateLimit", registry);
   private final RateLimitedYoVariable desiredYawInPlaceRateLimited;


   private final YoFrameLineSegment2d nominalYawLineSegment = new YoFrameLineSegment2d("nominalYawLineSegment", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoArtifactLineSegment2d nominalYawArtifact = new YoArtifactLineSegment2d("nominalYawArtifact", nominalYawLineSegment, Color.YELLOW, 0.02, 0.02);
   private final FramePoint2d endPoint2d = new FramePoint2d();
   private final FramePoint centroidFramePoint = new FramePoint();
   private final FramePoint2d centroidFramePoint2d = new FramePoint2d();

   private final QuadrupedSupportPolygon safeToStepSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon currentSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon commonSupportPolygon = new QuadrupedSupportPolygon();
   private final RecyclingQuadrantDependentList<FramePoint> tempSupportPolygonFramePointHolder = new RecyclingQuadrantDependentList<FramePoint>(FramePoint.class);
   private final QuadrupedSupportPolygon tempCommonShrunkenPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon tempPolygonForCommonShrunken = new QuadrupedSupportPolygon();

   private final QuadrantDependentList<QuadrupedSwingTrajectoryGenerator> swingTrajectoryGenerators = new QuadrantDependentList<>();
   private final DoubleYoVariable swingDuration = new DoubleYoVariable("swingDuration", registry);
   private final DoubleYoVariable swingHeight = new DoubleYoVariable("swingHeight", registry);
   private final DoubleYoVariable swingTimeRemaining = new DoubleYoVariable("swingTimeRemaining", registry);

   private final DoubleYoVariable distanceInside = new DoubleYoVariable("distanceInside", registry);

   private final QuadrantDependentList<ReferenceFrame> legAttachmentFrames = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint> actualFeetLocations = new QuadrantDependentList<YoFramePoint>();
   private final QuadrantDependentList<YoFramePoint> desiredFeetLocations = new QuadrantDependentList<YoFramePoint>();
   private final FramePoint desiredFootPosition = new FramePoint();
   private final FramePoint desiredFootPositionInBody = new FramePoint();

   private final QuadrantDependentList<YoFrameVector> desiredFeetPositionsInLegAttachmentFrame = new QuadrantDependentList<YoFrameVector>();
   private final QuadrantDependentList<YoFrameVector> actualFeetPositionsInLegAttachmentFrame = new QuadrantDependentList<YoFrameVector>();
   private final Vector3D desiredFootPositionForInverseKinematics = new Vector3D();

   private final YoFrameConvexPolygon2d supportPolygon = new YoFrameConvexPolygon2d("quadPolygon", "", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d currentTriplePolygon = new YoFrameConvexPolygon2d("currentTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameConvexPolygon2d upcomingTriplePolygon = new YoFrameConvexPolygon2d("upcomingTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameConvexPolygon2d commonTriplePolygon = new YoFrameConvexPolygon2d("commonTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);

   private final YoFrameConvexPolygon2d commonTriplePolygonLeft = new YoFrameConvexPolygon2d("commonTriplePolygonLeft", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameConvexPolygon2d commonTriplePolygonRight = new YoFrameConvexPolygon2d("commonTriplePolygonRight", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final SideDependentList<YoFrameConvexPolygon2d> commonTriplePolygons = new SideDependentList<>(commonTriplePolygonLeft, commonTriplePolygonRight);
//   private final YoFrameConvexPolygon2d[] tripleSupportPolygons = new YoFrameConvexPolygon2d[6];
//   private final YoArtifactPolygon[] tripleSupportArtifactPolygons = new YoArtifactPolygon[6];

   private final QuadrantDependentList<YoFrameConvexPolygon2d> tripleSupportPolygons = new QuadrantDependentList<YoFrameConvexPolygon2d>();
   private final QuadrantDependentList<YoArtifactPolygon> tripleSupportArtifactPolygons = new QuadrantDependentList<YoArtifactPolygon>();

   private final YoFramePoint circleCenter = new YoFramePoint("circleCenter", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint2d circleCenter2d = new FramePoint2d();
   private final FramePoint circleCenter3d = new FramePoint();
   private final YoGraphicPosition circleCenterGraphic = new YoGraphicPosition("circleCenterGraphic", circleCenter, 0.005, YoAppearance.Green());

   private final DoubleYoVariable inscribedCircleRadius = new DoubleYoVariable("inscribedCircleRadius", registry);
   private final YoArtifactOval inscribedCircle = new YoArtifactOval("inscribedCircle", circleCenter, inscribedCircleRadius, Color.BLACK);

   private final BooleanYoVariable useSubCircleForBodyShiftTarget = new BooleanYoVariable("useSubCircleForBodyShiftTarget", registry);
   private final DoubleYoVariable subCircleRadius = new DoubleYoVariable("subCircleRadius", registry);
   private final DoubleYoVariable comCloseRadius = new DoubleYoVariable("comCloseRadius", "Distance check from final desired circle to CoM for transitioning into swing state", registry);

   private final YoFrameVector yoVectorToSubtract = new YoFrameVector("yoVectorToSubtract", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint centerOfMassInBody = new FramePoint();
   private final FramePoint desiredRootJointPosition = new FramePoint();
   private final FrameVector vectorToSubtractHolder = new FrameVector();
   private final Vector3D linearVelocityHolder = new Vector3D();

   private final BooleanYoVariable isCoMInsideTriangleForSwingLeg = new BooleanYoVariable("isCoMInsideTriangleForSwingLeg", registry);
   private final BooleanYoVariable isCoMCloseToFinalDesired = new BooleanYoVariable("isCoMCloseToFinalDesired", registry);
   private final BooleanYoVariable useCommonTriangleForSwingTransition = new BooleanYoVariable("useCommonTriangleForSwingTransition", registry);

   private final YoFrameVector feedForwardCenterOfMassVelocity = new YoFrameVector("feedForwardCenterOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint centerOfMassPosition = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector centerOfMassVelocity = new YoFrameVector("centerOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final FramePoint centerOfMassFramePoint = new FramePoint();
   private final FramePoint2d centerOfMassPoint2d = new FramePoint2d();
   private final YoGraphicPosition centerOfMassViz = new YoGraphicPosition("centerOfMass", centerOfMassPosition, 0.02, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);

   private final YoFramePoint currentSwingTarget = new YoFramePoint("currentSwingTarget", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint finalSwingTarget = new YoFramePoint("finalSwingTarget", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition currentSwingTargetViz = new YoGraphicPosition("currentSwingTarget", currentSwingTarget, 0.01, YoAppearance.Red());
   private final YoGraphicPosition finalSwingTargetViz = new YoGraphicPosition("finalSwingTarget", finalSwingTarget, 0.01, YoAppearance.Purple());

   private final YoFramePoint desiredCoMTarget = new YoFramePoint("desiredCoMTarget", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition desiredCoMTargetViz = new YoGraphicPosition("desiredCoMTargetViz", desiredCoMTarget, 0.01, YoAppearance.Turquoise());

   private final YoFramePoint desiredCoM = new YoFramePoint("desiredCoM", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition desiredCoMViz = new YoGraphicPosition("desiredCoMViz", desiredCoM, 0.01, YoAppearance.HotPink());

   private final YoFramePoint currentICP = new YoFramePoint("currentICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition currentICPViz = new YoGraphicPosition("currentICPViz", currentICP, 0.01, YoAppearance.DarkSlateBlue());

   private final YoFramePoint feedForwardICP = new YoFramePoint("feedForwardICP", ReferenceFrame.getWorldFrame(), registry);
   private final YoGraphicPosition feedForwardICPViz = new YoGraphicPosition("feedForwardICPViz", feedForwardICP, 0.01, YoAppearance.DarkSlateBlue());

   private final YoGraphicReferenceFrame desiredCoMPoseYoGraphic = new YoGraphicReferenceFrame(desiredCoMPoseReferenceFrame, registry, 0.45);
   private final YoGraphicReferenceFrame comPoseYoGraphic, feedForwardCoMPoseYoGraphic;

   private final YoGraphicReferenceFrame centroidWithNominal;
   private final YoGraphicReferenceFrame centroidZUpWithNominal;
   private final QuadrantDependentList<YoGraphicReferenceFrame> tripleSupportFrames = new QuadrantDependentList<>();

   public final BooleanYoVariable isVelocityNegative = new BooleanYoVariable("isVelocityNegative", registry);
   public final DoubleYoVariable velocitySign = new DoubleYoVariable("velocitySign", registry);

   private final QuadrantDependentList<YoGraphicReferenceFrame> desiredAttachmentFrames = new QuadrantDependentList<YoGraphicReferenceFrame>();
   private final QuadrantDependentList<YoGraphicReferenceFrame> actualAttachmentFrames = new QuadrantDependentList<YoGraphicReferenceFrame>();

   private final DoubleYoVariable timeToFilterDesiredAtCrawlStart = new DoubleYoVariable("timeToFilterDesiredAtCrawlStart", registry);

   /** body sway trajectory **/
   private final BooleanYoVariable comTrajectoryGeneratorRequiresReInitailization = new BooleanYoVariable("comTrajectoryGeneratorRequiresReInitailization", registry);
   private final VelocityConstrainedPositionTrajectoryGenerator comTrajectoryGenerator = new VelocityConstrainedPositionTrajectoryGenerator("comTraj", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint initialCoMPosition = new YoFramePoint("initialCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector initialCoMVelocity = new YoFrameVector("initialCoMVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable comTrajectoryTimeStart = new DoubleYoVariable("comTrajectoryTimeStart", registry);
   private final DoubleYoVariable comTrajectoryTimeCurrent = new DoubleYoVariable("comTrajectoryTimeCurrent", registry);
   private final DoubleYoVariable comTrajectoryTimeDesired = new DoubleYoVariable("comTrajectoryTimeDesired", registry);
   private final DoubleYoVariable comTrajectoryTimeLastCalled = new DoubleYoVariable("comTrajectoryTimeLastCalled", registry);
   private final DoubleYoVariable comTrajectoryTimeScaleFactor = new DoubleYoVariable("comTrajectoryTimeScaleFactor", registry);
   private final DoubleYoVariable distanceToTrotLine = new DoubleYoVariable("distanceToTrotLine", registry);
   private final DoubleYoVariable distanceTravelAtEndOfSwing = new DoubleYoVariable("distanceTravelAtEndOfSwing", registry);
   private final BooleanYoVariable slowBodyDown = new BooleanYoVariable("slowBodyDown", registry);
   private final DoubleYoVariable timeToSlowTo = new DoubleYoVariable("timeToSlowTo", registry);



   private final DoubleYoVariable turnInPlaceCoMTrajectoryBuffer = new DoubleYoVariable("turnInPlaceCoMTrajectoryBuffer", registry);
   private final DoubleYoVariable comTrajectoryDuration = new DoubleYoVariable("comTrajectoryDuration", registry);
   private final DoubleYoVariable maximumCoMTrajectoryDuration = new DoubleYoVariable("maximumCoMTrajectoryDuration", registry);
   private final DoubleYoVariable minimumCoMTrajectoryDuration = new DoubleYoVariable("minimumCoMTrajectoryDuration", registry);

   private final QuadrupedPostureInputProviderInterface postureProvider;
   private final QuadrupedPlanarVelocityInputProvider planarVelocityProvider;

   private final TwistCalculator twistCalculator;
   private final Twist bodyTwist = new Twist();

   public QuadrupedPositionBasedCrawlController(QuadrupedRuntimeEnvironment environment, QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties, QuadrupedPositionBasedCrawlControllerParameters crawlControllerParameters, QuadrupedPostureInputProvider postureProvider, QuadrupedPlanarVelocityInputProvider planarVelocityProvider, QuadrupedLegInverseKinematicsCalculator legIkCalculator)
   {
      this(environment.getControlDT(), modelFactory, physicalProperties, crawlControllerParameters, environment.getFullRobotModel(), postureProvider, planarVelocityProvider, environment.getFootSwitches(), legIkCalculator, environment.getGlobalDataProducer(), environment.getRobotTimestamp(),
            environment.getParentRegistry(), environment.getGraphicsListRegistry(), environment.getGraphicsListRegistryForDetachedOverhead());
   }

   public QuadrupedPositionBasedCrawlController(final double dt, QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties, QuadrupedPositionBasedCrawlControllerParameters quadrupedControllerParameters, FullQuadrupedRobotModel fullRobotModel,
         QuadrupedPostureInputProviderInterface postureProvider, QuadrupedPlanarVelocityInputProvider planarVelocityProvider, QuadrantDependentList<FootSwitchInterface> footSwitches, QuadrupedLegInverseKinematicsCalculator quadrupedInverseKinematicsCalulcator, final GlobalDataProducer dataProducer, DoubleYoVariable yoTime,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead)
   {

      swingDuration.set(quadrupedControllerParameters.getDefaultSwingDuration());
      swingHeight.set(quadrupedControllerParameters.getDefaultSwingHeight());
      subCircleRadius.set(quadrupedControllerParameters.getDefaultSubCircleRadius());
      comCloseRadius.set(quadrupedControllerParameters.getDefaultCoMCloseToFinalDesiredTransitionRadius());
      minYawRate.set(quadrupedControllerParameters.getMaxYawRate() * -1.0);
      maxYawRate.set(quadrupedControllerParameters.getMaxYawRate());
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

      this.postureProvider = postureProvider;
      this.planarVelocityProvider = planarVelocityProvider;
      this.robotTimestamp = yoTime;
      this.dt = dt;
      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, physicalProperties);
      this.actualFullRobotModel = fullRobotModel;
      this.centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      this.walkingStateMachine = new StateMachine<CrawlGateWalkingState>("QuadrupedCrawlState", "walkingStateTranistionTime", CrawlGateWalkingState.class, yoTime, registry);
      this.inverseKinematicsCalculators = quadrupedInverseKinematicsCalulcator;

      actualRobotRootJoint = actualFullRobotModel.getRootJoint();
      twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), actualFullRobotModel.getElevator());
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
      oneDoFJointsActual = fullRobotModel.getOneDoFJoints();

      desiredCoMOffset = new YoFramePoint2d("desiredCoMOffset", feedForwardReferenceFrames.getBodyZUpFrame(), registry);
      desiredCoMOffset.set(quadrupedControllerParameters.getDefaultDesiredCoMOffset());

      updateFeedForwardModelAndFrames();

      for (RobotQuadrant robotQuadrant: RobotQuadrant.values)
      {
    	  YoGraphicReferenceFrame desiredAttachmentFrame = new YoGraphicReferenceFrame("ffLegAttachment", feedForwardReferenceFrames.getLegAttachmentFrame(robotQuadrant), registry, 0.25, YoAppearance.Purple());
    	  desiredAttachmentFrames.set(robotQuadrant, desiredAttachmentFrame);

    	  YoGraphicReferenceFrame actualAttachmentFrame = new YoGraphicReferenceFrame("legAttachment", referenceFrames.getLegAttachmentFrame(robotQuadrant), registry, 0.25, YoAppearance.Green());
    	  actualAttachmentFrames.set(robotQuadrant, actualAttachmentFrame);

          yoGraphicsListRegistry.registerYoGraphic("AttachementFrames", desiredAttachmentFrame);
          yoGraphicsListRegistry.registerYoGraphic("AttachementFrames", actualAttachmentFrame);
      }

      this.swingTargetGenerator = new MidFootZUpSwingTargetGenerator(quadrupedControllerParameters, feedForwardReferenceFrames, registry);
      this.footSwitches = footSwitches;

      desiredVelocity = new YoFrameVector("desiredVelocity", feedForwardBodyFrame, registry);
      lastDesiredVelocity = new YoFrameVector("lastDesiredVelocity", feedForwardBodyFrame, registry);

      desiredVelocity.setX(0.0);
      comTrajectoryTimeDesired.set(1.0);
      maximumCoMTrajectoryDuration.set(6.0);
      minimumCoMTrajectoryDuration.set(0.01);
      comTrajectoryTimeScaleFactor.set(1.0);

      shrunkenPolygonSize.set(0.02);

      timeToFilterDesiredAtCrawlStart.set(4.0);

      comPoseYoGraphic = new YoGraphicReferenceFrame("rasta_", comFrame, registry, 0.25, YoAppearance.Green());
      feedForwardCoMPoseYoGraphic = new YoGraphicReferenceFrame("feedForwardRasta_", feedForwardCenterOfMassFrame, registry, 0.25, YoAppearance.Purple());

      feedForwardCenterOfMassOffsetAlpha = new DoubleYoVariable("feedForwardCenterOfMassOffsetAlpha", registry);
      feedForwardCenterOfMassOffsetAlpha.set(AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(1.0, dt));
      feedForwardCenterOfMassOffset = new YoFramePoint("feedForwardCenterOfMassOffset", feedForwardCenterOfMassFrame, registry);
      filteredFeedForwardCenterOfMassOffset = AlphaFilteredYoFramePoint.createAlphaFilteredYoFramePoint("filteredFeedForwardCenterOfMassOffset", "", registry, feedForwardCenterOfMassOffsetAlpha, feedForwardCenterOfMassOffset);


      centroidWithNominal = new YoGraphicReferenceFrame(referenceFrames.getCenterOfFeetFrameAveragingLowestZHeightsAcrossEnds(), registry, 0.1);
      centroidZUpWithNominal = new YoGraphicReferenceFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds(), registry, 0.1);

      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         tripleSupportFrames.set(robotQuadrant, new YoGraphicReferenceFrame(referenceFrames.getTripleSupportFrameAveragingLowestZHeightsAcrossEnds(robotQuadrant), registry, 0.1));
      }

      filteredDesiredCoMYawAlphaBreakFrequency.set(DEFAULT_HEADING_CORRECTION_BREAK_FREQUENCY);
      filteredDesiredCoMYawAlpha.set(
            AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMYawAlphaBreakFrequency.getDoubleValue(), dt));
      filteredDesiredCoMYawAlphaBreakFrequency.addVariableChangedListener(
            createBreakFrequencyChangeListener(dt, filteredDesiredCoMYawAlphaBreakFrequency, filteredDesiredCoMYawAlpha));

      filteredDesiredCoMPitchAlphaBreakFrequency.set(DEFAULT_COM_PITCH_FILTER_BREAK_FREQUENCY);
      filteredDesiredCoMPitchAlpha.set(
            AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMPitchAlphaBreakFrequency.getDoubleValue(), dt));
      filteredDesiredCoMPitchAlphaBreakFrequency.addVariableChangedListener(
            createBreakFrequencyChangeListener(dt, filteredDesiredCoMPitchAlphaBreakFrequency, filteredDesiredCoMPitchAlpha));

      filteredDesiredCoMRollAlphaBreakFrequency.set(DEFAULT_COM_ROLL_FILTER_BREAK_FREQUENCY);
      filteredDesiredCoMRollAlpha.set(
            AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMRollAlphaBreakFrequency.getDoubleValue(), dt));
      filteredDesiredCoMRollAlphaBreakFrequency.addVariableChangedListener(
            createBreakFrequencyChangeListener(dt, filteredDesiredCoMRollAlphaBreakFrequency, filteredDesiredCoMRollAlpha));


      desiredYawInPlaceRateLimited  = new RateLimitedYoVariable("desiredYawInPlaceRateLimited", registry, desiredYawInPlaceRateLimit, dt);
      desiredYawInPlaceRateLimit.set(DEFAULT_YAW_IN_PLACE_RATE_LIMIT);


      filteredDesiredCoMHeightAlphaBreakFrequency.set(DEFAULT_COM_HEIGHT_Z_FILTER_BREAK_FREQUENCY);
      filteredDesiredCoMHeightAlpha.set(
            AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMHeightAlphaBreakFrequency.getDoubleValue(), dt));
      filteredDesiredCoMHeightAlphaBreakFrequency.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            filteredDesiredCoMHeightAlpha.set(
                  AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(filteredDesiredCoMHeightAlphaBreakFrequency.getDoubleValue(), dt));

         }
      });

      referenceFrames.updateFrames();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         swingTrajectoryGenerators.set(robotQuadrant, new QuadrupedSwingTrajectoryGenerator(robotQuadrant, registry, yoGraphicsListRegistry, dt));

         ReferenceFrame footReferenceFrame = referenceFrames.getFootFrame(robotQuadrant);
         ReferenceFrame legAttachmentFrame = referenceFrames.getLegAttachmentFrame(robotQuadrant);

         legAttachmentFrames.set(robotQuadrant, legAttachmentFrame);

         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();

         YoFramePoint actualFootPosition = new YoFramePoint(prefix + "actualFootPosition", ReferenceFrame.getWorldFrame(), registry);
         actualFeetLocations.set(robotQuadrant, actualFootPosition);

         YoFramePoint desiredFootLocation = new YoFramePoint(prefix + "FootDesiredPosition", ReferenceFrame.getWorldFrame(), registry);

         FramePoint footPosition = new FramePoint(footReferenceFrame);
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         footPosition.setZ(0.0);
         desiredFootLocation.set(footPosition);
         desiredFeetLocations.set(robotQuadrant, desiredFootLocation);

         YoFrameVector footPositionInLegAttachementFrame = new YoFrameVector(prefix + "FootPositionInLegFrame", referenceFrames.getLegAttachmentFrame(robotQuadrant), registry);
         desiredFeetPositionsInLegAttachmentFrame.set(robotQuadrant, footPositionInLegAttachementFrame);

         YoFrameVector actualFootPositionInLegAttachementFrame = new YoFrameVector(prefix + "ActualFootPositionInLegFrame", referenceFrames.getLegAttachmentFrame(robotQuadrant), registry);
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
         YoFrameConvexPolygon2d yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d(polygonName, "", ReferenceFrame.getWorldFrame(), 3, registry);
         tripleSupportPolygons.set(robotQuadrant, yoFrameConvexPolygon2d);

         float saturation = 0.5f;
         float brightness = 0.5f;
         float hue = (float) (0.1 * robotQuadrant.ordinal());

         YoArtifactPolygon yoArtifactPolygon = new YoArtifactPolygon(polygonName, yoFrameConvexPolygon2d, Color.getHSBColor(hue, saturation, brightness), false);
         tripleSupportArtifactPolygons.set(robotQuadrant, yoArtifactPolygon);
         yoGraphicsListRegistry.registerArtifact(polygonName, yoArtifactPolygon);
         yoGraphicsListRegistryForDetachedOverhead.registerArtifact(polygonName, yoArtifactPolygon);
      }

      createGraphicsAndArtifacts(yoGraphicsListRegistry, yoGraphicsListRegistryForDetachedOverhead);

      referenceFrames.updateFrames();
      updateFeetLocations();

      FramePose centerOfMassPose = new FramePose(referenceFrames.getCenterOfFourHipsFrame());
      centerOfMassPose.changeFrame(ReferenceFrame.getWorldFrame());
      desiredCoMHeight.set(quadrupedControllerParameters.getInitalCoMHeight());
      filteredDesiredCoMHeight.update();
      centerOfMassPose.setZ(filteredDesiredCoMHeight.getDoubleValue());
      desiredCoMPose.set(centerOfMassPose);
      desiredCoM.set(centerOfMassPose.getFramePointCopy());
      desiredCoMPoseReferenceFrame.setPoseAndUpdate(centerOfMassPose);
      updateFeedForwardModelAndFrames();

      quadrupleSupportState = new QuadrupleSupportState(CrawlGateWalkingState.QUADRUPLE_SUPPORT, DEFAULT_TIME_TO_STAY_IN_DOUBLE_SUPPORT, 0.2);
      TripleSupportState tripleSupportState = new TripleSupportState(CrawlGateWalkingState.TRIPLE_SUPPORT);

      filterDesiredsToMatchCrawlControllerOnTransitionIn = new FilterDesiredsToMatchCrawlControllerState(CrawlGateWalkingState.ALPHA_FILTERING_DESIREDS, modelFactory);

      walkingStateMachine.addState(filterDesiredsToMatchCrawlControllerOnTransitionIn);
      walkingStateMachine.addState(quadrupleSupportState);
      walkingStateMachine.addState(tripleSupportState);
      walkingStateMachine.setCurrentState(CrawlGateWalkingState.ALPHA_FILTERING_DESIREDS);

      StateTransitionCondition quadrupleToTripleCondition = new QuadrupleToTripleCondition(quadrupleSupportState);
      StateTransitionCondition tripleToQuadrupleCondition = new TripleToQuadrupleCondition(tripleSupportState);
      StateTransitionCondition filterToQuadrupleCondition = new FilterToQuadrupleCondition(filterDesiredsToMatchCrawlControllerOnTransitionIn);

      filterDesiredsToMatchCrawlControllerOnTransitionIn.addStateTransition(new StateTransition<CrawlGateWalkingState>(CrawlGateWalkingState.QUADRUPLE_SUPPORT, filterToQuadrupleCondition));
      quadrupleSupportState.addStateTransition(new StateTransition<CrawlGateWalkingState>(CrawlGateWalkingState.TRIPLE_SUPPORT, quadrupleToTripleCondition));
      tripleSupportState.addStateTransition(new StateTransition<CrawlGateWalkingState>(CrawlGateWalkingState.QUADRUPLE_SUPPORT, tripleToQuadrupleCondition));

      BooleanYoVariable applyJoystickInput = new BooleanYoVariable("applyJoystickInput", registry);
      applyJoystickInput.set(true);

      parentRegistry.addChild(registry);
   }


   private VariableChangedListener createBreakFrequencyChangeListener(final double dt, final DoubleYoVariable breakFrequency, final DoubleYoVariable alpha)
   {
      return new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
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
      YoArtifactPolygon commonTriplePolygonRightArtifact = new YoArtifactPolygon("commonTriplePolygonRightArtifact", commonTriplePolygonRight, Color.MAGENTA, false);

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

         YoFramePoint footPosition = actualFeetLocations.get(robotQuadrant);
         YoGraphicPosition actualFootPositionViz = new YoGraphicPosition(prefix + "actualFootPositionViz", footPosition, 0.02,
               getYoAppearance(robotQuadrant), GraphicType.BALL_WITH_CROSS);

         yoGraphicsListRegistry.registerYoGraphic("actualFootPosition", actualFootPositionViz);
         yoGraphicsListRegistry.registerArtifact("actualFootPosition", actualFootPositionViz.createArtifact());
         yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("actualFootPosition", actualFootPositionViz);
         yoGraphicsListRegistryForDetachedOverhead.registerArtifact("actualFootPosition", actualFootPositionViz.createArtifact());

         YoFramePoint desiredFootPosition = desiredFeetLocations.get(robotQuadrant);
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
   public ControllerEvent process()
   {
      doActionTimer.startTimer();

      referenceFrames.updateFrames();
      updateEstimates();
      updateGraphics();
      pollDataProviders();
      checkForReversedVelocity();
      walkingStateMachine.checkTransitionConditions();
      walkingStateMachine.doAction();
      updateDesiredCoMTrajectory();
      updateDesiredHeight();
      updateDesiredYaw();
      updateDesiredBodyOrientation();
      updateDesiredCoMPose();
      updateLegsBasedOnDesiredCoM();
      computeDesiredPositionsAndStoreInFullRobotModel(actualFullRobotModel);

      if(walkingStateMachine.isCurrentState(CrawlGateWalkingState.ALPHA_FILTERING_DESIREDS))
      {
         filterDesiredsToMatchCrawlControllerOnTransitionIn.filterDesireds(oneDoFJointsActual);
      }

      updateFeedForwardModelAndFrames();

      doActionTimer.stopTimer();
      return null;
   }

   private void checkForReversedVelocity()
   {
      if(desiredVelocity.getX() < 0 && lastDesiredVelocity.getX() >= 0)
      {
         isVelocityNegative.set(true);
         velocitySign.set(-1.0);
      }
      else if(desiredVelocity.getX() >= 0 && lastDesiredVelocity.getX() < 0)
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
      for (int i=0; i<oneDoFJointsActual.length; i++)
      {
         OneDoFJoint oneDoFJoint = oneDoFJointsActual[i];
         OneDoFJoint oneDoFJointFeedforward = oneDoFJointsFeedforward[i];

         oneDoFJointFeedforward.setQ(oneDoFJoint.getqDesired());
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
         ReferenceFrame footFrame = feedForwardReferenceFrames.getFootFrame(robotQuadrant);
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
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotQuadrant);
         actualFootLocation.setToZero(footFrame);
         actualFootLocation.changeFrame(ReferenceFrame.getWorldFrame());

         YoFramePoint yoActualFootLocation = actualFeetLocations.get(robotQuadrant);
         yoActualFootLocation.set(actualFootLocation);

         YoFramePoint yoDesiredFootLocation = desiredFeetLocations.get(robotQuadrant);
         yoDesiredFootLocation.set(actualFootLocation);

         fourFootSupportPolygon.setFootstep(robotQuadrant, actualFootLocation);
      }
   }

   private void updateFeedForwardModelAndFrames()
   {
	   for (int i=0; i<oneDoFJointsActual.length; i++)
	   {
		   OneDoFJoint oneDoFJoint = oneDoFJointsActual[i];
		   OneDoFJoint oneDoFJointFeedforward = oneDoFJointsFeedforward[i];

//		   oneDoFJointFeedforward.setQ(oneDoFJoint.getQ());
		   double qd = oneDoFJoint.getqDesired() - oneDoFJointFeedforward.getQ();
		   oneDoFJointFeedforward.setQd(qd * 1.0 / dt);
		   oneDoFJointFeedforward.setQ(oneDoFJoint.getqDesired());
	   }

	   FloatingInverseDynamicsJoint feedForwardRootJoint = feedForwardFullRobotModel.getRootJoint();

	   feedForwardRootJoint.setRotation(filteredDesiredCoMOrientation.getYaw().getDoubleValue(), filteredDesiredCoMOrientation.getPitch().getDoubleValue(), filteredDesiredCoMOrientation.getRoll().getDoubleValue());
	   feedForwardFullRobotModel.updateFrames();

//	   Vector3d rootJointPosition = new Vector3d();
//	   rootJoint.packTranslation(rootJointPosition);
//	   feedForwardRootJoint.setPosition(rootJointPosition);

	   centerOfMassInBody.setIncludingFrame(feedForwardCenterOfMassFrame, 0.0, 0.0, 0.0);
	   centerOfMassInBody.changeFrame(feedForwardRootJoint.getFrameAfterJoint());

	   vectorToSubtractHolder.setIncludingFrame(feedForwardFullRobotModel.getRootJoint().getFrameAfterJoint(), centerOfMassInBody.getPoint());
	   vectorToSubtractHolder.changeFrame(ReferenceFrame.getWorldFrame());

	   yoVectorToSubtract.set(vectorToSubtractHolder);
//	   System.out.println("VectorToSubtract = " + vectorToSubtract);

	   desiredRootJointPosition.setIncludingFrame(desiredCoM.getFrameTuple());
	   desiredRootJointPosition.sub(vectorToSubtractHolder);
      feedForwardRootJoint.getTranslation(linearVelocityHolder);
	   feedForwardRootJoint.setPosition(desiredRootJointPosition.getPoint());
	   linearVelocityHolder.sub(desiredRootJointPosition.getPoint(), linearVelocityHolder);
//	   feedForwardRootJoint.setLinearVelocityInWorld(linearVelocityHolder);

//	   feedForwardFullRobotModel.updateFrames();
	   feedForwardReferenceFrames.updateFrames();
   }

   private double lastProvidedDesiredYawRate = 0.0;
   private final FrameVector providedDesiredVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   private void pollDataProviders()
   {

      if(postureProvider != null)
      {
         postureProvider.getComVelocityInput();// support z up

         //body velocity
         Vector3D planarVelocityInput = planarVelocityProvider.get();//Frame up to controller - feedForwardBodyFrame
         desiredVelocity.setVector(planarVelocityInput);
         desiredVelocity.setY(0.0);
         desiredVelocity.setZ(0.0);

         //yaw rate
         double providedDesiredYawRate = planarVelocityInput.getZ();
         providedDesiredYawRate = MathTools.clipToMinMax(providedDesiredYawRate, minYawRate.getDoubleValue(), maxYawRate.getDoubleValue());
         if (providedDesiredYawRate != lastProvidedDesiredYawRate)
         {
            desiredYawRate.set(providedDesiredYawRate);
            lastProvidedDesiredYawRate = providedDesiredYawRate;
         }

         //body orientation
         Quaternion bodyOrientationInput = postureProvider.getBodyOrientationInput();// support z up
//            double providedDesiredYawInPlace = desiredYawInPlaceProvider.getValue();
         double providedDesiredYawInPlace = bodyOrientationInput.getYaw();
         providedDesiredYawInPlace = MathTools.clipToMinMax(providedDesiredYawInPlace, -MAX_YAW_IN_PLACE, MAX_YAW_IN_PLACE);
         if (isDesiredVelocityAndYawRateZero())
         {
            desiredYawInPlace.set(providedDesiredYawInPlace);
         }

         desiredCoMOrientation.setPitch(bodyOrientationInput.getPitch());
         desiredCoMOrientation.setRoll(bodyOrientationInput.getRoll());

         //com height
         Point3D comPositionInput = postureProvider.getComPositionInput();// support z up
         desiredCoMHeight.set(comPositionInput.getZ());
      }
   }

   private final double[] yawPitchRollArray = new double[3];
   private final Point3D centerOfMassOffset = new Point3D();
   private final FramePoint actualFootLocation = new FramePoint();
   private final FrameVector tempFrameVector = new FrameVector();



   /**
    * uses feedback to update the CoM Velocity, ICP, and Actual Foot Positions
    */
   private void updateEstimates()
   {
      twistCalculator.compute();
      twistCalculator.getTwistOfBody(bodyTwist, actualFullRobotModel.getPelvis());

      actualRobotRootJoint.getRotation(yawPitchRollArray);
      actualYaw.set(yawPitchRollArray[0]);
      actualPitch.set(yawPitchRollArray[1]);
      actualRoll.set(yawPitchRollArray[2]);

      filteredFeedForwardCenterOfMassOffset.update();
      filteredFeedForwardCenterOfMassOffset.get(centerOfMassOffset);
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
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotQuadrant);
         actualFootLocation.setToZero(footFrame);
         actualFootLocation.changeFrame(ReferenceFrame.getWorldFrame());

         YoFramePoint yoFootLocation = actualFeetLocations.get(robotQuadrant);
         yoFootLocation.set(actualFootLocation);

         // Use the desired foot locations instead of the actual locations
         YoFramePoint desiredFootLocation = desiredFeetLocations.get(robotQuadrant);
         fourFootSupportPolygon.setFootstep(robotQuadrant, desiredFootLocation.getFrameTuple());
      }
   }

   FramePoint tempCOMTarget = new FramePoint(ReferenceFrame.getWorldFrame());


   private double computeDistanceToTrotLine2d(RobotQuadrant swingQuadrant)
   {
      comTrajectoryGenerator.getFinalPosition(tempCOMTarget);
      FramePoint desiredBodyCurrent = desiredCoMPose.getPosition().getFrameTuple();

      //do this only if the desiredBodyCurrent != tempCOMTarget
      double distanceToTrotLine;
      if (desiredBodyCurrent.distance(tempCOMTarget) > 1e-3)
      {
         distanceToTrotLine = fourFootSupportPolygon.getDistanceFromP1ToTrotLineInDirection2d(swingQuadrant.getAcrossBodyQuadrant(), desiredBodyCurrent, tempCOMTarget);
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
      if (swingQuadrant != null) currentSupportPolygon.removeFootstep(swingQuadrant);
      centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      distanceInside.set(currentSupportPolygon.getDistanceInside2d(centerOfMassFramePoint));
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

      for (RobotQuadrant robotQuadrant: RobotQuadrant.values)
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
      if(!comTrajectoryGenerator.isDone() && !comTrajectoryGeneratorRequiresReInitailization.getBooleanValue() && (walkingStateMachine.isCurrentState(CrawlGateWalkingState.TRIPLE_SUPPORT) || !isDesiredVelocityAndYawRateZero()))
      {
//         double deltaTimeFromLastCall = robotTimestamp.getDoubleValue() - comTrajectoryTimeLastCalled.getDoubleValue();
         double deltaTimeToUse = comTrajectoryTimeScaleFactor.getDoubleValue()*dt;
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

      centroidFramePoint2d.setByProjectionOntoXYPlaneIncludingFrame(centroidFramePoint);
      endPoint2d.set(centroidFramePoint2d);
      endPoint2d.add(0.4,0.0);
      endPoint2d.yawAboutPoint(centroidFramePoint2d, endPoint2d, nominalYaw.getDoubleValue());

      nominalYawLineSegment.set(centroidFramePoint2d, endPoint2d);
      DoubleYoVariable desiredYaw = desiredCoMOrientation.getYaw();

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

      if(useImuFeedback.getBooleanValue())
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

   private FramePoint actualFootPositionInLegAttachmentFrame = new FramePoint();

   private void updateLegsBasedOnDesiredCoM()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         packFootPositionUsingDesiredBodyToBodyHack(robotQuadrant);

         actualFootPositionInLegAttachmentFrame.setIncludingFrame(referenceFrames.getFootFrame(robotQuadrant), 0.0, 0.0, 0.0);
         actualFootPositionInLegAttachmentFrame.changeFrame(referenceFrames.getLegAttachmentFrame(robotQuadrant));
         actualFeetPositionsInLegAttachmentFrame.get(robotQuadrant).set(actualFootPositionInLegAttachmentFrame);
      }
   }

   private void computeDesiredPositionsAndStoreInFullRobotModel(FullRobotModel fullRobotModel)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         desiredFeetPositionsInLegAttachmentFrame.get(robotQuadrant).get(desiredFootPositionForInverseKinematics);
         inverseKinematicsCalculators.solveForEndEffectorLocationInBodyAndUpdateDesireds(robotQuadrant, desiredFootPositionForInverseKinematics, fullRobotModel);
      }
   }

   /**
    * currently uses the difference between the actual CoM and desired CoM to move the body,
    * This should be a feedforward psuedo actual, integrated from the desired, relating to the desiredBody
    */
   private void packFootPositionUsingDesiredBodyToBodyHack(RobotQuadrant robotQuadrant)
   {
      desiredFootPosition.setIncludingFrame(desiredFeetLocations.get(robotQuadrant).getFrameTuple());
      desiredFootPosition.changeFrame(desiredCoMPoseReferenceFrame);

      // Fix this for feed forward!!!
      desiredFootPositionInBody.setIncludingFrame(feedForwardCenterOfMassFrame, desiredFootPosition.getPoint());
      desiredFootPositionInBody.changeFrame(feedForwardReferenceFrames.getLegAttachmentFrame(robotQuadrant));

      desiredFeetPositionsInLegAttachmentFrame.get(robotQuadrant).set(desiredFootPositionInBody.getPoint());
   }


   FrameVector expectedAverageVelocity = new FrameVector();
   /**
   * Calculates the next swing target in world using the actual feet,
   * should create another method for feedforward then handle moving to body frame
   */
   public void calculateSwingTarget(RobotQuadrant swingLeg, FramePoint framePointToPack)
   {
      tempComTrajComputedVelocity.setIncludingFrame(desiredCoMVelocity.getFrameTuple());
      tempDesiredVelocityVector.setIncludingFrame(desiredVelocity.getFrameTuple());

      tempComTrajComputedVelocity.changeFrame(feedForwardBodyFrame);
      tempDesiredVelocityVector.changeFrame(feedForwardBodyFrame);

      if(tempDesiredVelocityVector.length() < 1e-3)
      {
         expectedAverageVelocity.setIncludingFrame(tempDesiredVelocityVector);
      }
      else if(tempComTrajComputedVelocity.getX() > 0 && tempDesiredVelocityVector.getX() < 0 || tempComTrajComputedVelocity.getX() < 0 && tempDesiredVelocityVector.getX() > 0)
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
      swingTargetGenerator.getSwingTarget(swingLeg, feedForwardReferenceFrames.getLegAttachmentFrame(swingLeg), expectedAverageVelocity, swingDuration.getDoubleValue(), framePointToPack, yawRate);
      framePointToPack.setZ(0.0);
   }

   private void drawSupportPolygon(QuadrupedSupportPolygon supportPolygon, YoFrameConvexPolygon2d yoFramePolygon)
   {
      tempSupportPolygonFramePointHolder.clear();
      for(RobotQuadrant quadrant : supportPolygon.getSupportingQuadrantsInOrder())
      {
         tempSupportPolygonFramePointHolder.add(quadrant).setIncludingFrame(supportPolygon.getFootstep(quadrant));
      }

      yoFramePolygon.setConvexPolygon2d(tempSupportPolygonFramePointHolder.values());
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
      public boolean checkCondition()
      {
         if(!quadrupleSupportState.isMinimumTimeInQuadSupportElapsed())
         {
            return false;
         }

         if(quadrupleSupportState.isTransitioningToSafePosition())
         {
            return false;
         }

         if (swingLeg.getEnumValue().isQuadrantInHind())
         {
            if(isVelocityNegative.getBooleanValue())
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
            if(isVelocityNegative.getBooleanValue())
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

         return isCoMCloseToFinalDesired.getBooleanValue() && isCoMInsideTriangleForSwingLeg.getBooleanValue() && (desiredVelocity.length() != 0.0 || desiredYawRate.getDoubleValue() != 0); //bodyTrajectoryGenerator.isDone() &&
      }
   }

   private class TripleToQuadrupleCondition implements StateTransitionCondition
   {
      private final TripleSupportState tripleSupportState;

      public TripleToQuadrupleCondition(TripleSupportState tripleSupportState)
      {
         this.tripleSupportState = tripleSupportState;
      }
      @Override
      public boolean checkCondition()
      {
    	  RobotQuadrant swingQuadrant = swingLeg.getEnumValue();
    	  boolean swingTrajectoryIsDone = swingTrajectoryGenerators.get(swingQuadrant).isDone();
    	  boolean swingFootHitGround = false;
    	  boolean inSwingStateLongEnough = tripleSupportState.getTimeInCurrentState() > swingDuration.getDoubleValue() / 3.0;

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
      public boolean checkCondition()
      {
         return filterDesiredsToMatchCrawlControllerState.isInterpolationFinished();
      }
   }

   private class FilterDesiredsToMatchCrawlControllerState extends State<CrawlGateWalkingState>
   {
      private final FullRobotModel initialDesiredsUponEnteringFullRobotModel;

      private final MinimumJerkTrajectory minimumJerkTrajectory = new MinimumJerkTrajectory();

      public FilterDesiredsToMatchCrawlControllerState(CrawlGateWalkingState stateEnum, QuadrupedModelFactory robotParameters)
      {
         super(stateEnum);

         initialDesiredsUponEnteringFullRobotModel = robotParameters.createFullRobotModel();
      }

      public void filterDesireds(OneDoFJoint[] oneDoFJointsActual)
      {
         for(int i = 0; i < oneDoFJointsActual.length; i++)
         {
            OneDoFJoint actualOneDoFJoint = oneDoFJointsActual[i];
            String jointName = actualOneDoFJoint.getName();

            OneDoFJoint intialOneDoFJoint = initialDesiredsUponEnteringFullRobotModel.getOneDoFJointByName(jointName);

            double alpha = minimumJerkTrajectory.getPosition(); //filterStandPrepDesiredsToWalkingDesireds.getDoubleValue();

            double alphaFilteredQ = (1.0 - alpha) * intialOneDoFJoint.getqDesired() + alpha * actualOneDoFJoint.getqDesired();
            actualOneDoFJoint.setqDesired(alphaFilteredQ);
         }
      }

      public boolean isInterpolationFinished()
      {
         return  minimumJerkTrajectory.getTimeInMove() >=  timeToFilterDesiredAtCrawlStart.getDoubleValue(); //filterStandPrepDesiredsToWalkingDesireds.getDoubleValue() >= 0.9999;
      }

      @Override
      public void doAction()
      {
         double newTime = minimumJerkTrajectory.getTimeInMove() + dt;
         minimumJerkTrajectory.computeTrajectory(newTime);
      }

      @Override
      public void doTransitionIntoAction()
      {
         minimumJerkTrajectory.setMoveParameters(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, timeToFilterDesiredAtCrawlStart.getDoubleValue());

         for(int i = 0; i < oneDoFJointsActual.length; i++)
         {
            OneDoFJoint actualOneDoFJoint = oneDoFJointsActual[i];
            OneDoFJoint initialOneDofJoint = initialDesiredsUponEnteringFullRobotModel.getOneDoFJointByName(actualOneDoFJoint.getName());
            initialOneDofJoint.setqDesired(actualOneDoFJoint.getqDesired());
         }
      }

      @Override
      public void doTransitionOutOfAction()
      {

      }
   }

   private class QuadrupleSupportState extends State<CrawlGateWalkingState>
   {
      private final FramePoint swingDesired = new FramePoint();
      private final QuadrupedSupportPolygon quadStateAfterFirstStep = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon quadStateAfterSecondStep = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon quadStateAfterThirdStep = new QuadrupedSupportPolygon();

      private final RecyclingQuadrantDependentList<QuadrupedSupportPolygon> upcomingTripleSupportPolygons = new RecyclingQuadrantDependentList<>(QuadrupedSupportPolygon.class);
      private final QuadrupedSupportPolygon tripleStateWithFirstStepSwinging = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon tripleStateAfterFirstStepWithSecondSwinging = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon tripleStateAfterSecondStepWithThirdSwinging = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon tripleStateAfterThirdStepWithFourthSwinging = new QuadrupedSupportPolygon();
      private final QuadrupedSupportPolygon temporaryQuadrupedSupportPolygonForCheckingCoMInsideTriangleForSwingLeg = new QuadrupedSupportPolygon();

      private final RecyclingQuadrantDependentList<QuadrupedSupportPolygon> estimatedCommonTriangle = new RecyclingQuadrantDependentList<>(QuadrupedSupportPolygon.class);
      private final DoubleYoVariable minimumTimeInQuadSupport;
      private final DoubleYoVariable minimumTimeInQuadSupportForNormalOperation;
      private final DoubleYoVariable minimumTimeInQuadSupportAfterReverseDirection;
      private final BooleanYoVariable minimumTimeInQuadSupportElapsed;
      private final BooleanYoVariable transitioningToSafePosition;
      private final BooleanYoVariable swingLegUpdatedOnTransition;

      public QuadrupleSupportState(CrawlGateWalkingState stateEnum, double minimumTimeInQuadSupport, double minimumTimeInQuadAfterReverseDirection)
      {
         super(stateEnum);
         this.minimumTimeInQuadSupport = new DoubleYoVariable("minimumTimeInQuadSupport", registry);
         this.minimumTimeInQuadSupportForNormalOperation = new DoubleYoVariable("minimumTimeInQuadSupportForNormalOperation", registry);
         this.minimumTimeInQuadSupportAfterReverseDirection = new DoubleYoVariable("minimumTimeInQuadSupportAfterReverseDirection", registry);
         this.minimumTimeInQuadSupportElapsed = new BooleanYoVariable("minimumTimeInQuadSupportElapsed", registry);
         this.transitioningToSafePosition = new BooleanYoVariable("transitioningToSafePosition", registry);
         this.swingLegUpdatedOnTransition = new BooleanYoVariable("swingLegUpdatedOnTransition", registry);

         this.minimumTimeInQuadSupport.set(minimumTimeInQuadSupport);
         this.minimumTimeInQuadSupportForNormalOperation.set(minimumTimeInQuadSupport);
         this.minimumTimeInQuadSupportAfterReverseDirection.set(minimumTimeInQuadAfterReverseDirection);
      }

      @Override
      public void doAction()
      {
         doActionQuadrupleSupportTimer.startTimer();

         processVelocityChanges();

         if(isTransitioningToSafePosition() && comTrajectoryGenerator.isDone())
         {
            transitioningToSafePosition.set(false);
         }

         else if(shouldTranistionToTripleButItsNotSafeToStep())
         {
            shiftCoMToSafeStartingPosition();
         }

         computeCurrentSupportPolygonAndDistanceInside(null);

         doActionQuadrupleSupportTimer.stopTimer();
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
         if(isDesiredVelocityOrYawRateChanging())
         {
            //going to zero
            if(isDesiredVelocityAndYawRateZero() && !comTrajectoryGenerator.isDone())
            {
               setCoMTrajectoryToCurrentAndSetToDone();
            }

            if(!isDesiredVelocityAndYawRateZero())
            {
               //reversing sign
               if(isDesiredVelocityReversing())
               {
                  minimumTimeInQuadSupport.set(minimumTimeInQuadSupportAfterReverseDirection.getDoubleValue());
                  if(swingLegUpdatedOnTransition.getBooleanValue())
                  {
                     calculateNextSwingLeg();
                  }
                  calculateNextCoMTarget(true);
               }

               //same sign change
               else if(isVelocityChangingButKeepingSign())
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
         desiredVelocity.getFrameTupleIncludingFrame(desiredBodyVelocity);
         RobotQuadrant lastSwingLeg = swingLeg.getEnumValue();
         RobotQuadrant newSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, lastSwingLeg, desiredBodyVelocity,
               desiredYawRate.getDoubleValue());
         swingLeg.set(newSwingLeg);
      }

      private void calculateNextCoMTarget(boolean recalculateCurrent)
      {
         desiredVelocity.getFrameTupleIncludingFrame(desiredBodyVelocity);
         RobotQuadrant currentSwingLeg = swingLeg.getEnumValue();
         estimatedCommonTriangle.clear();
         if(recalculateCurrent)
         {
            RobotQuadrant previousSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, currentSwingLeg,
                  lastDesiredVelocity.getFrameTuple(), desiredYawRate.getDoubleValue());
            calculateNextThreeFootSteps(previousSwingLeg);
         }
         else
         {
            calculateNextThreeFootSteps(currentSwingLeg);
         }
         RobotQuadrant nextSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, currentSwingLeg, desiredBodyVelocity,
               desiredYawRate.getDoubleValue());

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

         FramePoint sameSideFootstep = fourFootSupportPolygon.getFootstep(sameSidQuadrant);
         FramePoint diagonalFootstep = fourFootSupportPolygon.getFootstep(diagonalQuadrant);
         FramePoint acrossBodyFootstep = fourFootSupportPolygon.getFootstep(acrossBodyQuadrant);

         centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         centerOfMassFramePoint.getFramePoint2d(centerOfMassPoint2d);

         FramePoint2d sameSideFootstep2d = new FramePoint2d();
         FramePoint2d diagonalFootstep2d = new FramePoint2d();
         FramePoint2d acrossBodyFootstep2d = new FramePoint2d();
         sameSideFootstep.getFramePoint2d(sameSideFootstep2d);
         diagonalFootstep.getFramePoint2d(diagonalFootstep2d);
         acrossBodyFootstep.getFramePoint2d(acrossBodyFootstep2d);
         FrameLineSegment2d lineSegment = new FrameLineSegment2d(ReferenceFrame.getWorldFrame());
         FramePoint2d comProjectionOnOutsideLegs2d = new FramePoint2d(ReferenceFrame.getWorldFrame());
         FramePoint comProjectionOnOutsideLegs = new FramePoint(ReferenceFrame.getWorldFrame());

         QuadrupedSupportPolygon tripleStateWithoutCurrentSwing = new QuadrupedSupportPolygon();
         fourFootSupportPolygon.getAndRemoveFootstep(tripleStateWithoutCurrentSwing, currentSwingLeg);

         switch(safeToShiftMode.getEnumValue())
         {
         case COMMON_TRIANGLE:
            RobotQuadrant nextSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, currentSwingLeg, desiredBodyVelocity, desiredYawRate.getDoubleValue());
            calculateNextThreeFootSteps(currentSwingLeg);
            QuadrupedSupportPolygon quadrupedSupportPolygon = estimatedCommonTriangle.get(nextSwingLeg);
            if(quadrupedSupportPolygon != null)
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
            lineSegment.getClosestPointOnLineSegment(comProjectionOnOutsideLegs2d, centerOfMassPoint2d);
            comProjectionOnOutsideLegs.setXY(comProjectionOnOutsideLegs2d);

            safeToStepSupportPolygon.clear();
            safeToStepSupportPolygon.setFootstep(currentSwingLeg, centerOfMassFramePoint);
            safeToStepSupportPolygon.setFootstep(diagonalQuadrant, fourFootSupportPolygon.getFootstep(diagonalQuadrant));
            safeToStepSupportPolygon.setFootstep(acrossBodyQuadrant, comProjectionOnOutsideLegs);
            safeToStepSupportPolygon.getInCircle2d(circleCenter3d);
            circleCenter2d.set(circleCenter3d.getX(), circleCenter3d.getY());
            break;

         case TTR:
            tripleStateWithoutCurrentSwing.getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(currentSwingLeg.getAcrossBodyQuadrant(), 0.1, circleCenter2d);
            break;

         case TROTLINE_MIDPOINT:
            lineSegment.set(sameSideFootstep2d, acrossBodyFootstep2d);

            FramePoint2d midpoint = lineSegment.midpoint();
            double bisectorLengthDesired = 0.1;
            FrameVector2d perpendicularBisector = new FrameVector2d();
            lineSegment.getPerpendicularBisector(perpendicularBisector, bisectorLengthDesired);
            circleCenter2d.add(midpoint, perpendicularBisector);
            if(!tripleStateWithoutCurrentSwing.isInside(circleCenter2d))
            {
               perpendicularBisector.scale(-1.0);
               circleCenter2d.add(midpoint, perpendicularBisector);
            }
            break;
         }

         /**
          * something went wrong!
          */
         if(!tripleStateWithoutCurrentSwing.isInside(circleCenter2d))
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
         if(Math.abs(desiredVelocity.getX()) < 1.0e-5)
         {
            return false;
         }

         return desiredVelocity.getX() > 0 && lastDesiredVelocity.getX() < 0 || desiredVelocity.getX() < 0 && lastDesiredVelocity.getX() > 0;
      }

      private boolean shouldTranistionToTripleButItsNotSafeToStep()
      {
         if(!comTrajectoryGenerator.isDone())
         {
            return false;
         }

         if(!isMinimumTimeInQuadSupportElapsed())
         {
            return false;
         }

         if(desiredVelocity.length() == 0.0 && desiredYawRate.getDoubleValue() == 0)
         {
            return false;
         }

         return true;
      }

      /**
       * uses actual to calculate the next three footsteps. Can use feedforward based on desireds instead of the fourFootSupportPolygon
       */
      @Override
      public void doTransitionIntoAction()
      {
         doTransitionIntoQuadrupleSupportTimer.startTimer();

         if(!isDesiredVelocityAndYawRateZero() && !isDesiredVelocityReversing())
         {
            swingLegUpdatedOnTransition.set(true);
            calculateNextSwingLeg();
            calculateNextCoMTargetTimer.startTimer();
            calculateNextCoMTarget(false);
            calculateNextCoMTargetTimer.stopTimer();
         }

         doTransitionIntoQuadrupleSupportTimer.stopTimer();
      }

      public void calculateNextThreeFootSteps2(RobotQuadrant firstSwingLeg)
      {
         RobotQuadrant secondSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, firstSwingLeg, desiredBodyVelocity, desiredYawRate.getDoubleValue());//firstSwingLeg.getNextRegularGaitSwingQuadrant();
         RobotQuadrant thirdSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, secondSwingLeg, desiredBodyVelocity, desiredYawRate.getDoubleValue());//secondSwingLeg.getNextRegularGaitSwingQuadrant();
         RobotQuadrant fourthSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, thirdSwingLeg, desiredBodyVelocity, desiredYawRate.getDoubleValue());//thirdSwingLeg.getNextRegularGaitSwingQuadrant();

         FrameVector desiredVelocityVector = desiredVelocity.getFrameTuple();
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
         tripleStateWithFirstStepSwinging.getShrunkenCommonTriangle2d(tripleStateAfterFirstStepWithSecondSwinging, firstAndSecondCommonPolygon, tempPolygonForCommonShrunken, firstSwingLeg, shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue());
         estimatedCommonTriangle.add(firstSwingLeg).set(firstAndSecondCommonPolygon);
         estimatedCommonTriangle.add(firstSwingLeg.getSameSideQuadrant());
         estimatedCommonTriangle.get(firstSwingLeg.getSameSideQuadrant()).clear();
         firstAndSecondCommonPolygon.getAndSwapSameSideFootsteps(estimatedCommonTriangle.get(firstSwingLeg.getSameSideQuadrant()), firstSwingLeg.getSide());
         drawSupportPolygon(firstAndSecondCommonPolygon, commonTriplePolygons.get(firstSwingLeg.getSide()));

         drawSupportPolygon(tripleStateAfterFirstStepWithSecondSwinging, tripleSupportPolygons.get(thirdSwingLeg));
         drawSupportPolygon(tripleStateAfterSecondStepWithThirdSwinging, tripleSupportPolygons.get(fourthSwingLeg));
         QuadrupedSupportPolygon secondAndThirdCommonPolygon = tempCommonShrunkenPolygon;
         tripleStateAfterFirstStepWithSecondSwinging.getShrunkenCommonTriangle2d(tripleStateAfterSecondStepWithThirdSwinging, secondAndThirdCommonPolygon, tempPolygonForCommonShrunken, secondSwingLeg, shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue());
         estimatedCommonTriangle.add(secondSwingLeg).set(secondAndThirdCommonPolygon);
         estimatedCommonTriangle.add(secondSwingLeg.getSameSideQuadrant());
         estimatedCommonTriangle.get(secondSwingLeg.getSameSideQuadrant()).clear();
         secondAndThirdCommonPolygon.getAndSwapSameSideFootsteps(estimatedCommonTriangle.get(secondSwingLeg.getSameSideQuadrant()), secondSwingLeg.getSide());
         drawSupportPolygon(secondAndThirdCommonPolygon, commonTriplePolygons.get(secondSwingLeg.getSide()));

         drawSupportPolygon(tripleStateAfterSecondStepWithThirdSwinging, tripleSupportPolygons.get(thirdSwingLeg));
         drawSupportPolygon(tripleStateAfterThirdStepWithFourthSwinging, tripleSupportPolygons.get(fourthSwingLeg));
         QuadrupedSupportPolygon thirdAndFourthCommonPolygon = tempCommonShrunkenPolygon;
         tripleStateAfterSecondStepWithThirdSwinging.getShrunkenCommonTriangle2d(tripleStateAfterThirdStepWithFourthSwinging, thirdAndFourthCommonPolygon, tempPolygonForCommonShrunken, thirdSwingLeg, shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue());
         estimatedCommonTriangle.add(thirdSwingLeg).set(thirdAndFourthCommonPolygon);
         estimatedCommonTriangle.add(thirdSwingLeg.getSameSideQuadrant());
         estimatedCommonTriangle.get(thirdSwingLeg.getSameSideQuadrant()).clear();
         thirdAndFourthCommonPolygon.getAndSwapSameSideFootsteps(estimatedCommonTriangle.get(thirdSwingLeg.getSameSideQuadrant()), thirdSwingLeg.getSide());
         drawSupportPolygon(thirdAndFourthCommonPolygon, commonTriplePolygons.get(thirdSwingLeg.getSide()));
      }

      public void calculateNextThreeFootSteps(RobotQuadrant firstSwingLeg)
      {
         calculateNextThreeFootstepsTimer.startTimer();

         // 1.7 MS START
         FrameVector desiredVelocityVector = desiredVelocity.getFrameTuple();
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

         for(RobotQuadrant robotQuadrant :  RobotQuadrant.values)
         {
            tripleSupportPolygons.get(robotQuadrant).hide();
         }

         // 5.1/7.2 MS 71% START
         if (!oneOffHappened) calculateNextThreeFootStepsOneOffTimer.startTimer();
         updateAndDrawCommonTriangle(tripleStateWithFirstStepSwinging, tripleStateAfterFirstStepWithSecondSwinging, firstSwingLeg, secondSwingLeg, shrunkenPolygonOffset);
         updateAndDrawCommonTriangle(tripleStateAfterFirstStepWithSecondSwinging, tripleStateAfterSecondStepWithThirdSwinging, secondSwingLeg, thirdSwingLeg, shrunkenPolygonOffset);
         updateAndDrawCommonTriangle(tripleStateAfterSecondStepWithThirdSwinging, tripleStateAfterThirdStepWithFourthSwinging, thirdSwingLeg, fourthSwingLeg, shrunkenPolygonOffset);
         if (!oneOffHappened) calculateNextThreeFootStepsOneOffTimer.stopTimer();
         // 5.1/7.2 MS 71% STOP

         calculateNextThreeFootstepsTimer.stopTimer();

         oneOffHappened = true;
      }

      private void calculateNextSupportPolygon(QuadrupedSupportPolygon supportPolygon, RobotQuadrant nextSwingLeg, FrameVector desiredVelocity, double desiredYawRate, QuadrupedSupportPolygon quadrupedSupportPolygonToPack)
      {
         swingDesired.changeFrame(ReferenceFrame.getWorldFrame());
         swingTargetGenerator.getSwingTarget(supportPolygon, nextSwingLeg, desiredVelocity, swingDesired, desiredYawRate);
         quadrupedSupportPolygonToPack.set(supportPolygon);
         quadrupedSupportPolygonToPack.setFootstep(nextSwingLeg, swingDesired);
      }

      private void updateAndDrawCommonTriangle(QuadrupedSupportPolygon firstTripleSupportPolygon, QuadrupedSupportPolygon secondTripleSupportPolygon, RobotQuadrant firstSwingLeg, RobotQuadrant secondSwingLeg, double shrunkenPolygonOffset)
      {
         if (firstTripleSupportPolygon.getFirstNonSupportingQuadrant().getSameSideQuadrant() != secondTripleSupportPolygon.getFirstNonSupportingQuadrant())
         {
            return;
         }

         firstTripleSupportPolygon.getShrunkenCommonTriangle2d(secondTripleSupportPolygon, tempCommonShrunkenPolygon, tempPolygonForCommonShrunken, firstSwingLeg, shrunkenPolygonOffset, shrunkenPolygonOffset, shrunkenPolygonOffset);
         estimatedCommonTriangle.add(firstSwingLeg).set(tempCommonShrunkenPolygon);
         estimatedCommonTriangle.add(firstSwingLeg.getSameSideQuadrant()).clear();
         tempCommonShrunkenPolygon.getAndSwapSameSideFootsteps(estimatedCommonTriangle.get(firstSwingLeg.getSameSideQuadrant()), firstSwingLeg.getSide());
         drawSupportPolygon(tempCommonShrunkenPolygon, commonTriplePolygons.get(firstSwingLeg.getSide()));
         YoFrameConvexPolygon2d firstTripleSupportPolygonGraphic = tripleSupportPolygons.get(firstSwingLeg);
         firstTripleSupportPolygonGraphic.hide();
         drawSupportPolygon(firstTripleSupportPolygon, firstTripleSupportPolygonGraphic);
         drawSupportPolygon(secondTripleSupportPolygon, tripleSupportPolygons.get(secondSwingLeg));
      }

      private final FrameVector2d tempFrameVector = new FrameVector2d();

      private void calculateTrajectoryTarget(RobotQuadrant upcomingSwingLeg, QuadrupedSupportPolygon commonTriangle, FramePoint2d comTargetToPack)
      {
         commonSupportPolygon.set(commonTriangle);
         double radius = subCircleRadius.getDoubleValue();
         boolean hasEnoughSides = commonSupportPolygon.size() >= 3;
         boolean requestedRadiusLargerThanInCircle = true;
         if(useSubCircleForBodyShiftTarget.getBooleanValue() && hasEnoughSides)
         {
            requestedRadiusLargerThanInCircle =
                  !commonSupportPolygon.getCenterOfCircleOfRadiusInCornerOfTriangleAndCheckNotLargerThanInCircle(upcomingSwingLeg, radius, comTargetToPack);
         }
         if(hasEnoughSides && requestedRadiusLargerThanInCircle)
         {
            radius = commonSupportPolygon.getInCircle2d(circleCenter3d);
            comTargetToPack.set(circleCenter3d.getX(), circleCenter3d.getY());
         }
         inscribedCircleRadius.set(radius);

         desiredCoMOffset.getFrameTuple2dIncludingFrame(tempFrameVector);
//         tempFrameVector.scale(velocitySign.getDoubleValue());
         tempFrameVector.changeFrame(ReferenceFrame.getWorldFrame());

         comTargetToPack.add(tempFrameVector.getVector());
         circleCenter.setXY(comTargetToPack);
      }

      public void reinitializeCoMTrajectory()
      {
         FramePoint finalPosition = new FramePoint(ReferenceFrame.getWorldFrame());
         comTrajectoryGenerator.getFinalPosition(finalPosition);
         initializeCoMTrajectory(new FramePoint2d(ReferenceFrame.getWorldFrame(), finalPosition.getX(), finalPosition.getY()));
      }

      public void setCoMTrajectoryToCurrentAndSetToDone()
      {
         desiredCoMVelocity.setToZero();
         FramePoint desiredBodyCurrent = desiredCoMPose.getPosition().getFramePointCopy();
         initialCoMPosition.set(desiredBodyCurrent);
         initializeCoMTrajectory(new FramePoint2d(ReferenceFrame.getWorldFrame(), initialCoMPosition.getX(), initialCoMPosition.getY()));
         comTrajectoryGenerator.setToDone();
      }

      Random random = new Random(100L);
      private void initializeCoMTrajectory(FramePoint2d target)
      {
         FramePoint desiredBodyCurrent = desiredCoMPose.getPosition().getFrameTuple();
         initialCoMPosition.set(desiredBodyCurrent);

         FrameVector desiredBodyVelocityCurrent = desiredCoMVelocity.getFrameTuple();
         initialCoMVelocity.set(desiredBodyVelocityCurrent);

         desiredCoMTarget.setXY(target);
         desiredCoMTarget.setZ(desiredBodyCurrent.getZ());

         double distance = initialCoMPosition.distance(desiredCoMTarget);
         desiredVelocity.getFrameTupleIncludingFrame(desiredBodyVelocity);

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

         if (comTrajectoryDuration.getDoubleValue() > maximumCoMTrajectoryDuration.getDoubleValue()) comTrajectoryDuration.set(maximumCoMTrajectoryDuration.getDoubleValue());
         if (comTrajectoryDuration.getDoubleValue() < minimumCoMTrajectoryDuration.getDoubleValue()) comTrajectoryDuration.set(minimumCoMTrajectoryDuration.getDoubleValue());


         //PDN: this is just so I can see when the value changes
         comTrajectoryDuration.set(comTrajectoryDuration.getDoubleValue() + random.nextDouble()/1000.0);
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

      public boolean isMinimumTimeInQuadSupportElapsed()
      {
         if(getTimeInCurrentState() > minimumTimeInQuadSupport.getDoubleValue())
         {
            minimumTimeInQuadSupportElapsed.set(true);
         }
         return minimumTimeInQuadSupportElapsed.getBooleanValue();
      }

      @Override
      public void doTransitionOutOfAction()
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
         if(isCommonTriangleNull(swingLeg))
         {
            return false;
         }
         centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         centerOfMassFramePoint.getFramePoint2d(centerOfMassPoint2d);

         return estimatedCommonTriangle.get(swingLeg).isInside(centerOfMassFramePoint);
      }

      /**
       * uses actuals
       */
      public boolean isCoMInsideTriangleForSwingLeg(RobotQuadrant swingLeg)
      {
         centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         centerOfMassFramePoint.getFramePoint2d(centerOfMassPoint2d);
         fourFootSupportPolygon.getAndRemoveFootstep(temporaryQuadrupedSupportPolygonForCheckingCoMInsideTriangleForSwingLeg, swingLeg);
//         return temporaryQuadrupedSupportPolygonForCheckingCoMInsideTriangleForSwingLeg.isInside(centerOfMassFramePoint);
         return temporaryQuadrupedSupportPolygonForCheckingCoMInsideTriangleForSwingLeg.getDistanceInside2d(centerOfMassFramePoint) > distanceInsideSupportPolygonBeforeSwingingLeg.getDoubleValue();
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
   private class TripleSupportState extends State<CrawlGateWalkingState>
   {
      private final FramePoint swingTarget = new FramePoint(ReferenceFrame.getWorldFrame());
      private final FramePoint currentDesiredInTrajectory = new FramePoint();
      private final FrameVector speedMatchVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
      private final DoubleYoVariable speedMatchScalar = new DoubleYoVariable("speedMatchScalar", registry);

      public TripleSupportState(CrawlGateWalkingState stateEnum)
      {
         super(stateEnum);
         speedMatchScalar.set(0.0);
      }

      private FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
      @Override
      public void doAction()
      {
         doActionTripleSupportTimer.startTimer();

         RobotQuadrant swingQuadrant = swingLeg.getEnumValue();

         computeFootPositionAlongSwingTrajectory(swingQuadrant, currentDesiredInTrajectory);
         currentDesiredInTrajectory.changeFrame(ReferenceFrame.getWorldFrame());
         currentSwingTarget.set(currentDesiredInTrajectory);

         desiredFeetLocations.get(swingQuadrant).setAndMatchFrame(currentDesiredInTrajectory);

         computeCurrentSupportPolygonAndDistanceInside(swingQuadrant);

         //Only need to slow down if com target is on other side of trot line
         double thresholdDistance = 0.01;
         comTrajectoryGenerator.getFinalPosition(tempFramePoint);

         distanceToTrotLine.set(computeDistanceToTrotLine2d(swingQuadrant));

         doActionTripleSupportTimer.stopTimer();
      }

      FramePoint tempCOMTarget = new FramePoint(ReferenceFrame.getWorldFrame());

      @Override
      public void doTransitionIntoAction()
      {
         doTransitionIntoTripleSupportTimer.startTimer();

         RobotQuadrant swingQuadrant = swingLeg.getEnumValue();
         swingTarget.setToZero(ReferenceFrame.getWorldFrame());
         calculateSwingTarget(swingQuadrant, swingTarget);

         YoFramePoint yoDesiredFootPosition = desiredFeetLocations.get(swingQuadrant);
         currentSwingTarget.set(swingTarget);
         finalSwingTarget.set(swingTarget);

         initializeSwingTrajectory(swingQuadrant, yoDesiredFootPosition.getFrameTuple(), swingTarget, swingDuration.getDoubleValue());
         doTransitionIntoTripleSupportTimer.stopTimer();

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
            double distanceOverTravel =  distanceToTrotLine.getDoubleValue() - distanceTravelAtEndOfSwing.getDoubleValue() - stabilityMargin;
            if (distanceOverTravel < 0.0)
            {
               slowBodyDown.set(true);
               timeToSlowTo.set(getTimeToTravelDistance(distanceToTrotLine.getDoubleValue() - stabilityMargin, swingDuration.getDoubleValue()));
               comTrajectoryTimeScaleFactor.set(timeToSlowTo.getDoubleValue()/swingDuration.getDoubleValue());
            }
            else
            {
               slowBodyDown.set(false);
               comTrajectoryTimeScaleFactor.set(1.0);
            }
         }
         if(footSwitches != null)
         {
            footSwitches.get(swingQuadrant).setFootContactState(false);
         }
      }

      FramePoint tempStartPoint = new FramePoint(ReferenceFrame.getWorldFrame());
      FramePoint tempEndPoint = new FramePoint(ReferenceFrame.getWorldFrame());
      private double getTimeToTravelDistance(double distanceToTravel, double swingTime)
      {
         double currentTimeForCOMTrajectoryGenerator = comTrajectoryGenerator.getCurrentTime();
         //recompute just to be safe
         comTrajectoryGenerator.compute(currentTimeForCOMTrajectoryGenerator);
         comTrajectoryGenerator.getPosition(tempStartPoint);

         int maxNumberOfTests = 20;
         double deltaT = swingTime/((double)maxNumberOfTests);
         double timeToSlowTo = swingTime;
         for(double timeToTest = swingTime; timeToTest > deltaT; timeToTest = timeToTest - deltaT)
         {
            timeToSlowTo = timeToTest;
            comTrajectoryGenerator.compute(timeToTest);
            comTrajectoryGenerator.getPosition(tempEndPoint);
            double distanceTraveled = tempStartPoint.getXYPlaneDistance(tempEndPoint);
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

         double distanceTraveled = tempStartPoint.getXYPlaneDistance(tempEndPoint);

         //set it back to what it was before
         comTrajectoryGenerator.compute(currentTimeForCOMTrajectoryGenerator);

         return distanceTraveled;
      }


      @Override
      public void doTransitionOutOfAction()
      {
         RobotQuadrant swingQuadrant = swingLeg.getEnumValue();
         if(footSwitches != null)
         {
            footSwitches.get(swingQuadrant).setFootContactState(true);
         }
         //reset this just in case
         comTrajectoryTimeScaleFactor.set(1.0);
      }

      private boolean isCOMTargetInsideSupportPolygon(RobotQuadrant swingQuadrant, FramePoint cOMTarget, double thresholdDistance)
      {
         currentSupportPolygon.set(fourFootSupportPolygon);
         if (swingQuadrant != null) currentSupportPolygon.removeFootstep(swingQuadrant);
         cOMTarget.changeFrame(ReferenceFrame.getWorldFrame());
         double distanceInside = currentSupportPolygon.getDistanceInside2d(cOMTarget);
         return distanceInside > thresholdDistance;
      }

      /**
       * tries to do some fancy ground speed matching, not well tuned
       */
      private void initializeSwingTrajectory(RobotQuadrant swingLeg, FramePoint swingInitial, FramePoint swingTarget, double swingTime)
      {
         QuadrupedSwingTrajectoryGenerator swingTrajectoryGenerator = swingTrajectoryGenerators.get(swingLeg);
         speedMatchVelocity.setIncludingFrame(desiredBodyVelocity);
         speedMatchVelocity.changeFrame(ReferenceFrame.getWorldFrame());
         speedMatchVelocity.scale(speedMatchScalar.getDoubleValue());
         swingTrajectoryGenerator.initializeSwing(swingTime, swingInitial, swingHeight.getDoubleValue(), swingTarget, speedMatchVelocity);
      }

      private void computeFootPositionAlongSwingTrajectory(RobotQuadrant swingLeg, FramePoint framePointToPack)
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
      for(OneDoFJoint oneDofJoint : oneDoFJointsActual)
      {
         oneDofJoint.setUnderPositionControl(true);
      }

      actualFullRobotModel.updateFrames();
      referenceFrames.updateFrames();

//      setFeedForwardToActuals();
      updateEstimates();
      updateFeedForwardModelAndFrames();
      walkingStateMachine.setCurrentState(CrawlGateWalkingState.ALPHA_FILTERING_DESIREDS);
   }

   @Override
   public void onExit()
   {

   }
}
