package us.ihmc.quadrupedRobotics.controller;

import java.awt.Color;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.quadrupedRobotics.controller.state.QuadrupedControllerState;
import us.ihmc.quadrupedRobotics.dataProviders.DesiredVelocityProvider;
import us.ihmc.quadrupedRobotics.dataProviders.DesiredYawRateProvider;
import us.ihmc.quadrupedRobotics.footstepChooser.MidFootZUpSwingTargetGenerator;
import us.ihmc.quadrupedRobotics.footstepChooser.SwingTargetGenerator;
import us.ihmc.quadrupedRobotics.inverseKinematics.QuadrupedLegInverseKinematicsCalculator;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedPositionBasedCrawlControllerParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.stateEstimator.QuadrupedStateEstimator;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.swingLegChooser.DefaultGaitSwingLegChooser;
import us.ihmc.quadrupedRobotics.swingLegChooser.NextSwingLegChooser;
import us.ihmc.quadrupedRobotics.trajectory.QuadrupedSwingTrajectoryGenerator;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.math.filters.AlphaFilteredWrappingYoVariable;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
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
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.CenterOfMassJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactCircle;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;

public class QuadrupedPositionBasedCrawlController extends QuadrupedController
{
   private static final double DEFAULT_HEADING_CORRECTION_BREAK_FREQUENCY = 1.0;
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
      QUADRUPLE_SUPPORT, TRIPLE_SUPPORT
   }
   
   private enum SafeStartingShiftMode
   {
      COMMON_TRIANGLE, CENTROID, TTR, COM_INCIRCLE, TROTLINE_MIDPOINT
   };
   
   private final EnumYoVariable<SafeStartingShiftMode> safeToShiftMode = new EnumYoVariable<>("safeStartingShiftMode", registry, SafeStartingShiftMode.class);
   {
      safeToShiftMode.set(SafeStartingShiftMode.TROTLINE_MIDPOINT);
   }
   
   private final SDFFullRobotModel feedForwardFullRobotModel;
   private final QuadrupedReferenceFrames feedForwardReferenceFrames;
   
   private final StateMachine<CrawlGateWalkingState> walkingStateMachine;
   private final QuadrupleSupportState quadrupleSupportState;
   private final QuadrupedLegInverseKinematicsCalculator inverseKinematicsCalculators;
   private final NextSwingLegChooser nextSwingLegChooser;
   private final SwingTargetGenerator swingTargetGenerator;
   private final QuadrupedStateEstimator stateEstimator;
   private final SDFFullRobotModel fullRobotModel;
   private final QuadrupedReferenceFrames referenceFrames;
   private final CenterOfMassJacobian centerOfMassJacobian;
   private final CenterOfMassJacobian feedForwardCenterOfMassJacobian;
   private final ReferenceFrame feedForwardBodyFrame;
   private final ReferenceFrame comFrame;
   private final PoseReferenceFrame desiredCoMPoseReferenceFrame = new PoseReferenceFrame("desiredCoMPoseReferenceFrame", ReferenceFrame.getWorldFrame());
   private final YoFramePoint desiredCoMPosition = new YoFramePoint("desiredCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector desiredCoMVelocity = new YoFrameVector("desiredCoMVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final FramePoint desiredCoMFramePosition = new FramePoint(ReferenceFrame.getWorldFrame());
   private final FramePose desiredCoMFramePose = new FramePose(ReferenceFrame.getWorldFrame());
   
   private final BooleanYoVariable runOpenLoop = new BooleanYoVariable("runOpenLoop", "If true, runs in open loop mode. The leg motions will not depend on any feedback signals.", registry);
   
   private final YoFramePoint2d desiredCoMOffset;

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
   private final YoFrameOrientation filteredDesiredCoMOrientation = new YoFrameOrientation(filteredDesiredCoMYaw, filteredDesiredCoMPitch, filteredDesiredCoMRoll, ReferenceFrame.getWorldFrame());
   private final YoFramePose desiredCoMPose = new YoFramePose(desiredCoMPosition, filteredDesiredCoMOrientation);

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
   private final YoFrameLineSegment2d nominalYawLineSegment = new YoFrameLineSegment2d("nominalYawLineSegment", "", ReferenceFrame.getWorldFrame(), registry);
   private final YoArtifactLineSegment2d nominalYawArtifact = new YoArtifactLineSegment2d("nominalYawArtifact", nominalYawLineSegment, Color.YELLOW, 0.02, 0.02);
   private final FramePoint2d endPoint2d = new FramePoint2d();
   private final FramePoint centroidFramePoint = new FramePoint();
   private final FramePoint2d centroidFramePoint2d = new FramePoint2d();
   
   private final QuadrupedSupportPolygon safeToStepSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon fourFootSupportPolygon = new QuadrupedSupportPolygon();
   private final QuadrupedSupportPolygon commonSupportPolygon = new QuadrupedSupportPolygon();
   private final ConvexPolygon2d supportPolygonHolder = new ConvexPolygon2d();

   private final QuadrantDependentList<QuadrupedSwingTrajectoryGenerator> swingTrajectoryGenerators = new QuadrantDependentList<>();
   private final DoubleYoVariable swingDuration = new DoubleYoVariable("swingDuration", registry);
   private final DoubleYoVariable swingHeight = new DoubleYoVariable("swingHeight", registry);
   
   private final QuadrantDependentList<ReferenceFrame> legAttachmentFrames = new QuadrantDependentList<>();
   private final QuadrantDependentList<YoFramePoint> actualFeetLocations = new QuadrantDependentList<YoFramePoint>();
   private final QuadrantDependentList<YoFramePoint> desiredFeetLocations = new QuadrantDependentList<YoFramePoint>();
   private final FramePoint desiredFootPosition = new FramePoint();
   private final FramePoint desiredFootPositionInBody = new FramePoint();
   
   private final QuadrantDependentList<YoFrameVector> desiredFeetPositionsInLegAttachmentFrame = new QuadrantDependentList<YoFrameVector>();
   private final QuadrantDependentList<YoFrameVector> actualFeetPositionsInLegAttachmentFrame = new QuadrantDependentList<YoFrameVector>();
   private final Vector3d desiredFootPositionForInverseKinematics = new Vector3d();
   
   private final YoFrameConvexPolygon2d supportPolygon = new YoFrameConvexPolygon2d("quadPolygon", "", ReferenceFrame.getWorldFrame(), 4, registry);
   private final YoFrameConvexPolygon2d currentTriplePolygon = new YoFrameConvexPolygon2d("currentTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameConvexPolygon2d upcommingTriplePolygon = new YoFrameConvexPolygon2d("upcommingTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameConvexPolygon2d commonTriplePolygon = new YoFrameConvexPolygon2d("commonTriplePolygon", "", ReferenceFrame.getWorldFrame(), 3, registry);
   
   private final YoFrameConvexPolygon2d commonTriplePolygonLeft = new YoFrameConvexPolygon2d("commonTriplePolygonLeft", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final YoFrameConvexPolygon2d commonTriplePolygonRight = new YoFrameConvexPolygon2d("commonTriplePolygonRight", "", ReferenceFrame.getWorldFrame(), 3, registry);
   private final SideDependentList<YoFrameConvexPolygon2d> commonTriplePolygons = new SideDependentList<>(commonTriplePolygonLeft, commonTriplePolygonRight);
   private final YoFrameConvexPolygon2d[] tripleSupportPolygons = new YoFrameConvexPolygon2d[6];
   private final YoArtifactPolygon[] tripleSupportArtifactPolygons = new YoArtifactPolygon[6];
   
   private final YoFramePoint circleCenter = new YoFramePoint("circleCenter", ReferenceFrame.getWorldFrame(), registry);
   private final Point2d circleCenter2d = new Point2d();
   private final YoGraphicPosition circleCenterGraphic = new YoGraphicPosition("circleCenterGraphic", circleCenter, 0.005, YoAppearance.Green());

   private final DoubleYoVariable inscribedCircleRadius = new DoubleYoVariable("inscribedCircleRadius", registry);
   private final YoArtifactCircle inscribedCircle = new YoArtifactCircle("inscribedCircle", circleCenter, inscribedCircleRadius, Color.BLACK);
   
   private final BooleanYoVariable useSubCircleForBodyShiftTarget = new BooleanYoVariable("useSubCircleForBodyShiftTarget", registry);
   private final DoubleYoVariable subCircleRadius = new DoubleYoVariable("subCircleRadius", registry);
   private final DoubleYoVariable comCloseRadius = new DoubleYoVariable("comCloseRadius", "Distance check from final desired circle to CoM for transitioning into swing state", registry);

   private final YoFrameVector yoVectorToSubtract = new YoFrameVector("yoVectorToSubtract", ReferenceFrame.getWorldFrame(), registry);
   private final FramePoint centerOfMassInBody = new FramePoint();
   private final FramePoint desiredRootJointPosition = new FramePoint();
   private final FrameVector vectorToSubtractHolder = new FrameVector();
   private final Vector3d linearVelocityHolder = new Vector3d();
   
   private final BooleanYoVariable isCoMInsideTriangleForSwingLeg = new BooleanYoVariable("isCoMInsideTriangleForSwingLeg", registry);
   private final BooleanYoVariable isCoMCloseToFinalDesired = new BooleanYoVariable("isCoMCloseToFinalDesired", registry);
   private final BooleanYoVariable useCommonTriangleForSwingTransition = new BooleanYoVariable("useCommonTriangleForSwingTransition", registry);

   private final YoFrameVector feedForwardCenterOfMassVelocity = new YoFrameVector("feedForwardCenterOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);
   
   private final YoFramePoint centerOfMassPosition = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector centerOfMassVelocity = new YoFrameVector("centerOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);

   private final FramePoint centerOfMassFramePoint = new FramePoint();
   private final Point2d centerOfMassPoint2d = new Point2d();
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
   private final YoGraphicReferenceFrame leftMidZUpFrameViz;
   private final YoGraphicReferenceFrame rightMidZUpFrameViz;
   
   public final BooleanYoVariable isVelocityNegative = new BooleanYoVariable("isVelocityNegative", registry);
   public final DoubleYoVariable velocitySign = new DoubleYoVariable("velocitySign", registry);
   
   private final QuadrantDependentList<YoGraphicReferenceFrame> desiredAttachmentFrames = new QuadrantDependentList<YoGraphicReferenceFrame>();
   private final QuadrantDependentList<YoGraphicReferenceFrame> actualAttachmentFrames = new QuadrantDependentList<YoGraphicReferenceFrame>();

   
   /** body sway trajectory **/
   private final BooleanYoVariable comTrajectoryGeneratorRequiresReInitailization = new BooleanYoVariable("comTrajectoryGeneratorRequiresReInitailization", registry);
   private final VelocityConstrainedPositionTrajectoryGenerator comTrajectoryGenerator = new VelocityConstrainedPositionTrajectoryGenerator("comTraj", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint initialCoMPosition = new YoFramePoint("initialCoMPosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector initialCoMVelocity = new YoFrameVector("initialCoMVelocity", ReferenceFrame.getWorldFrame(), registry);
   
   private final DoubleYoVariable comTrajectoryTimeStart = new DoubleYoVariable("comTrajectoryTimeStart", registry);
   private final DoubleYoVariable comTrajectoryTimeCurrent = new DoubleYoVariable("comTrajectoryTimeCurrent", registry);
   private final DoubleYoVariable comTrajectoryTimeDesired = new DoubleYoVariable("comTrajectoryTimeDesired", registry);

   private final DoubleYoVariable comTrajectoryDuration = new DoubleYoVariable("comTrajectoryDuration", registry);
   private final DoubleYoVariable maximumCoMTrajectoryDuration = new DoubleYoVariable("maximumCoMTrajectoryDuration", registry);
   private final DoubleYoVariable minimumCoMTrajectoryDuration = new DoubleYoVariable("minimumCoMTrajectoryDuration", registry);
   
   private VectorProvider desiredVelocityProvider;
   private DoubleProvider desiredYawRateProvider;
   
   public QuadrupedPositionBasedCrawlController(final double dt, QuadrupedRobotParameters robotParameters, SDFFullRobotModel fullRobotModel,
         QuadrupedStateEstimator stateEstimator, QuadrupedLegInverseKinematicsCalculator quadrupedInverseKinematicsCalulcator, final GlobalDataProducer dataProducer, DoubleYoVariable yoTime,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, YoGraphicsListRegistry yoGraphicsListRegistryForDetachedOverhead)
   {
      super(QuadrupedControllerState.POSITION_CRAWL);
      QuadrupedPositionBasedCrawlControllerParameters quadrupedControllerParameters = robotParameters.getQuadrupedPositionBasedCrawlControllerParameters();

      swingDuration.set(quadrupedControllerParameters.getDefaultSwingDuration());
      swingHeight.set(quadrupedControllerParameters.getDefaultSwingHeight());
      subCircleRadius.set(quadrupedControllerParameters.getDefaultSubCircleRadius());
      comCloseRadius.set(quadrupedControllerParameters.getDefaultCoMCloseToFinalDesiredTransitionRadius());
      minYawRate.set(quadrupedControllerParameters.getMaxYawRate() * -1.0);
      maxYawRate.set(quadrupedControllerParameters.getMaxYawRate());
      
      useSubCircleForBodyShiftTarget.set(true);
      swingLeg.set(RobotQuadrant.HIND_LEFT);
      
      runOpenLoop.set(true);
      
      this.robotTimestamp = yoTime;
      this.dt = dt;
      this.referenceFrames = new QuadrupedReferenceFrames(fullRobotModel, robotParameters.getJointMap(), robotParameters.getPhysicalProperties());
      this.fullRobotModel = fullRobotModel;
      this.centerOfMassJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      this.walkingStateMachine = new StateMachine<CrawlGateWalkingState>(name, "walkingStateTranistionTime", CrawlGateWalkingState.class, yoTime, registry);
      this.inverseKinematicsCalculators = quadrupedInverseKinematicsCalulcator;
      
      referenceFrames.updateFrames();
      comFrame = referenceFrames.getCenterOfMassFrame();
      
      feedForwardFullRobotModel = robotParameters.createFullRobotModel();
      this.feedForwardCenterOfMassJacobian = new CenterOfMassJacobian(feedForwardFullRobotModel.getElevator());
      feedForwardReferenceFrames = new QuadrupedReferenceFrames(feedForwardFullRobotModel, robotParameters.getJointMap(), robotParameters.getPhysicalProperties());
      feedForwardReferenceFrames.updateFrames();
      feedForwardBodyFrame = feedForwardReferenceFrames.getBodyFrame();
//      this.nextSwingLegChooser = new QuadrupedGaitSwingLegChooser(feedForwardReferenceFrames, registry, yoGraphicsListRegistry);
      this.nextSwingLegChooser = new DefaultGaitSwingLegChooser();
      
      desiredCoMOffset = new YoFramePoint2d("desiredCoMOffset", feedForwardReferenceFrames.getBodyZUpFrame(), registry);
      desiredCoMOffset.set(quadrupedControllerParameters.getDefaultDesiredCoMOffset());

      updateFeedForwardModelAndFrames();
      
      for (RobotQuadrant robotQuadrant: RobotQuadrant.values)
      {
    	  YoGraphicReferenceFrame desiredAttachmentFrame = new YoGraphicReferenceFrame("ffLegAttachment", feedForwardReferenceFrames.getLegAttachmentFrame(robotQuadrant), registry, 0.25, YoAppearance.Purple());
    	  desiredAttachmentFrames.put(robotQuadrant, desiredAttachmentFrame);
    	  
    	  YoGraphicReferenceFrame actualAttachmentFrame = new YoGraphicReferenceFrame("legAttachment", referenceFrames.getLegAttachmentFrame(robotQuadrant), registry, 0.25, YoAppearance.Green());
    	  actualAttachmentFrames.put(robotQuadrant, actualAttachmentFrame);
    	  
          yoGraphicsListRegistry.registerYoGraphic("AttachementFrames", desiredAttachmentFrame);
          yoGraphicsListRegistry.registerYoGraphic("AttachementFrames", actualAttachmentFrame);
      }
      
      this.swingTargetGenerator = new MidFootZUpSwingTargetGenerator(quadrupedControllerParameters, feedForwardReferenceFrames, registry);
      this.stateEstimator = stateEstimator;
      desiredVelocityProvider = new DesiredVelocityProvider(dataProducer, "userProvided", registry);
      desiredYawRateProvider = new DesiredYawRateProvider(dataProducer, "userProvided", registry);

      desiredVelocity = new YoFrameVector("desiredVelocity", feedForwardBodyFrame, registry);
      lastDesiredVelocity = new YoFrameVector("lastDesiredVelocity", feedForwardBodyFrame, registry); 
      
      desiredVelocity.setX(0.0);
      comTrajectoryTimeDesired.set(1.0);
      maximumCoMTrajectoryDuration.set(6.0);
      minimumCoMTrajectoryDuration.set(0.01);
       
      shrunkenPolygonSize.set(0.02);
      
      comPoseYoGraphic = new YoGraphicReferenceFrame("rasta_", referenceFrames.getCenterOfMassFrame(), registry, 0.25, YoAppearance.Green());
      feedForwardCoMPoseYoGraphic = new YoGraphicReferenceFrame("feedForwardRasta_", feedForwardReferenceFrames.getCenterOfMassFrame(), registry, 0.25, YoAppearance.Purple());
      

      leftMidZUpFrameViz = new YoGraphicReferenceFrame(referenceFrames.getSideDependentMidFeetZUpFrame(RobotSide.LEFT), registry, 0.2);
      rightMidZUpFrameViz = new YoGraphicReferenceFrame(referenceFrames.getSideDependentMidFeetZUpFrame(RobotSide.RIGHT), registry, 0.2);
      
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
         swingTrajectoryGenerators.put(robotQuadrant, new QuadrupedSwingTrajectoryGenerator(robotQuadrant, registry, yoGraphicsListRegistry, dt));

         ReferenceFrame footReferenceFrame = referenceFrames.getFootFrame(robotQuadrant);
         ReferenceFrame legAttachmentFrame = referenceFrames.getLegAttachmentFrame(robotQuadrant);
         
         legAttachmentFrames.put(robotQuadrant, legAttachmentFrame);

         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         
         YoFramePoint actualFootPosition = new YoFramePoint(prefix + "actualFootPosition", ReferenceFrame.getWorldFrame(), registry);
         actualFeetLocations.put(robotQuadrant, actualFootPosition);
         
         YoFramePoint desiredFootLocation = new YoFramePoint(prefix + "FootDesiredPosition", ReferenceFrame.getWorldFrame(), registry);
         
         FramePoint footPosition = new FramePoint(footReferenceFrame);
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         footPosition.setZ(0.0);
         desiredFootLocation.set(footPosition);
         desiredFeetLocations.put(robotQuadrant, desiredFootLocation);
         
         YoFrameVector footPositionInLegAttachementFrame = new YoFrameVector(prefix + "FootPositionInLegFrame", referenceFrames.getLegAttachmentFrame(robotQuadrant), registry);
         desiredFeetPositionsInLegAttachmentFrame.put(robotQuadrant, footPositionInLegAttachementFrame);
         
         YoFrameVector actualFootPositionInLegAttachementFrame = new YoFrameVector(prefix + "ActualFootPositionInLegFrame", referenceFrames.getLegAttachmentFrame(robotQuadrant), registry);
         actualFeetPositionsInLegAttachmentFrame.put(robotQuadrant, actualFootPositionInLegAttachementFrame);
      }
      
      for (int i = 0; i < tripleSupportPolygons.length; i++)
      {
         String polygonName = "trippleSupport" + i;
         YoFrameConvexPolygon2d yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d(polygonName, "", ReferenceFrame.getWorldFrame(), 3, registry);
         tripleSupportPolygons[i] = yoFrameConvexPolygon2d;

         float saturation = 0.5f;
         float brightness = 0.5f;
         float hue = (float) (0.1 * i);

         tripleSupportArtifactPolygons[i] = new YoArtifactPolygon(polygonName, yoFrameConvexPolygon2d, Color.getHSBColor(hue, saturation, brightness), false);
         yoGraphicsListRegistry.registerArtifact(polygonName, tripleSupportArtifactPolygons[i]);
         yoGraphicsListRegistryForDetachedOverhead.registerArtifact(polygonName, tripleSupportArtifactPolygons[i]);
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
      
      walkingStateMachine.addState(quadrupleSupportState);
      walkingStateMachine.addState(tripleSupportState);
      walkingStateMachine.setCurrentState(CrawlGateWalkingState.QUADRUPLE_SUPPORT);

      StateTransitionCondition quadrupleToTripleCondition = new QuadrupleToTripleCondition(quadrupleSupportState);
      StateTransitionCondition tripleToQuadrupleCondition = new TripleToQuadrupleCondition(tripleSupportState);
      
      quadrupleSupportState.addStateTransition(new StateTransition<CrawlGateWalkingState>(CrawlGateWalkingState.TRIPLE_SUPPORT, quadrupleToTripleCondition));
      tripleSupportState.addStateTransition(new StateTransition<CrawlGateWalkingState>(CrawlGateWalkingState.QUADRUPLE_SUPPORT, tripleToQuadrupleCondition));
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
      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("quadSupportPolygonArtifact", supportPolygon, Color.blue, false);
      YoArtifactPolygon currentTriplePolygonArtifact = new YoArtifactPolygon("currentTriplePolygonArtifact", currentTriplePolygon, Color.GREEN, false);
      YoArtifactPolygon upcommingTriplePolygonArtifact = new YoArtifactPolygon("upcommingTriplePolygonArtifact", upcommingTriplePolygon, Color.yellow, false);
      YoArtifactPolygon commonTriplePolygonArtifact = new YoArtifactPolygon("commonTriplePolygonArtifact", commonTriplePolygon, Color.RED, false);
      YoArtifactPolygon commonTriplePolygonLeftArtifact = new YoArtifactPolygon("commonTriplePolygonLeftArtifact", commonTriplePolygonLeft, Color.BLUE, false);
      YoArtifactPolygon commonTriplePolygonRightArtifact = new YoArtifactPolygon("commonTriplePolygonRightArtifact", commonTriplePolygonRight, Color.MAGENTA, false);
      
      yoGraphicsListRegistry.registerArtifact("supportPolygon", supportPolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("currentTriplePolygon", currentTriplePolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("upcommingTriplePolygon", upcommingTriplePolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("commonTriplePolygon", commonTriplePolygonArtifact);
      yoGraphicsListRegistry.registerArtifact("commonTriplePolygonLeft", commonTriplePolygonLeftArtifact);
      yoGraphicsListRegistry.registerArtifact("commonTriplePolygonRight", commonTriplePolygonRightArtifact);
      
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("supportPolygon", supportPolygonArtifact);
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("currentTriplePolygon", currentTriplePolygonArtifact);
      yoGraphicsListRegistryForDetachedOverhead.registerArtifact("upcommingTriplePolygon", upcommingTriplePolygonArtifact);
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
      yoGraphicsListRegistry.registerYoGraphic("leftMidZUpFrameViz", leftMidZUpFrameViz);
      yoGraphicsListRegistry.registerYoGraphic("rightMidZUpFrameViz", rightMidZUpFrameViz);
      
      yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("centerOfMassViz", centerOfMassViz);
      yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("desiredCoMPoseYoGraphic", desiredCoMPoseYoGraphic);
      yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("comPoseYoGraphic", comPoseYoGraphic);
      yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("leftMidZUpFrameViz", leftMidZUpFrameViz);
      yoGraphicsListRegistryForDetachedOverhead.registerYoGraphic("rightMidZUpFrameViz", rightMidZUpFrameViz);

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
   public void doAction()
   {
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
      alphaFilterDesiredBodyOrientation();
      updateDesiredCoMPose();
      updateLegsBasedOnDesiredCoM();
      computeDesiredPositionsAndStoreInFullRobotModel();
      updateFeedForwardModelAndFrames();
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

   private void updateFeedForwardModelAndFrames() 
   {
	   OneDoFJoint[] oneDoFJointsFeedforward = feedForwardFullRobotModel.getOneDoFJoints();
	   OneDoFJoint[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

	   for (int i=0; i<oneDoFJoints.length; i++)
	   {
		   OneDoFJoint oneDoFJoint = oneDoFJoints[i];
		   OneDoFJoint oneDoFJointFeedforward = oneDoFJointsFeedforward[i];

//		   oneDoFJointFeedforward.setQ(oneDoFJoint.getQ());
		   double qd = oneDoFJoint.getqDesired() - oneDoFJointFeedforward.getQ();
		   oneDoFJointFeedforward.setQd(qd * 1.0 / dt);   	  
		   oneDoFJointFeedforward.setQ(oneDoFJoint.getqDesired());
	   }

	   SixDoFJoint feedForwardRootJoint = feedForwardFullRobotModel.getRootJoint();
	   SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
	   
	   feedForwardRootJoint.setRotation(desiredCoMOrientation.getYaw().getDoubleValue(), desiredCoMOrientation.getPitch().getDoubleValue(), desiredCoMOrientation.getRoll().getDoubleValue());
	   feedForwardFullRobotModel.updateFrames();
	   
//	   Vector3d rootJointPosition = new Vector3d();
//	   rootJoint.packTranslation(rootJointPosition);
//	   feedForwardRootJoint.setPosition(rootJointPosition);
	   
	   centerOfMassInBody.setIncludingFrame(comFrame, 0.0, 0.0, 0.0);
	   centerOfMassInBody.changeFrame(rootJoint.getFrameAfterJoint());
	   
	   vectorToSubtractHolder.setIncludingFrame(feedForwardFullRobotModel.getRootJoint().getFrameAfterJoint(), centerOfMassInBody.getPoint());
	   vectorToSubtractHolder.changeFrame(ReferenceFrame.getWorldFrame());

	   yoVectorToSubtract.set(vectorToSubtractHolder);
//	   System.out.println("VectorToSubtract = " + vectorToSubtract);
	   
	   desiredRootJointPosition.setIncludingFrame(desiredCoM.getFrameTuple());
	   desiredRootJointPosition.sub(vectorToSubtractHolder);
      feedForwardRootJoint.packTranslation(linearVelocityHolder);
	   feedForwardRootJoint.setPosition(desiredRootJointPosition.getPoint());
	   linearVelocityHolder.sub(desiredRootJointPosition.getPoint(), linearVelocityHolder);
	   feedForwardRootJoint.setLinearVelocityInWorld(linearVelocityHolder);

	   feedForwardFullRobotModel.updateFrames();
	   feedForwardReferenceFrames.updateFrames();
   }

   private double lastProvidedDesiredYawRate = 0.0;
   
   private void pollDataProviders()
   {
      if(desiredVelocityProvider != null)
      {
         FrameVector providedDesiredVelocity = new FrameVector(desiredVelocity.getReferenceFrame());
         desiredVelocityProvider.get(providedDesiredVelocity);
         desiredVelocity.set(providedDesiredVelocity);
      }

      if(desiredYawRateProvider != null)
      {
         double providedDesiredYawRate = desiredYawRateProvider.getValue();
         providedDesiredYawRate = MathTools.clipToMinMax(providedDesiredYawRate, minYawRate.getDoubleValue(), maxYawRate.getDoubleValue());
         if (providedDesiredYawRate != lastProvidedDesiredYawRate)
         {
            desiredYawRate.set(providedDesiredYawRate);
            lastProvidedDesiredYawRate = providedDesiredYawRate;
         }
      }
   }
   
   private final FramePoint footLocation = new FramePoint();
   private final FrameVector tempFrameVector = new FrameVector();
   /**
    * uses feedback to update the CoM Velocity, ICP, and Actual Foot Positions
    */
   private void updateEstimates()
   {

      // compute center of mass position and velocity
      FramePoint feedForwardCoMPosition = new FramePoint(feedForwardReferenceFrames.getCenterOfMassFrame());
      feedForwardCoMPosition.changeFrame(ReferenceFrame.getWorldFrame());
      feedForwardCenterOfMassJacobian.compute();
      feedForwardCenterOfMassJacobian.packCenterOfMassVelocity(tempFrameVector);
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
      FramePoint comPosition = new FramePoint(comFrame);
      comPosition.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassJacobian.compute();
      centerOfMassJacobian.packCenterOfMassVelocity(tempFrameVector);
      tempFrameVector.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassVelocity.set(tempFrameVector);

      // compute instantaneous capture point
      double zFoot = actualFeetLocations.get(fourFootSupportPolygon.getLowestFootstep()).getZ();
      double zDelta = comPosition.getZ() - zFoot;
      double omega = Math.sqrt(9.81 / zDelta);
      currentICP.setX(comPosition.getX() + centerOfMassVelocity.getX() / omega);
      currentICP.setY(comPosition.getY() + centerOfMassVelocity.getY() / omega);
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
         footLocation.setToZero(footFrame);
         footLocation.changeFrame(ReferenceFrame.getWorldFrame());

         YoFramePoint yoFootLocation = actualFeetLocations.get(robotQuadrant);
         yoFootLocation.set(footLocation);

         // Use the desired foot locations instead of the actual locations
         YoFramePoint desiredFootLocation = desiredFeetLocations.get(robotQuadrant);
         fourFootSupportPolygon.setFootstep(robotQuadrant, desiredFootLocation.getFramePointCopy());
      }
   }

   private void updateGraphics()
   {
      desiredCoMPoseYoGraphic.update();
      leftMidZUpFrameViz.update();
      rightMidZUpFrameViz.update();
      desiredCoMViz.update();
      comPoseYoGraphic.update();
      feedForwardCoMPoseYoGraphic.update();
//      centerOfMassFramePoint.setToZero(comFrame);
      centerOfMassFramePoint.setToZero(desiredCoMPoseReferenceFrame);
      centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      centerOfMassPosition.set(centerOfMassFramePoint);
      drawSupportPolygon(fourFootSupportPolygon, supportPolygon);
      
      for (RobotQuadrant robotQuadrant: RobotQuadrant.values)
      {
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
         comTrajectoryTimeCurrent.set(robotTimestamp.getDoubleValue() - comTrajectoryTimeStart.getDoubleValue());
         comTrajectoryGenerator.compute(comTrajectoryTimeCurrent.getDoubleValue());
         comTrajectoryGenerator.get(desiredCoMFramePosition);
         comTrajectoryGenerator.packVelocity(desiredCoMVelocity);
         desiredCoMFramePosition.setZ(desiredCoMPose.getPosition().getZ());
         
         desiredCoMPose.setPosition(desiredCoMFramePosition);
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
      desiredYaw.set(nominalYaw.getDoubleValue());
   }
   
   /**
    * desired CoM height in world
    */
   private void updateDesiredHeight()
   {
      filteredDesiredCoMHeight.update();
      desiredCoMPosition.setZ(filteredDesiredCoMHeight.getDoubleValue());
   }
   
   private void alphaFilterDesiredBodyOrientation()
   {
      filteredDesiredCoMYaw.update();
      filteredDesiredCoMPitch.update();
      filteredDesiredCoMRoll.update();
   }
   
   private void updateDesiredCoMPose()
   {
      desiredCoMPose.getFramePose(desiredCoMFramePose);
      desiredCoM.set(desiredCoMFramePose.getFramePointCopy());
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
   
   private void computeDesiredPositionsAndStoreInFullRobotModel()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {         
         desiredFeetPositionsInLegAttachmentFrame.get(robotQuadrant).get(desiredFootPositionForInverseKinematics);
         computeDesiredPositionsAndStoreInFullRobotModel(robotQuadrant, desiredFootPositionForInverseKinematics);
      }
   }
   
   private void computeDesiredPositionsAndStoreInFullRobotModel(RobotQuadrant robotQuadrant, Vector3d footPositionInLegAttachmentFrame)
   {
      inverseKinematicsCalculators.solveForEndEffectorLocationInBodyAndUpdateDesireds(robotQuadrant, footPositionInLegAttachmentFrame, fullRobotModel);
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
      desiredFootPositionInBody.setIncludingFrame(comFrame, desiredFootPosition.getPoint());
      desiredFootPositionInBody.changeFrame(referenceFrames.getLegAttachmentFrame(robotQuadrant));

      desiredFeetPositionsInLegAttachmentFrame.get(robotQuadrant).set(desiredFootPositionInBody.getPoint());
   }
   
   /**
   * Calculates the next swing target in world using the actual feet,
   * should create another method for feedforward then handle moving to body frame
   */
   public void calculateSwingTarget(RobotQuadrant swingLeg, FramePoint framePointToPack)
   {
      
      FrameVector comTrajComputedVelocity = desiredCoMVelocity.getFrameVectorCopy();
      FrameVector desiredVelocityVector = desiredVelocity.getFrameVectorCopy();
      
      comTrajComputedVelocity.changeFrame(feedForwardBodyFrame);
      desiredVelocityVector.changeFrame(feedForwardBodyFrame);
      
      FrameVector expectedAverageVelocity = new FrameVector();
      
      if(desiredVelocityVector.length() < 1e-3)
      {
         expectedAverageVelocity.setIncludingFrame(desiredVelocityVector);
      }
      else if(comTrajComputedVelocity.getX() > 0 && desiredVelocityVector.getX() < 0 || comTrajComputedVelocity.getX() < 0 && desiredVelocityVector.getX() > 0)
      {
         expectedAverageVelocity.setIncludingFrame(desiredVelocityVector);
         expectedAverageVelocity.scale(swingDuration.getDoubleValue() / 3.0);
      } 
      else
      {
         expectedAverageVelocity.setIncludingFrame(desiredVelocityVector);
         expectedAverageVelocity.sub(desiredVelocityVector, comTrajComputedVelocity);
         expectedAverageVelocity.scale(swingDuration.getDoubleValue() / 3.0);
         expectedAverageVelocity.add(comTrajComputedVelocity);
      }
      
      double yawRate = desiredYawRate.getDoubleValue();
      swingTargetGenerator.getSwingTarget(swingLeg, feedForwardReferenceFrames.getLegAttachmentFrame(swingLeg), expectedAverageVelocity, swingDuration.getDoubleValue(), framePointToPack, yawRate);
   }
   
   private void drawSupportPolygon(QuadrupedSupportPolygon supportPolygon, YoFrameConvexPolygon2d yoFramePolygon)
   {
      supportPolygonHolder.clear();
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         FramePoint footstep = supportPolygon.getFootstep(quadrant);
         if(footstep != null)
         {
            supportPolygonHolder.addVertex(footstep.getX(), footstep.getY());
         }
      }
      supportPolygonHolder.update();
      yoFramePolygon.setConvexPolygon2d(supportPolygonHolder);
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
    	  boolean swingFootHitGround = stateEstimator.isFootInContact(swingQuadrant);
    	  boolean inSwingStateLongEnough = tripleSupportState.getTimeInCurrentState() > swingDuration.getDoubleValue() / 3.0;

    	  if (runOpenLoop.getBooleanValue())
    	  {
        	  swingFootHitGround = false;
    	  }
    	  else
    	  {
        	  swingFootHitGround = stateEstimator.isFootInContact(swingQuadrant);
    	  }
    	  
    	  return ((swingTrajectoryIsDone || swingFootHitGround) && inSwingStateLongEnough);
      }
   }
   
   private class QuadrupleSupportState extends State<CrawlGateWalkingState>
   {
      private final FramePoint swingDesired = new FramePoint();
      private QuadrupedSupportPolygon quadStateAfterFirstStep;
      private QuadrupedSupportPolygon quadStateAfterSecondStep;
      private QuadrupedSupportPolygon quadStateAfterThirdStep;
      private QuadrupedSupportPolygon trippleStateWithFirstStepSwinging;
      private QuadrupedSupportPolygon trippleStateAfterFirstStepWithSecondSwinging;
      private QuadrupedSupportPolygon trippleStateAfterSecondStepWithThirdSwinging;
      private QuadrupedSupportPolygon trippleStateAfterThirdStepWithFourthSwinging;
      
      private final QuadrantDependentList<QuadrupedSupportPolygon> estimatedCommonTriangle = new QuadrantDependentList<>();
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
         processVelocityChanges();
         
         if(isTransitioningToSafePosition() && comTrajectoryGenerator.isDone())
         {
            transitioningToSafePosition.set(false);
         }
         
         else if(shouldTranistionToTripleButItsNotSafeToStep())
         {
            shiftCoMToSafeStartingPosition();
         }
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
         centerOfMassFramePoint.getPoint2d(centerOfMassPoint2d);
         
         LineSegment2d lineSegment = new LineSegment2d();
         Point2d comProjectionOnOutsideLegs2d = new Point2d();
         FramePoint comProjectionOnOutsideLegs = new FramePoint(ReferenceFrame.getWorldFrame());
         
         QuadrupedSupportPolygon trippleStateWithoutCurrentSwing = fourFootSupportPolygon.deleteLegCopy(currentSwingLeg);
         
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
            FramePoint centroidFramePoint = trippleStateWithoutCurrentSwing.getCentroidFramePoint();
            centroidFramePoint.getPoint2d(circleCenter2d);
            break;
            
         case COM_INCIRCLE:
            lineSegment.set(diagonalFootstep.getX(), diagonalFootstep.getY(), acrossBodyFootstep.getX(), acrossBodyFootstep.getY());
            lineSegment.getClosestPointOnLineSegment(comProjectionOnOutsideLegs2d, centerOfMassPoint2d);
            comProjectionOnOutsideLegs.setXY(comProjectionOnOutsideLegs2d);
            
            safeToStepSupportPolygon.clear();
            safeToStepSupportPolygon.setFootstep(currentSwingLeg, centerOfMassFramePoint);
            safeToStepSupportPolygon.setFootstep(diagonalQuadrant, fourFootSupportPolygon.getFootstep(diagonalQuadrant));
            safeToStepSupportPolygon.setFootstep(acrossBodyQuadrant, comProjectionOnOutsideLegs);
            safeToStepSupportPolygon.getInCircle(circleCenter2d);
            break;
            
         case TTR:
            trippleStateWithoutCurrentSwing.getTangentTangentRadiusCircleCenter(currentSwingLeg.getAcrossBodyQuadrant(), 0.1, circleCenter2d);
            break;
            
         case TROTLINE_MIDPOINT:
            lineSegment.set(sameSideFootstep.getX(), sameSideFootstep.getY(), acrossBodyFootstep.getX(), acrossBodyFootstep.getY());
            
            Point2d midpoint = lineSegment.midpoint();
            double bisectorLengthDesired = 0.1;
            Vector2d perpendicularBisector = new Vector2d();
            lineSegment.getPerpendicularBisector(perpendicularBisector, bisectorLengthDesired);
            circleCenter2d.add(midpoint, perpendicularBisector);
            if(!trippleStateWithoutCurrentSwing.isInside(circleCenter2d))
            {
               perpendicularBisector.scale(-1.0);
               circleCenter2d.add(midpoint, perpendicularBisector);
            }
            break;
         }
         
         /**
          * something went wrong! 
          */
         if(!trippleStateWithoutCurrentSwing.isInside(circleCenter2d))
         {
            System.err.println(safeToShiftMode + " tried to shift outside of the support polygon. Fix this");
            FramePoint centroidFramePoint = trippleStateWithoutCurrentSwing.getCentroidFramePoint();
            centroidFramePoint.getPoint2d(circleCenter2d);
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
         if(!isDesiredVelocityAndYawRateZero() && !isDesiredVelocityReversing())
         {
            swingLegUpdatedOnTransition.set(true);
            calculateNextSwingLeg();
            calculateNextCoMTarget(false);
         }
      }
      
      public void calculateNextThreeFootSteps(RobotQuadrant firstSwingLeg)
      {
         RobotQuadrant secondSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, firstSwingLeg, desiredBodyVelocity, desiredYawRate.getDoubleValue());//firstSwingLeg.getNextRegularGaitSwingQuadrant();
         RobotQuadrant thirdSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, secondSwingLeg, desiredBodyVelocity, desiredYawRate.getDoubleValue());//secondSwingLeg.getNextRegularGaitSwingQuadrant();
         RobotQuadrant fourthSwingLeg = nextSwingLegChooser.chooseNextSwingLeg(fourFootSupportPolygon, thirdSwingLeg, desiredBodyVelocity, desiredYawRate.getDoubleValue());//thirdSwingLeg.getNextRegularGaitSwingQuadrant();
         
         FrameVector desiredVelocityVector = desiredVelocity.getFrameTuple();
         double yawRate = desiredYawRate.getDoubleValue();
         
         swingDesired.changeFrame(ReferenceFrame.getWorldFrame());
         calculateSwingTarget(firstSwingLeg, swingDesired);
         quadStateAfterFirstStep = fourFootSupportPolygon.replaceFootstepCopy(firstSwingLeg, swingDesired);
         
         swingTargetGenerator.getSwingTarget(quadStateAfterFirstStep, secondSwingLeg, desiredVelocityVector, swingDesired, yawRate);
         quadStateAfterSecondStep = quadStateAfterFirstStep.replaceFootstepCopy(secondSwingLeg, swingDesired);
         
         swingTargetGenerator.getSwingTarget(quadStateAfterSecondStep, thirdSwingLeg, desiredVelocityVector, swingDesired, yawRate);
         quadStateAfterThirdStep = quadStateAfterSecondStep.replaceFootstepCopy(thirdSwingLeg, swingDesired);
         
         trippleStateWithFirstStepSwinging = quadStateAfterFirstStep.deleteLegCopy(secondSwingLeg);
         trippleStateAfterFirstStepWithSecondSwinging = quadStateAfterFirstStep.deleteLegCopy(secondSwingLeg);
         trippleStateAfterSecondStepWithThirdSwinging = quadStateAfterSecondStep.deleteLegCopy(thirdSwingLeg);
         trippleStateAfterThirdStepWithFourthSwinging = quadStateAfterThirdStep.deleteLegCopy(fourthSwingLeg);
         
         drawSupportPolygon(trippleStateWithFirstStepSwinging, tripleSupportPolygons[0]);
         drawSupportPolygon(trippleStateAfterFirstStepWithSecondSwinging, tripleSupportPolygons[1]);
         QuadrupedSupportPolygon firstAndSecondCommonPolygon = trippleStateWithFirstStepSwinging.getShrunkenCommonSupportPolygon(trippleStateAfterFirstStepWithSecondSwinging,
               firstSwingLeg, shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue());
         if(firstAndSecondCommonPolygon != null)
         {
            estimatedCommonTriangle.put(firstSwingLeg, firstAndSecondCommonPolygon);
            estimatedCommonTriangle.put(firstSwingLeg.getSameSideQuadrant(), firstAndSecondCommonPolygon.swapSameSideFeetCopy(firstSwingLeg));
            drawSupportPolygon(firstAndSecondCommonPolygon, commonTriplePolygons.get(firstSwingLeg.getSide()));
         }
         
         drawSupportPolygon(trippleStateAfterFirstStepWithSecondSwinging, tripleSupportPolygons[2]);
         drawSupportPolygon(trippleStateAfterSecondStepWithThirdSwinging, tripleSupportPolygons[3]);
         QuadrupedSupportPolygon secondAndThirdCommonPolygon = trippleStateAfterFirstStepWithSecondSwinging.getShrunkenCommonSupportPolygon(
               trippleStateAfterSecondStepWithThirdSwinging, secondSwingLeg, shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue());
         if(secondAndThirdCommonPolygon != null)
         {
            estimatedCommonTriangle.put(secondSwingLeg, secondAndThirdCommonPolygon);
            estimatedCommonTriangle.put(secondSwingLeg.getSameSideQuadrant(), secondAndThirdCommonPolygon.swapSameSideFeetCopy(secondSwingLeg));
            drawSupportPolygon(secondAndThirdCommonPolygon, commonTriplePolygons.get(secondSwingLeg.getSide()));
         }
         
         drawSupportPolygon(trippleStateAfterSecondStepWithThirdSwinging, tripleSupportPolygons[2]);
         drawSupportPolygon(trippleStateAfterThirdStepWithFourthSwinging, tripleSupportPolygons[3]);
         QuadrupedSupportPolygon thirdAndFourthCommonPolygon = trippleStateAfterSecondStepWithThirdSwinging.getShrunkenCommonSupportPolygon(
               trippleStateAfterThirdStepWithFourthSwinging, thirdSwingLeg, shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue(), shrunkenPolygonSize.getDoubleValue());
         if(thirdAndFourthCommonPolygon != null)
         {
            estimatedCommonTriangle.put(thirdSwingLeg, thirdAndFourthCommonPolygon);
            estimatedCommonTriangle.put(thirdSwingLeg.getSameSideQuadrant(), thirdAndFourthCommonPolygon.swapSameSideFeetCopy(thirdSwingLeg));
            drawSupportPolygon(thirdAndFourthCommonPolygon, commonTriplePolygons.get(thirdSwingLeg.getSide()));
         }
      }
       
      private final FrameVector2d tempFrameVector = new FrameVector2d();
      
      private void calculateTrajectoryTarget(RobotQuadrant upcommingSwingLeg, QuadrupedSupportPolygon commonTriangle, Point2d comTargetToPack)
      {
         commonSupportPolygon.set(commonTriangle);
         boolean ttrCircleSuccess = false;
         double radius = subCircleRadius.getDoubleValue();
         if(useSubCircleForBodyShiftTarget.getBooleanValue())
         {
            ttrCircleSuccess = commonSupportPolygon.getTangentTangentRadiusCircleCenter(upcommingSwingLeg, radius, comTargetToPack);
         }
         
         if(!ttrCircleSuccess)
         {
            radius = commonSupportPolygon.getInCircle(comTargetToPack);
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
         initializeCoMTrajectory(new Point2d(finalPosition.getX(), finalPosition.getY()));
      }
      
      public void setCoMTrajectoryToCurrentAndSetToDone()
      {
         desiredCoMVelocity.setToZero();
         FramePoint desiredBodyCurrent = desiredCoMPose.getPosition().getFramePointCopy();
         initialCoMPosition.set(desiredBodyCurrent);
         initializeCoMTrajectory(new Point2d(initialCoMPosition.getX(), initialCoMPosition.getY()));
         comTrajectoryGenerator.setToDone();
      }
      
      private void initializeCoMTrajectory(Point2d target)
      {
         FramePoint desiredBodyCurrent = desiredCoMPose.getPosition().getFramePointCopy();
         initialCoMPosition.set(desiredBodyCurrent);
         
         FrameVector desiredBodyVelocityCurrent = desiredCoMVelocity.getFrameVectorCopy();
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
            comTrajectoryDuration.set(1.0);
         }
         
         if (comTrajectoryDuration.getDoubleValue() > maximumCoMTrajectoryDuration.getDoubleValue()) comTrajectoryDuration.set(maximumCoMTrajectoryDuration.getDoubleValue());
         if (comTrajectoryDuration.getDoubleValue() < minimumCoMTrajectoryDuration.getDoubleValue()) comTrajectoryDuration.set(minimumCoMTrajectoryDuration.getDoubleValue());
         
         comTrajectoryGenerator.setTrajectoryTime(comTrajectoryDuration.getDoubleValue());

         desiredBodyVelocity.changeFrame(ReferenceFrame.getWorldFrame());
         
         comTrajectoryTimeStart.set(robotTimestamp.getDoubleValue());
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
         return estimatedCommonTriangle.get(swingLeg.getSide()) == null;
      }
      
      public boolean isCoMInsideCommonTriangleForSwingLeg(RobotQuadrant swingLeg)
      {
         if(isCommonTriangleNull(swingLeg))
         {
            return false;
         }
         centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         centerOfMassFramePoint.getPoint2d(centerOfMassPoint2d);
        
         return estimatedCommonTriangle.get(swingLeg.getSide()).isInside(centerOfMassPoint2d);
      }
      
      /**
       * uses actuals
       */
      public boolean isCoMInsideTriangleForSwingLeg(RobotQuadrant swingLeg)
      {
         centerOfMassFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
         centerOfMassFramePoint.getPoint2d(centerOfMassPoint2d);
         QuadrupedSupportPolygon supportTriangleDuringStep = fourFootSupportPolygon.deleteLegCopy(swingLeg);
         return supportTriangleDuringStep.isInside(centerOfMassPoint2d);
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
         speedMatchScalar.set(-1.0);
      }

      @Override
      public void doAction()
      {
         RobotQuadrant swingQuadrant = swingLeg.getEnumValue();
         
         computeFootPositionAlongSwingTrajectory(swingQuadrant, currentDesiredInTrajectory);
         currentDesiredInTrajectory.changeFrame(ReferenceFrame.getWorldFrame());
         currentSwingTarget.set(currentDesiredInTrajectory);
         
         desiredFeetLocations.get(swingQuadrant).setAndMatchFrame(currentDesiredInTrajectory);
      }

      @Override
      public void doTransitionIntoAction()
      {        
         RobotQuadrant swingQuadrant = swingLeg.getEnumValue();
         swingTarget.setToZero(ReferenceFrame.getWorldFrame());
         calculateSwingTarget(swingQuadrant, swingTarget);
         
         YoFramePoint yoDesiredFootPosition = desiredFeetLocations.get(swingQuadrant);
         currentSwingTarget.set(swingTarget);
         finalSwingTarget.set(swingTarget);
         
         initializeSwingTrajectory(swingQuadrant, yoDesiredFootPosition.getFramePointCopy(), swingTarget, swingDuration.getDoubleValue());
      }

      @Override
      public void doTransitionOutOfAction()
      {

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
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void doTransitionIntoAction()
   {
      for(OneDoFJoint oneDofJoint : fullRobotModel.getOneDoFJoints())
      {
         oneDofJoint.setUnderPositionControl(true);
      }

      stateEstimator.initialize();
      stateEstimator.enable();
      
      fullRobotModel.updateFrames();
      referenceFrames.updateFrames();

      updateEstimates();
      updateFeedForwardModelAndFrames();
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
