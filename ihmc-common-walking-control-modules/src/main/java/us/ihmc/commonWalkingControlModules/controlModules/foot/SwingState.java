package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith.FootLeapOfFaithModule;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.math.filters.RateLimitedYoFramePose;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsBlendedPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.trajectorypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.robotics.trajectories.providers.CurrentRigidBodyStateProvider;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class SwingState extends AbstractFootControlState
{
   private static final boolean USE_ALL_LEG_JOINT_SWING_CORRECTOR = false;

   private final YoBoolean replanTrajectory;
   private final YoBoolean footstepWasAdjusted;

   private static final double maxScalingFactor = 1.5;
   private static final double minScalingFactor = 0.1;
   private static final double exponentialScalingRate = 5.0;

   private final YoEnum<TrajectoryType> activeTrajectoryType;

   private final TwoWaypointSwingGenerator swingTrajectoryOptimizer;
   private final MultipleWaypointsBlendedPoseTrajectoryGenerator blendedSwingTrajectory;
   private final SoftTouchdownPoseTrajectoryGenerator touchdownTrajectory;
   private double swingTrajectoryBlendDuration = 0.0;

   private final CurrentRigidBodyStateProvider currentStateProvider;

   private final FootLeapOfFaithModule leapOfFaithModule;

   private final FrameVector3DReadOnly touchdownAcceleration;
   private final FrameVector3DReadOnly touchdownVelocity;

   private final ReferenceFrame oppositeSoleFrame;
   private final ReferenceFrame oppositeSoleZUpFrame;

   private final FramePose3D initialPose = new FramePose3D();
   private final FramePoint3D initialPosition = new FramePoint3D();
   private final FrameVector3D initialLinearVelocity = new FrameVector3D();
   private final FrameQuaternion initialOrientation = new FrameQuaternion();
   private final FrameVector3D initialAngularVelocity = new FrameVector3D();
   private final FramePoint3D finalPosition = new FramePoint3D();
   private final FrameVector3D finalLinearVelocity = new FrameVector3D();
   private final FrameQuaternion finalOrientation = new FrameQuaternion();
   private final FrameVector3D finalAngularVelocity = new FrameVector3D();
   private final FramePoint3D stanceFootPosition = new FramePoint3D();

   private final FrameQuaternion tmpOrientation = new FrameQuaternion();
   private final FrameVector3D tmpVector = new FrameVector3D();

   private final double[] waypointProportions = new double[2];
   private final List<DoubleProvider> defaultWaypointProportions = new ArrayList<>();
   private final List<DoubleProvider> defaultObstacleClearanceWaypointProportions = new ArrayList<>();

   private final RecyclingArrayList<FramePoint3D> positionWaypointsForSole;
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> swingWaypoints;
   private final List<FixedFramePoint3DBasics> swingWaypointsForViz = new ArrayList<>();
   private final FramePoint3D tempWaypoint = new FramePoint3D();

   private final YoDouble swingDuration;
   private final YoDouble swingHeight;

   private final YoDouble swingTimeSpeedUpFactor;
   private final YoDouble maxSwingTimeSpeedUpFactor;
   private final YoDouble minSwingTimeForDisturbanceRecovery;
   private final YoBoolean isSwingSpeedUpEnabled;
   private final YoDouble currentTime;
   private final YoDouble currentTimeWithSwingSpeedUp;

   private final BooleanParameter doHeelTouchdownIfPossible;
   private final DoubleParameter heelTouchdownAngle;
   private final DoubleParameter maximumHeightForHeelTouchdown;
   private final DoubleParameter heelTouchdownLengthRatio;

   private final BooleanParameter doToeTouchdownIfPossible;
   private final DoubleParameter toeTouchdownAngle;
   private final DoubleParameter stepDownHeightForToeTouchdown;
   private final DoubleParameter toeTouchdownDepthRatio;

   private final BooleanParameter addOrientationMidpointForClearance;
   private final DoubleParameter midpointOrientationInterpolationForClearance;

   private final YoBoolean ignoreInitialAngularVelocityZ;
   private final YoDouble maxInitialLinearVelocityMagnitude;
   private final YoDouble maxInitialAngularVelocityMagnitude;

   private final DoubleParameter finalSwingHeightOffset;
   private final double controlDT;

   private final DoubleParameter minHeightDifferenceForObstacleClearance;

   private final MovingReferenceFrame soleFrame;

   private final PoseReferenceFrame desiredSoleFrame = new PoseReferenceFrame("desiredSoleFrame", worldFrame);
   private final PoseReferenceFrame desiredControlFrame = new PoseReferenceFrame("desiredControlFrame", desiredSoleFrame);
   private final FramePose3D desiredPose = new FramePose3D();
   private final Twist desiredTwist = new Twist();
   private final SpatialAcceleration desiredSpatialAcceleration = new SpatialAcceleration();

   private final RigidBodyTransform transformFromToeToAnkle = new RigidBodyTransform();

   private final DoubleParameter velocityAdjustmentDamping;
   private final YoFrameVector3D adjustmentVelocityCorrection;
   private final FramePoint3D unadjustedPosition = new FramePoint3D(worldFrame);

   private final FramePose3D adjustedFootstepPose = new FramePose3D();
   private final RateLimitedYoFramePose rateLimitedAdjustedPose;

   private final FramePose3D footstepPose = new FramePose3D();
   private final FramePose3D lastFootstepPose = new FramePose3D();

   private final FrameEuclideanTrajectoryPoint tempPositionTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

   // for unit testing and debugging:
   private final YoInteger currentTrajectoryWaypoint;
   private final YoFramePoint3D yoDesiredSolePosition;
   private final YoFrameQuaternion yoDesiredSoleOrientation;
   private final YoFrameVector3D yoDesiredSoleLinearVelocity;
   private final YoFrameVector3D yoDesiredSoleAngularVelocity;
   private final SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
   private final LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule;
   private final LegJointLimitAvoidanceControlModule legJointLimitAvoidanceControlModule;
   private final YoFramePoint3D yoDesiredPosition;
   private final YoFrameVector3D yoDesiredLinearVelocity;
   private final YoBoolean yoSetDesiredAccelerationToZero;
   private final YoBoolean yoSetDesiredVelocityToZero;
   private final YoBoolean scaleSecondaryJointWeights;
   private final YoDouble secondaryJointWeightScale;
   private Vector3DReadOnly nominalAngularWeight;
   private Vector3DReadOnly nominalLinearWeight;
   private final YoFrameVector3D currentAngularWeight;
   private final YoFrameVector3D currentLinearWeight;
   private final ReferenceFrame ankleFrame;
   private final PoseReferenceFrame controlFrame;
   private final PIDSE3GainsReadOnly gains;
   private final FramePoint3D desiredAnklePosition = new FramePoint3D();
   private final RigidBodyTransform oldBodyFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform newBodyFrameDesiredTransform = new RigidBodyTransform();
   private final RigidBodyTransform transformFromNewBodyFrameToOldBodyFrame = new RigidBodyTransform();

   public SwingState(FootControlHelper footControlHelper, FrameVector3DReadOnly touchdownVelocity, FrameVector3DReadOnly touchdownAcceleration,
                     PIDSE3GainsReadOnly gains, YoVariableRegistry registry)
   {
      super(footControlHelper);
      this.gains = gains;

      this.legSingularityAndKneeCollapseAvoidanceControlModule = footControlHelper.getLegSingularityAndKneeCollapseAvoidanceControlModule();

      RigidBodyBasics foot = contactableFoot.getRigidBody();
      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "FootSwing";
      yoDesiredLinearVelocity = new YoFrameVector3D(namePrefix + "DesiredLinearVelocity", worldFrame, registry);
      yoDesiredLinearVelocity.setToNaN();
      yoDesiredPosition = new YoFramePoint3D(namePrefix + "DesiredPosition", worldFrame, registry);
      yoDesiredPosition.setToNaN();
      yoSetDesiredAccelerationToZero = new YoBoolean(namePrefix + "SetDesiredAccelerationToZero", registry);
      yoSetDesiredVelocityToZero = new YoBoolean(namePrefix + "SetDesiredVelocityToZero", registry);

      scaleSecondaryJointWeights = new YoBoolean(namePrefix + "ScaleSecondaryJointWeights", registry);
      secondaryJointWeightScale = new YoDouble(namePrefix + "SecondaryJointWeightScale", registry);
      secondaryJointWeightScale.set(1.0);

      currentAngularWeight = new YoFrameVector3D(namePrefix + "CurrentAngularWeight", worldFrame, registry);
      currentLinearWeight = new YoFrameVector3D(namePrefix + "CurrentLinearWeight", worldFrame, registry);

      if (USE_ALL_LEG_JOINT_SWING_CORRECTOR)
         legJointLimitAvoidanceControlModule = new LegJointLimitAvoidanceControlModule(namePrefix, registry, controllerToolbox, robotSide);
      else
         legJointLimitAvoidanceControlModule = null;

      ankleFrame = contactableFoot.getFrameAfterParentJoint();
      controlFrame = new PoseReferenceFrame("controlFrame", contactableFoot.getRigidBody().getBodyFixedFrame());

      spatialFeedbackControlCommand.set(rootBody, foot);
      spatialFeedbackControlCommand.setPrimaryBase(pelvis);
      ReferenceFrame linearGainsFrame = footControlHelper.getHighLevelHumanoidControllerToolbox().getPelvisZUpFrame();
      spatialFeedbackControlCommand.setGainsFrames(null, linearGainsFrame);
      FramePose3D anklePoseInFoot = new FramePose3D(ankleFrame);
      anklePoseInFoot.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      changeControlFrame(anklePoseInFoot);

      this.touchdownAcceleration = touchdownAcceleration;
      this.touchdownVelocity = touchdownVelocity;

      controlDT = footControlHelper.getHighLevelHumanoidControllerToolbox().getControlDT();

      swingWaypoints = new RecyclingArrayList<>(Footstep.maxNumberOfSwingWaypoints, FrameSE3TrajectoryPoint.class);
      positionWaypointsForSole = new RecyclingArrayList<>(2, FramePoint3D.class);

      WalkingControllerParameters walkingControllerParameters = footControlHelper.getWalkingControllerParameters();
      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();

      finalSwingHeightOffset = new DoubleParameter(namePrefix + "FinalHeightOffset", registry, swingTrajectoryParameters.getDesiredTouchdownHeightOffset());
      //finalSwingHeightOffset.set(swingTrajectoryParameters.getDesiredTouchdownHeightOffset());
      replanTrajectory = new YoBoolean(namePrefix + "ReplanTrajectory", registry);
      footstepWasAdjusted = new YoBoolean(namePrefix + "FootstepWasAdjusted", registry);

      minHeightDifferenceForObstacleClearance = new DoubleParameter(namePrefix + "MinHeightDifferenceForObstacleClearance", registry,
                                                                    swingTrajectoryParameters.getMinHeightDifferenceForStepUpOrDown());

      velocityAdjustmentDamping = new DoubleParameter(namePrefix + "VelocityAdjustmentDamping", registry,
                                                      swingTrajectoryParameters.getSwingFootVelocityAdjustmentDamping());
      adjustmentVelocityCorrection = new YoFrameVector3D(namePrefix + "AdjustmentVelocityCorrection", worldFrame, registry);

      doHeelTouchdownIfPossible = new BooleanParameter(namePrefix + "DoHeelTouchdownIfPossible", registry,
                                                       swingTrajectoryParameters.doHeelTouchdownIfPossible());
      heelTouchdownAngle = new DoubleParameter(namePrefix + "HeelTouchdownAngle", registry, swingTrajectoryParameters.getHeelTouchdownAngle());
      maximumHeightForHeelTouchdown = new DoubleParameter(namePrefix + "MaximumHeightForHeelTouchdown", registry,
                                                          swingTrajectoryParameters.getMaximumHeightForHeelTouchdown());
      heelTouchdownLengthRatio = new DoubleParameter(namePrefix + "HeelTouchdownLengthRatio", registry,
                                                     swingTrajectoryParameters.getHeelTouchdownLengthRatio());

      rateLimitedAdjustedPose = new RateLimitedYoFramePose(namePrefix + "AdjustedFootstepPose", "", registry, 10.0, controlDT, worldFrame);

      doToeTouchdownIfPossible = new BooleanParameter(namePrefix + "DoToeTouchdownIfPossible", registry, swingTrajectoryParameters.doToeTouchdownIfPossible());
      toeTouchdownAngle = new DoubleParameter(namePrefix + "ToeTouchdownAngle", registry, swingTrajectoryParameters.getToeTouchdownAngle());
      stepDownHeightForToeTouchdown = new DoubleParameter(namePrefix + "StepDownHeightForToeTouchdown", registry,
                                                          swingTrajectoryParameters.getStepDownHeightForToeTouchdown());
      toeTouchdownDepthRatio = new DoubleParameter(namePrefix + "ToeTouchdownDepthRatio", registry, swingTrajectoryParameters.getToeTouchdownDepthRatio());

      addOrientationMidpointForClearance = new BooleanParameter(namePrefix + "AddOrientationMidpointForClearance", registry,
                                                                swingTrajectoryParameters.addOrientationMidpointForObstacleClearance());
      midpointOrientationInterpolationForClearance = new DoubleParameter(namePrefix + "MidpointOrientationInterpolationForClearance", registry,
                                                                         swingTrajectoryParameters.midpointOrientationInterpolationForObstacleClearance());

      int numberWaypoints = 2;
      double[] defaultWaypointProportions = swingTrajectoryParameters.getSwingWaypointProportions();
      double[] defaultObstacleClearanceWaypointProportions = swingTrajectoryParameters.getObstacleClearanceProportions();

      for (int i = 0; i < numberWaypoints; i++)
      {
         DoubleParameter waypointProportion = new DoubleParameter(namePrefix + "WaypointProportion" + i, registry, defaultWaypointProportions[i]);
         DoubleParameter obstacleClearanceWaypointProportion = new DoubleParameter(namePrefix + "ObstacleClearanceWaypointProportion" + i, registry,
                                                                                   defaultObstacleClearanceWaypointProportions[i]);
         this.defaultWaypointProportions.add(waypointProportion);
         this.defaultObstacleClearanceWaypointProportions.add(obstacleClearanceWaypointProportion);
      }

      ignoreInitialAngularVelocityZ = new YoBoolean(namePrefix + "IgnoreInitialAngularVelocityZ", registry);
      maxInitialLinearVelocityMagnitude = new YoDouble(namePrefix + "MaxInitialLinearVelocityMagnitude", registry);
      maxInitialAngularVelocityMagnitude = new YoDouble(namePrefix + "MaxInitialAngularVelocityMagnitude", registry);
      ignoreInitialAngularVelocityZ.set(walkingControllerParameters.ignoreSwingInitialAngularVelocityZ());
      maxInitialLinearVelocityMagnitude.set(walkingControllerParameters.getMaxSwingInitialLinearVelocityMagnitude());
      maxInitialAngularVelocityMagnitude.set(walkingControllerParameters.getMaxSwingInitialAngularVelocityMagnitude());

      soleFrame = footControlHelper.getHighLevelHumanoidControllerToolbox().getReferenceFrames().getSoleFrame(robotSide);
      ReferenceFrame footFrame = contactableFoot.getFrameAfterParentJoint();
      ReferenceFrame toeFrame = createToeFrame(robotSide);
      ReferenceFrame controlFrame = walkingControllerParameters.controlToeDuringSwing() ? toeFrame : footFrame;
      RigidBodyTransform soleToControlFrameTransform = new RigidBodyTransform();
      controlFrame.getTransformToDesiredFrame(soleToControlFrameTransform, soleFrame);
      desiredControlFrame.setPoseAndUpdate(soleToControlFrameTransform);

      oppositeSoleFrame = controllerToolbox.getReferenceFrames().getSoleFrame(robotSide.getOppositeSide());
      oppositeSoleZUpFrame = controllerToolbox.getReferenceFrames().getSoleZUpFrame(robotSide.getOppositeSide());

      double maxSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getMaxSwingHeightFromStanceFoot();
      double minSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getMinSwingHeightFromStanceFoot();
      double defaultSwingHeightFromStanceFoot = walkingControllerParameters.getSteppingParameters().getDefaultSwingHeightFromStanceFoot();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      swingTrajectoryOptimizer = new TwoWaypointSwingGenerator(namePrefix, minSwingHeightFromStanceFoot, maxSwingHeightFromStanceFoot,
                                                               defaultSwingHeightFromStanceFoot, registry, yoGraphicsListRegistry);

      double minDistanceToStance = walkingControllerParameters.getMinSwingTrajectoryClearanceFromStanceFoot();
      swingTrajectoryOptimizer.enableStanceCollisionAvoidance(robotSide, oppositeSoleZUpFrame, minDistanceToStance);

      MultipleWaypointsPoseTrajectoryGenerator swingTrajectory = new MultipleWaypointsPoseTrajectoryGenerator(namePrefix,
                                                                                                              Footstep.maxNumberOfSwingWaypoints + 2, registry);
      blendedSwingTrajectory = new MultipleWaypointsBlendedPoseTrajectoryGenerator(namePrefix, swingTrajectory, worldFrame, registry);
      touchdownTrajectory = new SoftTouchdownPoseTrajectoryGenerator(namePrefix + "Touchdown", registry);
      currentStateProvider = new CurrentRigidBodyStateProvider(soleFrame);

      activeTrajectoryType = new YoEnum<>(namePrefix + TrajectoryType.class.getSimpleName(), registry, TrajectoryType.class);
      swingDuration = new YoDouble(namePrefix + "Duration", registry);
      swingHeight = new YoDouble(namePrefix + "Height", registry);

      swingTimeSpeedUpFactor = new YoDouble(namePrefix + "TimeSpeedUpFactor", registry);
      minSwingTimeForDisturbanceRecovery = new YoDouble(namePrefix + "MinTimeForDisturbanceRecovery", registry);
      minSwingTimeForDisturbanceRecovery.set(walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery());
      maxSwingTimeSpeedUpFactor = new YoDouble(namePrefix + "MaxTimeSpeedUpFactor", registry);
      currentTime = new YoDouble(namePrefix + "CurrentTime", registry);
      currentTimeWithSwingSpeedUp = new YoDouble(namePrefix + "CurrentTimeWithSpeedUp", registry);
      isSwingSpeedUpEnabled = new YoBoolean(namePrefix + "IsSpeedUpEnabled", registry);
      isSwingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());

      swingTimeSpeedUpFactor.setToNaN();

      scaleSecondaryJointWeights.set(walkingControllerParameters.applySecondaryJointScaleDuringSwing());

      LeapOfFaithParameters leapOfFaithParameters = walkingControllerParameters.getLeapOfFaithParameters();
      leapOfFaithModule = new FootLeapOfFaithModule(swingDuration, leapOfFaithParameters, registry);

      FramePose3D controlFramePose = new FramePose3D(controlFrame);
      controlFramePose.changeFrame(contactableFoot.getRigidBody().getBodyFixedFrame());
      changeControlFrame(controlFramePose);

      lastFootstepPose.setToNaN();
      footstepPose.setToNaN();
      footstepWasAdjusted.set(false);

      currentTrajectoryWaypoint = new YoInteger(namePrefix + "CurrentTrajectoryWaypoint", registry);
      yoDesiredSolePosition = new YoFramePoint3D(namePrefix + "DesiredSolePositionInWorld", worldFrame, registry);
      yoDesiredSoleOrientation = new YoFrameQuaternion(namePrefix + "DesiredSoleOrientationInWorld", worldFrame, registry);
      yoDesiredSoleLinearVelocity = new YoFrameVector3D(namePrefix + "DesiredSoleLinearVelocityInWorld", worldFrame, registry);
      yoDesiredSoleAngularVelocity = new YoFrameVector3D(namePrefix + "DesiredSoleAngularVelocityInWorld", worldFrame, registry);

      setupViz(yoGraphicsListRegistry, registry);
   }

   private void setupViz(YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      if (yoGraphicsListRegistry == null)
      {
         return;
      }

      for (int i = 0; i < Footstep.maxNumberOfSwingWaypoints; i++)
      {
         YoFramePoint3D yoWaypoint = new YoFramePoint3D("SwingWaypoint" + robotSide.getPascalCaseName() + i, ReferenceFrame.getWorldFrame(), registry);
         YoGraphicPosition waypointViz = new YoGraphicPosition("SwingWaypoint" + robotSide.getPascalCaseName() + i, yoWaypoint , 0.01, YoAppearance.GreenYellow());
         yoWaypoint.setToNaN();
         yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), waypointViz);
         swingWaypointsForViz.add(yoWaypoint);
      }
   }

   private ReferenceFrame createToeFrame(RobotSide robotSide)
   {
      ContactableFoot contactableFoot = controllerToolbox.getContactableFeet().get(robotSide);
      ReferenceFrame footFrame = controllerToolbox.getReferenceFrames().getFootFrame(robotSide);
      FramePoint2D toeContactPoint2d = new FramePoint2D();
      contactableFoot.getToeOffContactPoint(toeContactPoint2d);
      FramePoint3D toeContactPoint = new FramePoint3D();
      toeContactPoint.setIncludingFrame(toeContactPoint2d, 0.0);
      toeContactPoint.changeFrame(footFrame);

      transformFromToeToAnkle.setTranslation(toeContactPoint);
      return ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression() + "ToeFrame", footFrame,
                                                                               transformFromToeToAnkle);
   }

   private void initializeTrajectory()
   {
      currentStateProvider.getPosition(initialPosition);
      currentStateProvider.getLinearVelocity(initialLinearVelocity);
      currentStateProvider.getOrientation(initialOrientation);
      currentStateProvider.getAngularVelocity(initialAngularVelocity);
      initialPose.changeFrame(initialPosition.getReferenceFrame());
      initialPose.setPosition(initialPosition);
      initialPose.changeFrame(initialOrientation.getReferenceFrame());
      initialPose.setOrientation(initialOrientation);
      if (ignoreInitialAngularVelocityZ.getBooleanValue())
      {
         initialAngularVelocity.changeFrame(worldFrame);
         initialAngularVelocity.setZ(0.0);
      }
      initialLinearVelocity.clipToMaxLength(maxInitialLinearVelocityMagnitude.getDoubleValue());
      initialAngularVelocity.clipToMaxLength(maxInitialAngularVelocityMagnitude.getDoubleValue());
      stanceFootPosition.setToZero(oppositeSoleFrame);

      fillAndInitializeTrajectories(true);
   }

   @Override
   public void onEntry()
   {
      super.onEntry();
      currentTime.set(0.0);
      swingTimeSpeedUpFactor.set(1.0);
      currentTimeWithSwingSpeedUp.set(Double.NaN);
      replanTrajectory.set(false);

      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         legSingularityAndKneeCollapseAvoidanceControlModule.setCheckVelocityForSwingSingularityAvoidance(true);
      }

      YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
      contactState.notifyContactStateHasChanged();

      spatialFeedbackControlCommand.resetSecondaryTaskJointWeightScale();

      initializeTrajectory();
   }

   @Override
   public void onExit()
   {
      super.onExit();
      currentTime.set(0.0);
      swingTimeSpeedUpFactor.set(Double.NaN);
      currentTimeWithSwingSpeedUp.set(Double.NaN);

      swingTrajectoryOptimizer.informDone();

      adjustmentVelocityCorrection.setToZero();

      yoDesiredSolePosition.setToNaN();
      yoDesiredSoleOrientation.setToNaN();
      yoDesiredSoleLinearVelocity.setToNaN();
      yoDesiredSoleAngularVelocity.setToNaN();
      yoDesiredPosition.setToNaN();
      yoDesiredLinearVelocity.setToNaN();

      footstepWasAdjusted.set(false);

      for (int i = 0; i < swingWaypointsForViz.size(); i++)
      {
         swingWaypointsForViz.get(i).setToNaN();
      }
   }

   @Override
   public void doSpecificAction(double timeInState)
   {
      computeAndPackTrajectory(timeInState);

      if (USE_ALL_LEG_JOINT_SWING_CORRECTOR)
      {
         legJointLimitAvoidanceControlModule.correctSwingFootTrajectory(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
                                                                        desiredLinearAcceleration, desiredAngularAcceleration);
      }

      if (legSingularityAndKneeCollapseAvoidanceControlModule != null)
      {
         desiredPose.setIncludingFrame(desiredPosition, desiredOrientation);
         changeDesiredPoseBodyFrame(controlFrame, ankleFrame, desiredPose);
         desiredAnklePosition.setIncludingFrame(desiredPose.getPosition());

         legSingularityAndKneeCollapseAvoidanceControlModule.correctSwingFootTrajectory(desiredAnklePosition, desiredLinearVelocity, desiredLinearAcceleration);

         desiredPose.setPosition(desiredAnklePosition);
         changeDesiredPoseBodyFrame(ankleFrame, controlFrame, desiredPose);
         desiredPosition.setIncludingFrame(desiredPose.getPosition());
      }

      if (yoSetDesiredVelocityToZero.getBooleanValue())
      {
         desiredLinearVelocity.setToZero();
      }

      if (yoSetDesiredAccelerationToZero.getBooleanValue())
      {
         desiredLinearAcceleration.setToZero();
      }

      computeCurrentWeights(nominalAngularWeight, nominalLinearWeight, currentAngularWeight, currentLinearWeight);

      spatialFeedbackControlCommand.setInverseDynamics(desiredOrientation, desiredPosition, desiredAngularVelocity, desiredLinearVelocity, desiredAngularAcceleration, desiredLinearAcceleration);
      spatialFeedbackControlCommand.setWeightsForSolver(currentAngularWeight, currentLinearWeight);
      spatialFeedbackControlCommand.setScaleSecondaryTaskJointWeight(scaleSecondaryJointWeights.getBooleanValue(), secondaryJointWeightScale.getDoubleValue());
      spatialFeedbackControlCommand.setGains(gains);

      yoDesiredPosition.setMatchingFrame(desiredPosition);
      yoDesiredLinearVelocity.setMatchingFrame(desiredLinearVelocity);
   }

   private void computeAndPackTrajectory(double timeInState)
   {
      currentTime.set(timeInState);

      if (footstepWasAdjusted.getBooleanValue())
      {
         if (!rateLimitedAdjustedPose.geometricallyEquals(adjustedFootstepPose, 1.0e-7))
            replanTrajectory.set(true); // As long as the rate-limited pose has not reached the adjusted pose, we'll have to replan the swing.

         rateLimitedAdjustedPose.update(adjustedFootstepPose);
      }

      double time;
      if (!isSwingSpeedUpEnabled.getBooleanValue() || currentTimeWithSwingSpeedUp.isNaN())
      {
         time = currentTime.getDoubleValue();
      }
      else
      {
         currentTimeWithSwingSpeedUp.add(swingTimeSpeedUpFactor.getDoubleValue() * controlDT);
         time = currentTimeWithSwingSpeedUp.getDoubleValue();
      }

      PoseTrajectoryGenerator activeTrajectory;
      if (time > swingDuration.getDoubleValue())
         activeTrajectory = touchdownTrajectory;
      else
         activeTrajectory = blendedSwingTrajectory;

      boolean footstepWasAdjusted = false;
      if (replanTrajectory.getBooleanValue())
      {
         activeTrajectory.compute(time); // compute to get the current unadjusted position
         activeTrajectory.getPosition(unadjustedPosition);
         footstepWasAdjusted = true;
      }

      if (activeTrajectoryType.getEnumValue() != TrajectoryType.WAYPOINTS && swingTrajectoryOptimizer.doOptimizationUpdate()) // haven't finished original planning
         fillAndInitializeTrajectories(false);
      else if (replanTrajectory.getBooleanValue()) // need to update the beginning and end blending
         fillAndInitializeBlendedTrajectories();
      replanTrajectory.set(false);

      activeTrajectory.compute(time);
      activeTrajectory.getLinearData(desiredPosition, desiredLinearVelocity, desiredLinearAcceleration);
      activeTrajectory.getAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      leapOfFaithModule.compute(time);

      if (footstepWasAdjusted)
      {
         adjustmentVelocityCorrection.set(desiredPosition);
         adjustmentVelocityCorrection.sub(unadjustedPosition);
         adjustmentVelocityCorrection.scale(1.0 / controlDT);
         adjustmentVelocityCorrection.setZ(0.0);
         adjustmentVelocityCorrection.scale(velocityAdjustmentDamping.getValue());

         desiredLinearVelocity.add(adjustmentVelocityCorrection);
      }
      else
      {
         adjustmentVelocityCorrection.setToZero();
      }

      if (isSwingSpeedUpEnabled.getBooleanValue() && !currentTimeWithSwingSpeedUp.isNaN())
      {
         desiredLinearVelocity.scale(swingTimeSpeedUpFactor.getDoubleValue());
         desiredAngularVelocity.scale(swingTimeSpeedUpFactor.getDoubleValue());

         double speedUpFactorSquared = swingTimeSpeedUpFactor.getDoubleValue() * swingTimeSpeedUpFactor.getDoubleValue();
         desiredLinearAcceleration.scale(speedUpFactorSquared);
         desiredAngularAcceleration.scale(speedUpFactorSquared);
      }

      yoDesiredSolePosition.setMatchingFrame(desiredPosition);
      yoDesiredSoleOrientation.setMatchingFrame(desiredOrientation);
      yoDesiredSoleLinearVelocity.setMatchingFrame(desiredLinearVelocity);
      yoDesiredSoleAngularVelocity.setMatchingFrame(desiredAngularVelocity);
      currentTrajectoryWaypoint.set(blendedSwingTrajectory.getCurrentPositionWaypointIndex());

      transformDesiredsFromSoleFrameToControlFrame();
      secondaryJointWeightScale.set(computeSecondaryJointWeightScale(time));
   }

   public void setFootstep(Footstep footstep, double swingTime)
   {
      lastFootstepPose.setIncludingFrame(footstepPose);
      if (lastFootstepPose.containsNaN())
         lastFootstepPose.setToZero(soleFrame);

      setFootstepInternal(footstep);
      setFootstepDurationInternal(swingTime);

      adjustedFootstepPose.set(footstepPose);
      rateLimitedAdjustedPose.set(footstepPose);
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link WalkingControllerParameters#getMinimumSwingTimeForDisturbanceRecovery()}.
    * @param speedUpFactor
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(double speedUpFactor)
   {
      if (isSwingSpeedUpEnabled.getBooleanValue() && (speedUpFactor > 1.1 && speedUpFactor > swingTimeSpeedUpFactor.getDoubleValue()))
      {
         speedUpFactor = MathTools.clamp(speedUpFactor, swingTimeSpeedUpFactor.getDoubleValue(), maxSwingTimeSpeedUpFactor.getDoubleValue());

         swingTimeSpeedUpFactor.set(speedUpFactor);
         if (currentTimeWithSwingSpeedUp.isNaN())
            currentTimeWithSwingSpeedUp.set(currentTime.getDoubleValue());
      }

      return computeSwingTimeRemaining(currentTime.getDoubleValue());
   }

   public void setAdjustedFootstepAndTime(Footstep adjustedFootstep, double swingTime)
   {
      replanTrajectory.set(true);
      footstepWasAdjusted.set(true);

      adjustedFootstep.getPose(adjustedFootstepPose);

      setFootstepDurationInternal(swingTime);
   }

   public void getDesireds(FramePose3D desiredPoseToPack, FrameVector3D desiredLinearVelocityToPack, FrameVector3D desiredAngularVelocityToPack)
   {
      desiredPoseToPack.setIncludingFrame(this.desiredPose);
      desiredAngularVelocityToPack.setIncludingFrame(this.desiredAngularVelocity);
      desiredLinearVelocityToPack.setIncludingFrame(this.desiredLinearVelocity);
   }

   private void fillAndInitializeTrajectories(boolean initializeOptimizer)
   {
      footstepPose.get(finalPosition, finalOrientation);
      finalLinearVelocity.setIncludingFrame(touchdownVelocity);
      finalAngularVelocity.setToZero(worldFrame);

      // append current pose as initial trajectory point
      blendedSwingTrajectory.clearTrajectory(worldFrame);
      boolean appendFootstepPose = true;
      double swingDuration = this.swingDuration.getDoubleValue();

      if (activeTrajectoryType.getEnumValue() == TrajectoryType.WAYPOINTS)
      {
         if (swingWaypoints.get(0).getTime() > 1.0e-5)
         {
            blendedSwingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialLinearVelocity);
            blendedSwingTrajectory.appendOrientationWaypoint(0.0, initialOrientation, initialAngularVelocity);
         }
         else if (swingTrajectoryBlendDuration < 1.0e-5)
         {
            LogTools.warn("Should use blending when providing waypoint at t = 0.0.");
         }

         for (int i = 0; i < swingWaypoints.size(); i++)
         {
            blendedSwingTrajectory.appendPoseWaypoint(swingWaypoints.get(i));
         }

         appendFootstepPose = !Precision.equals(swingWaypoints.getLast().getTime(), swingDuration);
      }
      else
      {
         blendedSwingTrajectory.appendPositionWaypoint(0.0, initialPosition, initialLinearVelocity);
         blendedSwingTrajectory.appendOrientationWaypoint(0.0, initialOrientation, initialAngularVelocity);

         // TODO: initialize optimizer somewhere else
         if (initializeOptimizer)
         {
            initializeOptimizer();
         }

         for (int i = 0; i < swingTrajectoryOptimizer.getNumberOfWaypoints(); i++)
         {
            swingTrajectoryOptimizer.getWaypointData(i, tempPositionTrajectoryPoint);
            blendedSwingTrajectory.appendPositionWaypoint(tempPositionTrajectoryPoint);
         }

         // make the foot orientation better for avoidance
         if (addOrientationMidpointForClearance.getValue() && activeTrajectoryType.getEnumValue() == TrajectoryType.OBSTACLE_CLEARANCE)
         {
            tmpOrientation.setToZero(worldFrame);
            tmpVector.setToZero(worldFrame);
            tmpOrientation.interpolate(initialOrientation, finalOrientation, midpointOrientationInterpolationForClearance.getValue());
            blendedSwingTrajectory.appendOrientationWaypoint(0.5 * swingDuration, tmpOrientation, tmpVector);
         }
      }

      // append footstep pose if not provided in the waypoints
      if (appendFootstepPose)
      {
         modifyFinalOrientationForTouchdown(finalOrientation);
         blendedSwingTrajectory.appendPositionWaypoint(swingDuration, finalPosition, finalLinearVelocity);
         blendedSwingTrajectory.appendOrientationWaypoint(swingDuration, finalOrientation, finalAngularVelocity);
      }
      else
      {
         // In this case our swing trajectory contains the touchdown so we should use those values to be continuous.
         FrameSE3TrajectoryPoint lastPoint = swingWaypoints.getLast();
         lastPoint.getPositionIncludingFrame(finalPosition);
         lastPoint.getLinearVelocityIncludingFrame(finalLinearVelocity);
         lastPoint.getOrientationIncludingFrame(finalOrientation);
      }

      // Setup touchdown trajectory.
      touchdownTrajectory.setLinearTrajectory(swingDuration, finalPosition, finalLinearVelocity, touchdownAcceleration);
      touchdownTrajectory.setOrientation(finalOrientation);

      blendedSwingTrajectory.initializeTrajectory();
      fillAndInitializeBlendedTrajectories();
   }

   private final PoseReferenceFrame footstepFrame = new PoseReferenceFrame("FootstepFrame", worldFrame);
   private final PoseReferenceFrame adjustedFootstepFrame = new PoseReferenceFrame("AdjustedFootstepFrame", worldFrame);
   private final FramePose3D adjustedWaypoint = new FramePose3D();

   private void fillAndInitializeBlendedTrajectories()
   {
      double swingDuration = this.swingDuration.getDoubleValue();
      blendedSwingTrajectory.clear();
      if (swingTrajectoryBlendDuration > 0.0)
      {
         initialPose.changeFrame(worldFrame);
         blendedSwingTrajectory.blendInitialConstraint(initialPose, 0.0, swingTrajectoryBlendDuration);
      }
      if (footstepWasAdjusted.getBooleanValue())
      {
         touchdownTrajectory.setLinearTrajectory(swingDuration, rateLimitedAdjustedPose.getPosition(), finalLinearVelocity, touchdownAcceleration);

         // If there is a swing waypoint at the end of swing we want to preserve its transform to the footstep pose to not blend out
         // any touchdown trajectory when doing step adjustment.
         if (activeTrajectoryType.getValue() == TrajectoryType.WAYPOINTS && Precision.equals(swingWaypoints.getLast().getTime(), swingDuration))
         {
            footstepFrame.setPoseAndUpdate(footstepPose);
            adjustedFootstepFrame.setPoseAndUpdate(rateLimitedAdjustedPose);
            swingWaypoints.getLast().getPose(adjustedWaypoint);
            adjustedWaypoint.changeFrame(footstepFrame);
            adjustedWaypoint.setReferenceFrame(adjustedFootstepFrame);
            adjustedWaypoint.changeFrame(worldFrame);
            blendedSwingTrajectory.blendFinalConstraint(adjustedWaypoint, swingDuration, swingDuration);
            touchdownTrajectory.setOrientation(adjustedWaypoint.getOrientation());
         }
         else
         {
            blendedSwingTrajectory.blendFinalConstraint(rateLimitedAdjustedPose, swingDuration, swingDuration);
            touchdownTrajectory.setOrientation(rateLimitedAdjustedPose.getOrientation());
         }
      }
      blendedSwingTrajectory.initialize();
      touchdownTrajectory.initialize();

      if (!swingWaypointsForViz.isEmpty() && activeTrajectoryType.getEnumValue() == TrajectoryType.WAYPOINTS)
      {
         for (int i = 0; i < swingWaypoints.size(); i++)
         {
            blendedSwingTrajectory.compute(swingWaypoints.get(i).getTime());
            blendedSwingTrajectory.getPosition(tempWaypoint);
            swingWaypointsForViz.get(i).setMatchingFrame(tempWaypoint);
         }
      }
   }

   private void modifyFinalOrientationForTouchdown(FrameQuaternion finalOrientationToPack)
   {
      finalPosition.changeFrame(oppositeSoleZUpFrame);
      stanceFootPosition.changeFrame(oppositeSoleZUpFrame);
      double stepHeight = finalPosition.getZ() - stanceFootPosition.getZ();
      double initialFootstepPitch = finalOrientationToPack.getPitch();

      double footstepPitchModification;
      if (MathTools.intervalContains(stepHeight, stepDownHeightForToeTouchdown.getValue(), maximumHeightForHeelTouchdown.getValue())
            && doHeelTouchdownIfPossible.getValue())
      { // not stepping down too far, and not stepping up too far, so do heel strike
         double stepLength = finalPosition.getX() - stanceFootPosition.getX();
         double heelTouchdownAngle = MathTools.clamp(-stepLength * heelTouchdownLengthRatio.getValue(), -this.heelTouchdownAngle.getValue());
         // use the footstep pitch if its greater than the heel strike angle
         footstepPitchModification = Math.max(initialFootstepPitch, heelTouchdownAngle);
         // decrease the foot pitch modification if next step pitches down
         footstepPitchModification = Math.min(footstepPitchModification, heelTouchdownAngle + initialFootstepPitch);
         footstepPitchModification -= initialFootstepPitch;
      }
      else if (stepHeight < stepDownHeightForToeTouchdown.getValue() && doToeTouchdownIfPossible.getValue())
      { // stepping down and do toe touchdown
         double toeTouchdownAngle = MathTools.clamp(-toeTouchdownDepthRatio.getValue() * (stepHeight - stepDownHeightForToeTouchdown.getValue()),
                                                    this.toeTouchdownAngle.getValue());
         footstepPitchModification = Math.max(toeTouchdownAngle, initialFootstepPitch);
         footstepPitchModification -= initialFootstepPitch;
      }
      else
      {
         footstepPitchModification = 0.0;
      }

      finalOrientationToPack.appendPitchRotation(footstepPitchModification);
   }

   private void initializeOptimizer()
   {
      swingTrajectoryOptimizer.setInitialConditions(initialPosition, initialLinearVelocity);
      swingTrajectoryOptimizer.setFinalConditions(finalPosition, finalLinearVelocity);
      swingTrajectoryOptimizer.setStepTime(swingDuration.getDoubleValue());
      swingTrajectoryOptimizer.setTrajectoryType(activeTrajectoryType.getEnumValue(), positionWaypointsForSole);
      swingTrajectoryOptimizer.setSwingHeight(swingHeight.getDoubleValue());
      swingTrajectoryOptimizer.setStanceFootPosition(stanceFootPosition);
      swingTrajectoryOptimizer.setWaypointProportions(waypointProportions);
      swingTrajectoryOptimizer.initialize();
   }

   private void computeCurrentWeights(Vector3DReadOnly nominalAngularWeight, Vector3DReadOnly nominalLinearWeight, Vector3DBasics currentAngularWeightToPack,
                                      Vector3DBasics currentLinearWeightToPack)
   {
      currentAngularWeightToPack.set(nominalAngularWeight);
      leapOfFaithModule.scaleFootWeight(nominalLinearWeight, currentLinearWeightToPack);
   }

   private void transformDesiredsFromSoleFrameToControlFrame()
   {
      desiredSoleFrame.setPoseAndUpdate(desiredPosition, desiredOrientation);

      // change pose
      desiredPose.setToZero(desiredControlFrame);
      desiredPose.changeFrame(worldFrame);
      desiredPosition.setIncludingFrame(desiredPose.getPosition());
      desiredOrientation.setIncludingFrame(desiredPose.getOrientation());

      // change twist
      desiredLinearVelocity.changeFrame(desiredSoleFrame);
      desiredAngularVelocity.changeFrame(desiredSoleFrame);
      desiredTwist.setIncludingFrame(desiredSoleFrame, worldFrame, desiredSoleFrame, desiredAngularVelocity, desiredLinearVelocity);
      desiredTwist.changeFrame(desiredControlFrame);
      desiredLinearVelocity.setIncludingFrame(desiredTwist.getLinearPart());
      desiredAngularVelocity.setIncludingFrame(desiredTwist.getAngularPart());
      desiredLinearVelocity.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);

      // change spatial acceleration
      desiredLinearAcceleration.changeFrame(desiredSoleFrame);
      desiredAngularAcceleration.changeFrame(desiredSoleFrame);
      desiredSpatialAcceleration.setIncludingFrame(desiredSoleFrame, worldFrame, desiredSoleFrame, desiredAngularAcceleration, desiredLinearAcceleration);
      desiredSpatialAcceleration.changeFrame(desiredControlFrame);
      desiredLinearAcceleration.setIncludingFrame(desiredSpatialAcceleration.getLinearPart());
      desiredAngularAcceleration.setIncludingFrame(desiredSpatialAcceleration.getAngularPart());
      desiredLinearAcceleration.changeFrame(worldFrame);
      desiredAngularAcceleration.changeFrame(worldFrame);
   }

   private void setFootstepDurationInternal(double swingTime)
   {
      swingDuration.set(swingTime);
      maxSwingTimeSpeedUpFactor.set(Math.max(swingTime / minSwingTimeForDisturbanceRecovery.getDoubleValue(), 1.0));
   }

   private void setFootstepInternal(Footstep footstep)
   {
      footstep.getPose(footstepPose);
      footstepPose.changeFrame(worldFrame);
      footstepPose.setZ(footstepPose.getZ() + finalSwingHeightOffset.getValue());

      if (footstep.getTrajectoryType() == null)
      {
         activeTrajectoryType.set(TrajectoryType.DEFAULT);
      }
      else
      {
         activeTrajectoryType.set(footstep.getTrajectoryType());
      }
      this.positionWaypointsForSole.clear();
      this.swingWaypoints.clear();
      lastFootstepPose.changeFrame(worldFrame);

      if (activeTrajectoryType.getEnumValue() == TrajectoryType.CUSTOM)
      {
         List<FramePoint3D> positionWaypointsForSole = footstep.getCustomPositionWaypoints();
         for (int i = 0; i < positionWaypointsForSole.size(); i++)
            this.positionWaypointsForSole.add().setIncludingFrame(positionWaypointsForSole.get(i));
      }
      else if (activeTrajectoryType.getEnumValue() == TrajectoryType.WAYPOINTS)
      {
         List<FrameSE3TrajectoryPoint> swingWaypoints = footstep.getSwingTrajectory();
         for (int i = 0; i < swingWaypoints.size(); i++)
         {
            this.swingWaypoints.add().set(swingWaypoints.get(i));
         }
      }
      else
      {
         swingHeight.set(footstep.getSwingHeight());

         if (checkStepUpOrDown(footstepPose))
            activeTrajectoryType.set(TrajectoryType.OBSTACLE_CLEARANCE);

         RecyclingArrayList<MutableDouble> customWaypointProportions = footstep.getCustomWaypointProportions();
         if(customWaypointProportions.size() != 2)
         {
            if(!customWaypointProportions.isEmpty())
            {
               LogTools.warn("Ignoring custom waypoint proportions. Expected 2, got: " + customWaypointProportions.size());
            }

            List<DoubleProvider> waypointProportions = activeTrajectoryType.getEnumValue() == TrajectoryType.OBSTACLE_CLEARANCE ?
                  defaultObstacleClearanceWaypointProportions :
                  defaultWaypointProportions;
            this.waypointProportions[0] = waypointProportions.get(0).getValue();
            this.waypointProportions[1] = waypointProportions.get(1).getValue();
         }
         else
         {
            waypointProportions[0] = customWaypointProportions.get(0).getValue();
            waypointProportions[1] = customWaypointProportions.get(1).getValue();
         }
      }

      if (activeTrajectoryType.getEnumValue() == TrajectoryType.WAYPOINTS)
         swingTrajectoryBlendDuration = footstep.getSwingTrajectoryBlendDuration();
      else
         swingTrajectoryBlendDuration = 0.0;
   }

   private boolean checkStepUpOrDown(FramePose3D footstepPose)
   {
      double zDifference = Math.abs(footstepPose.getZ() - lastFootstepPose.getZ());
      return zDifference > minHeightDifferenceForObstacleClearance.getValue();
   }

   private double computeSwingTimeRemaining(double timeInState)
   {
      double swingDuration = this.swingDuration.getDoubleValue();
      if (!currentTimeWithSwingSpeedUp.isNaN())
      {
         double swingTimeRemaining = (swingDuration - currentTimeWithSwingSpeedUp.getDoubleValue()) / swingTimeSpeedUpFactor.getDoubleValue();
         return swingTimeRemaining;
      }
      else
      {
         return swingDuration - timeInState;
      }
   }

   private double computeSecondaryJointWeightScale(double timeInState)
   {
      double phaseInSwingState = timeInState / swingDuration.getDoubleValue();

      double scaleFactor;
      if (timeInState < swingDuration.getDoubleValue())
         scaleFactor = (maxScalingFactor - minScalingFactor) * (1.0 - Math.exp(-exponentialScalingRate * phaseInSwingState)) + minScalingFactor;
      else
         scaleFactor = (1.0 + 0.1 * (timeInState - swingDuration.getDoubleValue())) * maxScalingFactor;

      return scaleFactor;
   }

   private void changeControlFrame(FramePose3D controlFramePoseInEndEffector)
   {
      controlFramePoseInEndEffector.checkReferenceFrameMatch(contactableFoot.getRigidBody().getBodyFixedFrame());
      spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(controlFramePoseInEndEffector);
      controlFrame.setPoseAndUpdate(controlFramePoseInEndEffector);
   }

   public void setWeights(Vector3DReadOnly angularWeight, Vector3DReadOnly linearWeight)
   {
      this.nominalAngularWeight = angularWeight;
      this.nominalLinearWeight = linearWeight;
   }

   private void changeDesiredPoseBodyFrame(ReferenceFrame oldBodyFrame, ReferenceFrame newBodyFrame, FramePose3D framePoseToModify)
   {
      if (oldBodyFrame == newBodyFrame)
         return;

      framePoseToModify.get(oldBodyFrameDesiredTransform);
      newBodyFrame.getTransformToDesiredFrame(transformFromNewBodyFrameToOldBodyFrame, oldBodyFrame);
      newBodyFrameDesiredTransform.set(oldBodyFrameDesiredTransform);
      newBodyFrameDesiredTransform.multiply(transformFromNewBodyFrameToOldBodyFrame);
      framePoseToModify.set(newBodyFrameDesiredTransform);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   @Override
   public SpatialFeedbackControlCommand getFeedbackControlCommand()
   {
      return spatialFeedbackControlCommand;
   }

}
