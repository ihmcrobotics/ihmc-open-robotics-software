package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.List;

import org.apache.commons.math3.util.Precision;

import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith.FootLeapOfFaithModule;
import us.ihmc.commonWalkingControlModules.trajectories.SoftTouchdownPoseTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.controllers.pidGains.PIDSE3GainsReadOnly;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.filters.RateLimitedYoFramePose;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.MultipleWaypointsBlendedPoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.PoseTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPoint;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPoseTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.robotics.trajectories.providers.CurrentRigidBodyStateProvider;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class SwingState extends AbstractUnconstrainedState
{
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

   private final YoFrameVector yoTouchdownAcceleration;
   private final YoFrameVector yoTouchdownVelocity;

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

   private final RecyclingArrayList<FramePoint3D> positionWaypointsForSole;
   private final RecyclingArrayList<FrameSE3TrajectoryPoint> swingWaypoints;

   private final YoDouble swingDuration;
   private final YoDouble swingHeight;

   private final YoDouble swingTimeSpeedUpFactor;
   private final YoDouble maxSwingTimeSpeedUpFactor;
   private final YoDouble minSwingTimeForDisturbanceRecovery;
   private final YoBoolean isSwingSpeedUpEnabled;
   private final YoDouble currentTime;
   private final YoDouble currentTimeWithSwingSpeedUp;

   private final YoBoolean doHeelTouchdownIfPossible;
   private final YoDouble heelTouchdownAngle;
   private final YoDouble maximumHeightForHeelTouchdown;
   private final YoDouble heelTouchdownLengthRatio;

   private final YoBoolean doToeTouchdownIfPossible;
   private final YoDouble toeTouchdownAngle;
   private final YoDouble stepDownHeightForToeTouchdown;
   private final YoDouble toeTouchdownDepthRatio;

   private final YoBoolean addOrientationMidpointForClearance;
   private final YoDouble midpointOrientationInterpolationForClearance;

   private final YoBoolean ignoreInitialAngularVelocityZ;
   private final YoDouble maxInitialLinearVelocityMagnitude;
   private final YoDouble maxInitialAngularVelocityMagnitude;

   private final DoubleParameter finalSwingHeightOffset;
   private final double controlDT;

   private final YoDouble minHeightDifferenceForObstacleClearance;

   private final MovingReferenceFrame soleFrame;

   private final PoseReferenceFrame desiredSoleFrame = new PoseReferenceFrame("desiredSoleFrame", worldFrame);
   private final PoseReferenceFrame desiredControlFrame = new PoseReferenceFrame("desiredControlFrame", desiredSoleFrame);
   private final FramePose3D desiredPose = new FramePose3D();
   private final Twist desiredTwist = new Twist();
   private final SpatialAccelerationVector desiredSpatialAcceleration = new SpatialAccelerationVector();

   private final RigidBodyTransform transformFromToeToAnkle = new RigidBodyTransform();

   private final YoDouble velocityAdjustmentDamping;
   private final YoFrameVector adjustmentVelocityCorrection;
   private final FramePoint3D unadjustedPosition = new FramePoint3D(worldFrame);

   private final FramePose3D adjustedFootstepPose = new FramePose3D();
   private final RateLimitedYoFramePose rateLimitedAdjustedPose;

   private final FramePose3D footstepPose = new FramePose3D();
   private final FramePose3D lastFootstepPose = new FramePose3D();

   private final FrameEuclideanTrajectoryPoint tempPositionTrajectoryPoint = new FrameEuclideanTrajectoryPoint();

   // for unit testing and debugging:
   private final YoInteger currentTrajectoryWaypoint;
   private final YoFramePoint yoDesiredSolePosition;
   private final YoFrameQuaternion yoDesiredSoleOrientation;
   private final YoFrameVector yoDesiredSoleLinearVelocity;
   private final YoFrameVector yoDesiredSoleAngularVelocity;

   public SwingState(FootControlHelper footControlHelper, YoFrameVector yoTouchdownVelocity, YoFrameVector yoTouchdownAcceleration, PIDSE3GainsReadOnly gains,
                     YoVariableRegistry registry)
   {
      super(ConstraintType.SWING, footControlHelper, gains, registry);

      this.yoTouchdownAcceleration = yoTouchdownAcceleration;
      this.yoTouchdownVelocity = yoTouchdownVelocity;

      controlDT = footControlHelper.getHighLevelHumanoidControllerToolbox().getControlDT();

      swingWaypoints = new RecyclingArrayList<>(Footstep.maxNumberOfSwingWaypoints, FrameSE3TrajectoryPoint.class);
      positionWaypointsForSole = new RecyclingArrayList<>(2, FramePoint3D.class);

      String namePrefix = robotSide.getCamelCaseNameForStartOfExpression() + "Foot";
      WalkingControllerParameters walkingControllerParameters = footControlHelper.getWalkingControllerParameters();
      SwingTrajectoryParameters swingTrajectoryParameters = walkingControllerParameters.getSwingTrajectoryParameters();

      finalSwingHeightOffset = new DoubleParameter(namePrefix + "SwingFinalHeightOffset", registry, swingTrajectoryParameters.getDesiredTouchdownHeightOffset(), -0.01, 0.005);
      //finalSwingHeightOffset.set(swingTrajectoryParameters.getDesiredTouchdownHeightOffset());
      replanTrajectory = new YoBoolean(namePrefix + "SwingReplanTrajectory", registry);
      footstepWasAdjusted = new YoBoolean(namePrefix + "FootstepWasAdjusted", registry);

      minHeightDifferenceForObstacleClearance = new YoDouble(namePrefix + "MinHeightDifferenceForObstacleClearance", registry);
      minHeightDifferenceForObstacleClearance.set(swingTrajectoryParameters.getMinHeightDifferenceForStepUpOrDown());

      velocityAdjustmentDamping = new YoDouble(namePrefix + "VelocityAdjustmentDamping", registry);
      velocityAdjustmentDamping.set(swingTrajectoryParameters.getSwingFootVelocityAdjustmentDamping());
      adjustmentVelocityCorrection = new YoFrameVector(namePrefix + "AdjustmentVelocityCorrection", worldFrame, registry);

      doHeelTouchdownIfPossible = new YoBoolean(namePrefix + "DoHeelTouchdownIfPossible", registry);
      heelTouchdownAngle = new YoDouble(namePrefix + "HeelTouchdownAngle", registry);
      maximumHeightForHeelTouchdown = new YoDouble(namePrefix + "MaximumHeightForHeelTouchdown", registry);
      heelTouchdownLengthRatio = new YoDouble(namePrefix + "HeelTouchdownLengthRatio", registry);
      doHeelTouchdownIfPossible.set(swingTrajectoryParameters.doHeelTouchdownIfPossible());
      heelTouchdownAngle.set(swingTrajectoryParameters.getHeelTouchdownAngle());
      maximumHeightForHeelTouchdown.set(swingTrajectoryParameters.getMaximumHeightForHeelTouchdown());
      heelTouchdownLengthRatio.set(swingTrajectoryParameters.getHeelTouchdownLengthRatio());

      rateLimitedAdjustedPose = new RateLimitedYoFramePose(namePrefix + "AdjustedFootstepPose", "", registry, 10.0, controlDT, worldFrame);

      doToeTouchdownIfPossible = new YoBoolean(namePrefix + "DoToeTouchdownIfPossible", registry);
      toeTouchdownAngle = new YoDouble(namePrefix + "ToeTouchdownAngle", registry);
      stepDownHeightForToeTouchdown = new YoDouble(namePrefix + "StepDownHeightForToeTouchdown", registry);
      toeTouchdownDepthRatio = new YoDouble(namePrefix + "ToeTouchdownDepthRatio", registry);
      doToeTouchdownIfPossible.set(swingTrajectoryParameters.doToeTouchdownIfPossible());
      toeTouchdownAngle.set(swingTrajectoryParameters.getToeTouchdownAngle());
      stepDownHeightForToeTouchdown.set(swingTrajectoryParameters.getStepDownHeightForToeTouchdown());
      toeTouchdownDepthRatio.set(swingTrajectoryParameters.getToeTouchdownDepthRatio());

      addOrientationMidpointForClearance = new YoBoolean(namePrefix + "AddOrientationMidpointForClearance", registry);
      midpointOrientationInterpolationForClearance = new YoDouble(namePrefix + "MidpointOrientationInterpolationForClearance", registry);
      addOrientationMidpointForClearance.set(swingTrajectoryParameters.addOrientationMidpointForObstacleClearance());
      midpointOrientationInterpolationForClearance.set(swingTrajectoryParameters.midpointOrientationInterpolationForObstacleClearance());

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
      double[] waypointProportions = swingTrajectoryParameters.getSwingWaypointProportions();
      double[] obstacleClearanceProportions = swingTrajectoryParameters.getObstacleClearanceProportions();

      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();

      swingTrajectoryOptimizer = new TwoWaypointSwingGenerator(namePrefix + "Swing", waypointProportions, obstacleClearanceProportions,
                                                               minSwingHeightFromStanceFoot, maxSwingHeightFromStanceFoot, registry, yoGraphicsListRegistry);

      double minDistanceToStance = walkingControllerParameters.getMinSwingTrajectoryClearanceFromStanceFoot();
      swingTrajectoryOptimizer.enableStanceCollisionAvoidance(robotSide, oppositeSoleZUpFrame, minDistanceToStance);

      MultipleWaypointsPoseTrajectoryGenerator swingTrajectory = new MultipleWaypointsPoseTrajectoryGenerator(namePrefix + "Swing", Footstep.maxNumberOfSwingWaypoints + 2, registry);
      blendedSwingTrajectory = new MultipleWaypointsBlendedPoseTrajectoryGenerator(namePrefix + "Swing", swingTrajectory, worldFrame, registry);
      touchdownTrajectory = new SoftTouchdownPoseTrajectoryGenerator(namePrefix + "Touchdown", registry);
      currentStateProvider = new CurrentRigidBodyStateProvider(soleFrame);

      activeTrajectoryType = new YoEnum<>(namePrefix + TrajectoryType.class.getSimpleName(), registry, TrajectoryType.class);
      swingDuration = new YoDouble(namePrefix + "SwingDuration", registry);
      swingHeight = new YoDouble(namePrefix + "SwingHeight", registry);

      swingTimeSpeedUpFactor = new YoDouble(namePrefix + "SwingTimeSpeedUpFactor", registry);
      minSwingTimeForDisturbanceRecovery = new YoDouble(namePrefix + "MinSwingTimeForDisturbanceRecovery", registry);
      minSwingTimeForDisturbanceRecovery.set(walkingControllerParameters.getMinimumSwingTimeForDisturbanceRecovery());
      maxSwingTimeSpeedUpFactor = new YoDouble(namePrefix + "MaxSwingTimeSpeedUpFactor", registry);
      currentTime = new YoDouble(namePrefix + "CurrentTime", registry);
      currentTimeWithSwingSpeedUp = new YoDouble(namePrefix + "CurrentTimeWithSwingSpeedUp", registry);
      isSwingSpeedUpEnabled = new YoBoolean(namePrefix + "IsSwingSpeedUpEnabled", registry);
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
      yoDesiredSolePosition = new YoFramePoint(namePrefix + "DesiredSolePositionInWorld", worldFrame, registry);
      yoDesiredSoleOrientation = new YoFrameQuaternion(namePrefix + "DesiredSoleOrientationInWorld", worldFrame, registry);
      yoDesiredSoleLinearVelocity = new YoFrameVector(namePrefix + "DesiredSoleLinearVelocityInWorld", worldFrame, registry);
      yoDesiredSoleAngularVelocity = new YoFrameVector(namePrefix + "DesiredSoleAngularVelocityInWorld", worldFrame, registry);
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
      return ReferenceFrame.constructFrameWithUnchangingTransformToParent(robotSide.getCamelCaseNameForStartOfExpression() + "ToeFrame", footFrame, transformFromToeToAnkle);
   }

   @Override
   protected void initializeTrajectory()
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

      footstepWasAdjusted.set(false);

      fillAndInitializeTrajectories(true);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      swingTimeSpeedUpFactor.set(1.0);
      currentTimeWithSwingSpeedUp.set(Double.NaN);
      replanTrajectory.set(false);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();

      swingTimeSpeedUpFactor.set(Double.NaN);
      currentTimeWithSwingSpeedUp.set(Double.NaN);

      swingTrajectoryOptimizer.informDone();

      adjustmentVelocityCorrection.setToZero();

      yoDesiredSolePosition.setToNaN();
      yoDesiredSoleOrientation.setToNaN();
      yoDesiredSoleLinearVelocity.setToNaN();
      yoDesiredSoleAngularVelocity.setToNaN();
   }


   @Override
   protected void computeAndPackTrajectory()
   {
      currentTime.set(getTimeInCurrentState());

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
         fillAndInitializeBlendedTrajectory();
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
         adjustmentVelocityCorrection.scale(velocityAdjustmentDamping.getDoubleValue());

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

      yoDesiredSolePosition.setAndMatchFrame(desiredPosition);
      yoDesiredSoleOrientation.setAndMatchFrame(desiredOrientation);
      yoDesiredSoleLinearVelocity.setAndMatchFrame(desiredLinearVelocity);
      yoDesiredSoleAngularVelocity.setAndMatchFrame(desiredAngularVelocity);
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

      return computeSwingTimeRemaining();
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
      finalLinearVelocity.setIncludingFrame(yoTouchdownVelocity);
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
            PrintTools.warn(this, "Should use blending when providing waypoint at t = 0.0.");
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
         if (addOrientationMidpointForClearance.getBooleanValue() && activeTrajectoryType.getEnumValue() == TrajectoryType.OBSTACLE_CLEARANCE)
         {
            tmpOrientation.setToZero(worldFrame);
            tmpVector.setToZero(worldFrame);
            tmpOrientation.interpolate(initialOrientation, finalOrientation, midpointOrientationInterpolationForClearance.getDoubleValue());
            blendedSwingTrajectory.appendOrientationWaypoint(0.5 * swingDuration, tmpOrientation, tmpVector);
         }
      }

      modifyFinalOrientationForTouchdown(finalOrientation);

      // append footstep pose if not provided in the waypoints
      if (appendFootstepPose)
      {
         blendedSwingTrajectory.appendPositionWaypoint(swingDuration, finalPosition, finalLinearVelocity);
         blendedSwingTrajectory.appendOrientationWaypoint(swingDuration, finalOrientation, finalAngularVelocity);
      }

      // setup touchdown trajectory
      // TODO: revisit the touchdown velocity and accelerations
      touchdownTrajectory.setLinearTrajectory(swingDuration, finalPosition, finalLinearVelocity, yoTouchdownAcceleration);
      touchdownTrajectory.setOrientation(finalOrientation);
      touchdownTrajectory.initialize();

      blendedSwingTrajectory.initializeTrajectory();

      fillAndInitializeBlendedTrajectory();
   }

   private void fillAndInitializeBlendedTrajectory()
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
         blendedSwingTrajectory.blendFinalConstraint(rateLimitedAdjustedPose, swingDuration, swingDuration);
      }
      blendedSwingTrajectory.initialize();
   }

   private void modifyFinalOrientationForTouchdown(FrameQuaternion finalOrientationToPack)
   {
      finalPosition.changeFrame(oppositeSoleZUpFrame);
      stanceFootPosition.changeFrame(oppositeSoleZUpFrame);
      double stepHeight = finalPosition.getZ() - stanceFootPosition.getZ();
      double initialFootstepPitch = finalOrientationToPack.getPitch();

      double footstepPitchModification;
      if (MathTools.intervalContains(stepHeight, stepDownHeightForToeTouchdown.getDoubleValue(), maximumHeightForHeelTouchdown.getDoubleValue()) && doHeelTouchdownIfPossible.getBooleanValue())
      { // not stepping down too far, and not stepping up too far, so do heel strike
         double stepLength = finalPosition.getX() - stanceFootPosition.getX();
         double heelTouchdownAngle = MathTools.clamp(-stepLength * heelTouchdownLengthRatio.getDoubleValue(), -this.heelTouchdownAngle.getDoubleValue());
         // use the footstep pitch if its greater than the heel strike angle
         footstepPitchModification = Math.max(initialFootstepPitch, heelTouchdownAngle);
         // decrease the foot pitch modification if next step pitches down
         footstepPitchModification = Math.min(footstepPitchModification, heelTouchdownAngle + initialFootstepPitch);
         footstepPitchModification -= initialFootstepPitch;
      }
      else if (stepHeight < stepDownHeightForToeTouchdown.getDoubleValue() && doToeTouchdownIfPossible.getBooleanValue())
      { // stepping down and do toe touchdown
         double toeTouchdownAngle = MathTools.clamp(-toeTouchdownDepthRatio.getDoubleValue() * (stepHeight - stepDownHeightForToeTouchdown.getDoubleValue()),
               this.toeTouchdownAngle.getDoubleValue());
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
      swingTrajectoryOptimizer.initialize();
   }


   @Override
   protected void computeCurrentWeights(Vector3DReadOnly nominalAngularWeight, Vector3DReadOnly nominalLinearWeight, Vector3DBasics currentAngularWeightToPack,
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
      desiredTwist.set(desiredSoleFrame, worldFrame, desiredSoleFrame, desiredLinearVelocity, desiredAngularVelocity);
      desiredTwist.changeFrame(desiredControlFrame);
      desiredTwist.getLinearPart(desiredLinearVelocity);
      desiredTwist.getAngularPart(desiredAngularVelocity);
      desiredLinearVelocity.changeFrame(worldFrame);
      desiredAngularVelocity.changeFrame(worldFrame);

      // change spatial acceleration
      desiredLinearAcceleration.changeFrame(desiredSoleFrame);
      desiredAngularAcceleration.changeFrame(desiredSoleFrame);
      desiredSpatialAcceleration.set(desiredSoleFrame, worldFrame, desiredSoleFrame, desiredLinearAcceleration, desiredAngularAcceleration);
      desiredSpatialAcceleration.changeFrameNoRelativeMotion(desiredControlFrame);
      desiredSpatialAcceleration.getLinearPart(desiredLinearAcceleration);
      desiredSpatialAcceleration.getAngularPart(desiredAngularAcceleration);
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
            this.swingWaypoints.add().set(swingWaypoints.get(i));
      }
      else
      {
         swingHeight.set(footstep.getSwingHeight());

         if (checkStepUpOrDown(footstepPose))
            activeTrajectoryType.set(TrajectoryType.OBSTACLE_CLEARANCE);
      }

      if (activeTrajectoryType.getEnumValue() == TrajectoryType.WAYPOINTS)
         swingTrajectoryBlendDuration = footstep.getSwingTrajectoryBlendDuration();
      else
         swingTrajectoryBlendDuration = 0.0;
   }

   private boolean checkStepUpOrDown(FramePose3D footstepPose)
   {
      double zDifference = Math.abs(footstepPose.getZ() - lastFootstepPose.getZ());
      return zDifference > minHeightDifferenceForObstacleClearance.getDoubleValue();
   }

   private double computeSwingTimeRemaining()
   {
      double swingDuration = this.swingDuration.getDoubleValue();
      if (!currentTimeWithSwingSpeedUp.isNaN())
      {
         double swingTimeRemaining = (swingDuration - currentTimeWithSwingSpeedUp.getDoubleValue()) / swingTimeSpeedUpFactor.getDoubleValue();
         return swingTimeRemaining;
      }
      else
      {
         return swingDuration - getTimeInCurrentState();
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

}
