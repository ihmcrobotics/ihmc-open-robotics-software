package us.ihmc.commonWalkingControlModules.configurations;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;

public abstract class WalkingControllerParameters implements HeadOrientationControllerParameters, SteppingParameters
{
   protected JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;

   public abstract SideDependentList<RigidBodyTransform> getDesiredHandPosesWithRespectToChestFrame();

   public abstract String[] getDefaultChestOrientationControlJointNames();

   public abstract double getOmega0();

   public abstract double getAnkleHeight();

   public abstract double getLegLength();

   public abstract double getMinLegLengthBeforeCollapsingSingleSupport();

   public abstract double getMinMechanicalLegLength();

   public abstract double minimumHeightAboveAnkle();

   public abstract double nominalHeightAboveAnkle();

   public abstract double maximumHeightAboveAnkle();

   public abstract double defaultOffsetHeightAboveAnkle();

   public abstract double pelvisToAnkleThresholdForWalking();

   public abstract double getTimeToGetPreparedForLocomotion();

   /**
    * Boolean to enable transitions to the toe off contact state, if the appropriate conditions are satisfied.
    * @return boolean (true = Allow Toe Off, false = Don't Allow Toe Off)
    */
   public abstract boolean doToeOffIfPossible();

   public abstract boolean doToeOffIfPossibleInSingleSupport();

   public abstract boolean checkECMPLocationToTriggerToeOff();

   /**
    * Minimum stance length in double support to enable toe off.
    * @return threshold stance length in meters
    */
   public abstract double getMinStepLengthForToeOff();

   /**
    * If the leading foot is above this value in height, it is one of the last checks that says whether or not to
    * switch the contact state to toe off for the trailing foot.
    * @return threshold height in meters for stepping up to cause toe off
    */
   public double getMinStepHeightForToeOff()
   {
      return 0.10;
   }

   /**
    * To enable that feature, {@link WalkingControllerParameters#doToeOffIfPossible()} return true is required. John parameter
    */
   public abstract boolean doToeOffWhenHittingAnkleLimit();

   /**
    * Ankle limit that triggers {@link WalkingControllerParameters#doToeOffWhenHittingAnkleLimit()}.
    * The minimum limit is taken between the returned value and the joint limit.
    */
   public double getAnkleLowerLimitToTriggerToeOff()
   {
      return -1.0;
   }
   /**
    * Sets the maximum pitch of the foot during toe off to be fed into the whole-body controller
    * @return maximum pitch angle
    */
   public abstract double getMaximumToeOffAngle();

   public abstract boolean doToeTouchdownIfPossible();

   public abstract double getToeTouchdownAngle();

   public abstract boolean doHeelTouchdownIfPossible();

   public abstract double getHeelTouchdownAngle();

   public abstract boolean allowShrinkingSingleSupportFootPolygon();

   /**
    * Attempts to speed up the swing state to match the desired ICP to the current ICP.
    * @return boolean (true = allow speed up, false = don't allow speed up)
    */
   public abstract boolean allowDisturbanceRecoveryBySpeedingUpSwing();

   public abstract boolean allowAutomaticManipulationAbort();

   public abstract double getMinimumSwingTimeForDisturbanceRecovery();

   /**
    * Determines whether to use the ICP Optimization controller or a standard ICP proportional controller (new feature to be tested with Atlas)
    * @return boolean (true = use ICP Optimization, false = use ICP Proportional Controller)
    */
   public abstract boolean useOptimizationBasedICPController();

   public abstract double getICPErrorThresholdToSpeedUpSwing();

   public abstract ICPControlGains createICPControlGains(YoVariableRegistry registry);

   public abstract YoPDGains createPelvisICPBasedXYControlGains(YoVariableRegistry registry);

   public abstract YoOrientationPIDGainsInterface createPelvisOrientationControlGains(YoVariableRegistry registry);

   public abstract YoPDGains createCoMHeightControlGains(YoVariableRegistry registry);

   public abstract boolean getCoMHeightDriftCompensation();

   public abstract YoPDGains createUnconstrainedJointsControlGains(YoVariableRegistry registry);

   public abstract YoOrientationPIDGainsInterface createChestControlGains(YoVariableRegistry registry);

   /** The gains used when the spine joints are controlled directly instead of the chest orientation */
   public YoPIDGains createSpineControlGains(YoVariableRegistry registry)
   {
      return null;
   }

   public Map<String, YoPIDGains> getOrCreateJointSpaceControlGains(YoVariableRegistry registry)
   {
      return new HashMap<String, YoPIDGains>();
   }

   public Map<String, YoOrientationPIDGainsInterface> getOrCreateTaskspaceOrientationControlGains(YoVariableRegistry registry)
   {
      return new HashMap<String, YoOrientationPIDGainsInterface>();
   }

   public Map<String, YoPositionPIDGainsInterface> getOrCreateTaskspacePositionControlGains(YoVariableRegistry registry)
   {
      return new HashMap<String, YoPositionPIDGainsInterface>();
   }

   public abstract YoSE3PIDGainsInterface createSwingFootControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGainsInterface createHoldPositionFootControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGainsInterface createToeOffFootControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGainsInterface createEdgeTouchdownFootControlGains(YoVariableRegistry registry);

   public abstract double getSwingHeightMaxForPushRecoveryTrajectory();

   /**
    * Specifies if the arm controller should be switching
    * to chest frame or jointspace only if necessary.
    * This is particularly useful when manipulation was performed
    * with respect to world during standing to prevent "leaving a hand behind"
    * when the robot starts walking.
    *
    * @return whether the manipulation control should get prepared
    *  for walking.
    */
   public abstract boolean doPrepareManipulationForLocomotion();

   /**
    * Specifies if the pelvis orientation controller should
    * be initialized before starting to walk.
    * When the controller is initialized, the pelvis will
    * smoothly cancel out the user orientation offset on
    * the first transfer of a walking sequence.
    *
    * @return whether the pelvis orientation control should get prepared
    *  for walking.
    */
   public boolean doPreparePelvisForLocomotion()
   {
      return true;
   }

   /**
    * Specifies whether upper-body motion is allowed when the robot is walking
    * or during any exchange support.
    *
    * @return whether the upper-body can be moved during walking or not.
    */
   public boolean allowUpperBodyMotionDuringLocomotion()
   {
      return false;
   }

   public abstract boolean controlHeadAndHandsWithSliders();

   /**
    * The default transfer time used in the walking controller. This is the time interval spent in double support shifting
    * the weight from one foot to the other while walking.
    */
   public abstract double getDefaultTransferTime();

   /**
    * The default swing time used in the walking controller. This is the time interval spent in single support moving the
    * swing foot to the next foothold.
    */
   public abstract double getDefaultSwingTime();

   /**
    * This is the default transfer time used in the walking controller to shift the weight back to the center of the feet
    * after executing a footstep plan.
    */
   public double getDefaultFinalTransferTime()
   {
      return getDefaultTransferTime();
   }

   /**
    * This is the default transfer time used in the walking controller to shift the weight to the initial stance foot
    * when starting to execute a footstep plan.
    */
   public double getDefaultInitialTransferTime()
   {
      return 1.0;
   }

   /**
    * This is the minimum transfer time that the controller will allow when adjusting transfer times to achieve certain step
    * times in footstep plans.
    */
   public double getMinimumTransferTime()
   {
      return 0.1;
   }

   /** Used by the UI to limit motion range of the spine yaw. It doesn't have to be equal to the actual joint limit */
   public abstract double getSpineYawLimit();

   /** Used by the UI to limit motion range of the spine pitch. It doesn't have to be equal to the actual joint limit */
   public abstract double getSpinePitchUpperLimit();

   /** Used by the UI to limit motion range of the spine pitch. It doesn't have to be equal to the actual joint limit */
   public abstract double getSpinePitchLowerLimit();

   /** Used by the UI to limit motion range of the spine roll. It doesn't have to be equal to the actual joint limit */
   public abstract double getSpineRollLimit();

   /** Used by the UI to indicate if the spine pitch joint is reversed (true for Valkyrie) */
   public abstract boolean isSpinePitchReversed();

   public abstract double getFoot_start_toetaper_from_back();

   public abstract double getSideLengthOfBoundingBoxForFootstepHeight();

   /** Useful to force the swing foot to end up with an height offset with respect to the given footstep. */
   public abstract double getDesiredTouchdownHeightOffset();

   /** Useful to force the swing foot go towards the ground once the desired final position is reached but the foot has not touched the ground yet. */
   public abstract double getDesiredTouchdownVelocity();

   /** Useful to force the swing foot accelerate towards the ground once the desired final position is reached but the foot has not touched the ground yet. */
   public abstract double getDesiredTouchdownAcceleration();

   public abstract double getContactThresholdForce();

   public abstract double getSecondContactThresholdForceIgnoringCoP();

   /** Returns a map of neck joint names and associated min/max value joint limits. */
   public abstract LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits();

   public abstract SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits();

   public abstract double getCoPThresholdFraction();

   public abstract String[] getJointsToIgnoreInController();

   public abstract MomentumOptimizationSettings getMomentumOptimizationSettings();

   /**
    * Boolean that determines if the foot state switch to hold position if the desired cop is close
    * to the edge of the support polygon.
    *
    * @return holdPositionIfCopOnEdge
    */
   public abstract boolean doFancyOnToesControl();

   public abstract FootSwitchType getFootSwitchType();

   public abstract double getContactThresholdHeight();

   public abstract double getMaxICPErrorBeforeSingleSupportX();

   public abstract double getMaxICPErrorBeforeSingleSupportY();

   public abstract boolean finishSingleSupportWhenICPPlannerIsDone();

   public abstract void useInverseDynamicsControlCore();

   public abstract void useVirtualModelControlCore();

   /**
    * This is the duration for which the desired foot center of pressure will be
    * drastically dampened to calm shakies. This particularly useful when
    * dealing with bad footholds.
    * Set to -1.0 to deactivate this feature.
    */
   public abstract double getHighCoPDampingDurationToPreventFootShakies();

   /**
    * This is complimentary information to {@link #getHighCoPDampingDurationToPreventFootShakies()}.
    * The high CoP damping is triggered on large CoP tracking error.
    * Set to {@link Double#POSITIVE_INFINITY} to deactivate this feature.
    */
   public abstract double getCoPErrorThresholdForHighCoPDamping();

   /**
    * Get the parameters for foothold exploration. The parameters should be created the first time this
    * method is called.
    */
   public ExplorationParameters getOrCreateExplorationParameters(YoVariableRegistry registry)
   {
      return null;
   }

   /**
    * During normal execution the control algorithm computes a desired CMP. It is then projected in the
    * support polygon to avoid angular momentum of the upper body. When the robot is falling and recovery is
    * impossible otherwise, the support used for CMP projection can be increased and the robot uses upper body
    * momentum. This value defines the amount the support polygon for CMP projection is increased in that case.
    *
    * @return maxAllowedDistanceCMPSupport
    */
   public double getMaxAllowedDistanceCMPSupport()
   {
      return Double.NaN;
   }

   /**
    * When true, some of the tracking performance will be degraded to reduce the generated angular momentum rate around
    * the vertical axis during swing only.
    * Useful when the robot has heavy legs and tends to slips during swing.
    * @return
    */
   public boolean minimizeAngularMomentumRateZDuringSwing()
   {
      return false;
   }

   /**
    * Determines whether the swing trajectory should be optimized (new feature to be tested with Atlas)
    */
   public boolean useSwingTrajectoryOptimizer()
   {
      return false;
   }

   /**
    * Determined whether the robot should use the 'support state' or the 'fully constrained' & 'hold position' states (new feature to be tested with Atlas)
    */
   public boolean useSupportState()
   {
      return false;
   }

   /**
    *
    * Determined whether the robot should use the velocity to be computed in the estimator, or just compute it from the robot state in the controller (new feature to be tested with Atlas)
    */
   public boolean useCenterOfMassVelocityFromEstimator()
   {
      return false;
   }

   /**
    * Returns a list of joint that should use the more restrictive joint limit enforcement in the QP
    */
   public String[] getJointsWithRestrictiveLimits(JointLimitParameters jointLimitParametersToPack)
   {
      return new String[0];
   }

   /**
    * Returns a ratio to multiply the swing foot velocity adjustment when the swing trajectory is modified online.
    * 0.0 will eliminate any velocity adjustment.
    * 1.0 will make it try to move to the new trajectory in 1 dt.
    * @return damping ratio (0.0 to 1.0)
    */
   public double getSwingFootVelocityAdjustmentDamping()
   {
      return 0.0;
   }

   public double getMinSwingHeightFromStanceFoot()
   {
      return 0.1;
   }

   /**
    * Determines whether the swing of the robot controls the toe point of the foot for better tracking or not.
    * (new feature to be tested with Atlas)
    */
   public boolean controlToeDuringSwing()
   {
      return false;
   }

   /**
    * Returns the parameters used in the privileged configuration handler.
    */
   public JointPrivilegedConfigurationParameters getJointPrivilegedConfigurationParameters()
   {
      if (jointPrivilegedConfigurationParameters == null)
         jointPrivilegedConfigurationParameters = new JointPrivilegedConfigurationParameters();

      return jointPrivilegedConfigurationParameters;
   }

   /**
    * Determines whether or not to attempt to directly control the height.
    * If true, the height will be controlled by controlling either the pelvis or the center of mass height.
    * If false, the height will be controlled inside the nullspace by trying to achieve the desired
    * privileged configuration in the legs.
    * @return boolean (true = control height with momentum, false = do not control height with momentum)
    */
   public boolean controlHeightWithMomentum()
   {
      return true;
   }

   /**
    * Determines whether or not to attempt to use straight legs when controlling the height in the nullspace.
    * This will not do anything noticeable unless {@link WalkingControllerParameters#controlHeightWithMomentum()} returns true.
    * @return boolean (true = try and straighten, false = do not try and straighten)
    */
   public boolean attemptToStraightenLegs()
   {
      return false;
   }

   /**
    * Returns a percent of the swing state to switch the privileged configuration to having straight knees
    * @return ratio of swing state (0.0 to 1.0)
    */
   public double getPercentOfSwingToStraightenLeg()
   {
      return 0.8;
   }

   /**
    * In transfer, this is the maximum distance from the ICP to the leading foot support polygon to allow toe-off.
    * If it is further than this, do not allow toe-off, as more control authority is needed from the trailing foot.
    * @return ICP proximity in meters
    */
   public double getICPProximityToLeadingFootForToeOff()
   {
      return 0.0;
   }

   /**
    * In swing, this determines maximum distance from the ICP to the leading foot support polygon to allow toe-off.
    * This distance is determined by finding the stance length, and multiplying it by the returned variable.
    * If it is further than this, do not allow toe-off, as more control authority is needed from the trailing foot.
    * @return percent of stance length for proximity
    */
   public double getICPPercentOfStanceForSSToeOff()
   {
      return 0.0;
   }

   /**
    * This is the duration used to straighten the desire privileged configuration of the stance leg's knee.
    * @return time in seconds for straightening
    */
   public double getDurationForStanceLegStraightening()
   {
      return 1.3;
   }

   /**
    * Angle used by the privileged configuration that is defined as straight for the knees.
    * @return angle in radians
    */
   public double getStraightKneeAngle()
   {
      return 0.05;
   }

   /**
    * If a step up or a step down is executed, the swing trajectory will switch to the obstacle clearance
    * mode. The value defined here determines the threshold for the height difference between current foot
    * position and step position that causes this switch.
    */
   public double getMinHeightDifferenceForStepUpOrDown()
   {
      return 0.04;
   }
}
