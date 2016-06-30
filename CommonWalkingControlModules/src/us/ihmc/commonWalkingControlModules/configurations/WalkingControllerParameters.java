package us.ihmc.commonWalkingControlModules.configurations;

import java.util.LinkedHashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;

public abstract class WalkingControllerParameters implements HeadOrientationControllerParameters, SteppingParameters
{
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

   public abstract boolean doToeOffIfPossible();

   public abstract boolean doToeOffIfPossibleInSingleSupport();

   public abstract boolean checkECMPLocationToTriggerToeOff();

   public abstract double getMinStepLengthForToeOff();

   public abstract boolean doToeOffWhenHittingAnkleLimit();

   public abstract double getMaximumToeOffAngle();

   public abstract boolean doToeTouchdownIfPossible();

   public abstract double getToeTouchdownAngle();

   public abstract boolean doHeelTouchdownIfPossible();

   public abstract double getHeelTouchdownAngle();

   public abstract boolean allowShrinkingSingleSupportFootPolygon();

   public abstract boolean allowDisturbanceRecoveryBySpeedingUpSwing();

   public abstract boolean allowAutomaticManipulationAbort();

   public abstract double getMinimumSwingTimeForDisturbanceRecovery();

   public abstract double getICPErrorThresholdToSpeedUpSwing();

   public abstract ICPControlGains createICPControlGains(YoVariableRegistry registry);

   public abstract YoPDGains createPelvisICPBasedXYControlGains(YoVariableRegistry registry);

   public abstract YoOrientationPIDGainsInterface createPelvisOrientationControlGains(YoVariableRegistry registry);

   public abstract YoPDGains createCoMHeightControlGains(YoVariableRegistry registry);

   public abstract boolean getCoMHeightDriftCompensation();

   public abstract YoPDGains createUnconstrainedJointsControlGains(YoVariableRegistry registry);

   public abstract YoOrientationPIDGainsInterface createChestControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGainsInterface createSwingFootControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGainsInterface createHoldPositionFootControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGainsInterface createToeOffFootControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGainsInterface createEdgeTouchdownFootControlGains(YoVariableRegistry registry);

   public abstract double getSwingHeightMaxForPushRecoveryTrajectory();

   public abstract boolean doPrepareManipulationForLocomotion();

   public abstract boolean controlHeadAndHandsWithSliders();

   public abstract double getDefaultTransferTime();

   public abstract double getDefaultSwingTime();

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
}
