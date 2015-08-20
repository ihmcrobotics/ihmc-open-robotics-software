package us.ihmc.commonWalkingControlModules.configurations;

import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;


public interface WalkingControllerParameters extends HeadOrientationControllerParameters, SteppingParameters
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

   public abstract double minimumHeightBetweenAnkleAndPelvisForHeightAdjustment();

   public abstract double nominalHeightBetweenAnkleAndPelvisForHeightAdjustment();

   public abstract double maximumHeightBetweenAnkleAndPelvisForHeightAdjustment();

   public abstract double pelvisToAnkleThresholdForWalking();

   public abstract double getTimeToGetPreparedForLocomotion();

   public abstract boolean doToeOffIfPossible();

   public abstract boolean checkTrailingLegJacobianDeterminantToTriggerToeOff();

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

   public abstract ICPControlGains getICPControlGains();

   public abstract YoPDGains createPelvisICPBasedXYControlGains(YoVariableRegistry registry);

   public abstract YoOrientationPIDGains createPelvisOrientationControlGains(YoVariableRegistry registry);

   public abstract YoPDGains createCoMHeightControlGains(YoVariableRegistry registry);

   public abstract boolean getCoMHeightDriftCompensation();

   public abstract YoPDGains createUnconstrainedJointsControlGains(YoVariableRegistry registry);

   public abstract YoOrientationPIDGains createChestControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createSwingFootControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createHoldPositionFootControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createToeOffFootControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createEdgeTouchdownFootControlGains(YoVariableRegistry registry);

   public abstract YoSE3PIDGains createSupportFootControlGains(YoVariableRegistry registry);

   public abstract double getSwingHeightMaxForPushRecoveryTrajectory();

   public abstract double getSupportSingularityEscapeMultiplier();

   public abstract double getSwingSingularityEscapeMultiplier();

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
   public abstract LinkedHashMap<NeckJointName,ImmutablePair<Double,Double>> getSliderBoardControlledNeckJointsWithLimits();
   
   public abstract SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits();

   public abstract double getCoPThresholdFraction();

   public abstract String[] getJointsToIgnoreInController();

   public abstract void setupMomentumOptimizationSettings(MomentumOptimizationSettings momentumOptimizationSettings);

   public abstract boolean doFancyOnToesControl();

   public abstract FootSwitchType getFootSwitchType();

   public abstract double getContactThresholdHeight();

   public abstract double getMaxICPErrorBeforeSingleSupportX();
   public abstract double getMaxICPErrorBeforeSingleSupportY();

   /**
    * Sometimes the robot can get stuck in transfer state because the ICP error is too large to to switch to swing.
    * Even in that state, the ICP planner is still giving a desired ICP velocity preventing the ICP convergence to a certain extent.
    * This parameter allows to cancel out this desired velocity when stuck in the transfer state helping the convergence of the ICP and will help to get the robot to switch to swing.
    * Set it to {@link Double#POSITIVE_INFINITY} so the ICP velocity won't be cancelled out.
    * A value around 0.5sec to 1.0sec seems reasonable.
    */
   public abstract double getDurationToCancelOutDesiredICPVelocityWhenStuckInTransfer();

   public abstract boolean finishSingleSupportWhenICPPlannerIsDone();

   /**
    * Whether or not to use the hack n13.
    * This hack consists in forcing the desired ICP to move inside the foot at the end of swing.
    * It is terrible because it is done outside the ICP planner, the parameters are not robot agnostic, and it is buggy.
    * Best thing to do is to switch to the new ICP planner that also provides the option to move the ICP towards the inside of the foot at the end of the swing but according to the ICP dynamics, much better. 
    */
   @Deprecated
   public abstract boolean useICPPlannerHackN13();
}
