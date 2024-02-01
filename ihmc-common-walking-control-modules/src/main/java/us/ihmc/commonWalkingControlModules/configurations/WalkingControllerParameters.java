package us.ihmc.commonWalkingControlModules.configurations;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ToeSlippingDetector;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PID3DConfiguration;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;

public abstract class WalkingControllerParameters
{
   private final JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;
   private final OneDoFJointPrivilegedConfigurationParameters kneePrivilegedConfigurationParameters;

   public WalkingControllerParameters()
   {
      jointPrivilegedConfigurationParameters = new JointPrivilegedConfigurationParameters();

      kneePrivilegedConfigurationParameters = new OneDoFJointPrivilegedConfigurationParameters();
      kneePrivilegedConfigurationParameters.setConfigurationGain(40.0);
      kneePrivilegedConfigurationParameters.setVelocityGain(6.0);
      kneePrivilegedConfigurationParameters.setWeight(5.0);
      kneePrivilegedConfigurationParameters.setMaxAcceleration(Double.POSITIVE_INFINITY);
      kneePrivilegedConfigurationParameters.setPrivilegedConfigurationOption(PrivilegedConfigurationCommand.PrivilegedConfigurationOption.AT_MID_RANGE);
   }

   /**
    * Returns the value of sqrt(g / z0) which corresponds to omega0 in the Linear Inverted Pendulum
    * Model that the ICP is based on. Note, that this value is a tuning parameter for each robot and is
    * not computed from the actual CoM height.
    *
    * @return the value for omega0 that is used in the controller for ICP related computations.
    */
   public abstract double getOmega0();

   public enum SmoothFootUnloadMethod
   {
      HARD_CONSTRAINT, RHO_WEIGHT
   };

   /**
    * Specifies if the desired ground reaction force for the force that is about to swing should
    * smoothly be brought to zero by either:
    * <ul>
    * <li>adding a inequality constraint on the z-force,
    * <li>or increasing the rho weights.
    * </ul>
    * 
    * @return whether to perform smooth unloading before swing or not. Default value is {@code null}
    *         for disabling the feature.
    */
   public SmoothFootUnloadMethod enforceSmoothFootUnloading()
   {
      return null;
   }

   /**
    * Only used when {@link #enforceSmoothFootUnloading()} equals
    * {@code SmoothFootUnloadMethod.RHO_WEIGHT}, it specifies the final rho weight value of the foot
    * being unloaded.
    * 
    * @return the final unloaded rho weight value, should be greater that the default rho weight
    *         specified in {@link MomentumOptimizationSettings}.
    */
   public double getFinalUnloadedRhoWeight()
   {
      return 0.001;
   }

   /**
    * Specifies if the controller should attempt at detecting foot slipping during toe off when
    * walking. If foot slip is detected the swing is started right away.
    *
    * @return whether the controller will detect foot slipping during the toe off state
    */
   public boolean enableToeOffSlippingDetection()
   {
      return false;
   }

   public boolean resubmitStepsInSwingEveryTick()
   {
      return false;
   }

   public boolean resubmitStepsInTransferEveryTick()
   {
      return false;
   }

   /**
    * Method returns robot specific parameters for the {@link ToeSlippingDetector}.
    * <p>
    * Must be overwritten if {@link #enableToeOffSlippingDetection()} returns {@code true}.
    * </p>
    * 
    * @return the parameters for slip detection during toe off.
    * @see ToeSlippingDetector#configure(ToeSlippingDetectorParameters)
    */
   public ToeSlippingDetectorParameters getToeSlippingDetectorParameters()
   {
      return null;
   }

   /**
    * If the return value is {@code true} the controller will speed up the swing of a foot when walking
    * to match the desired ICP to the current ICP. See {@link #getICPErrorThresholdToSpeedUpSwing()} to
    * specify the threshold on the ICP error which will cause a swing speedup.
    *
    * @return whether swing speedup is enabled
    */
   public abstract boolean allowDisturbanceRecoveryBySpeedingUpSwing();

   /**
    * Parameter determines the minimum swing time in case the controller is speeding up the swing.
    *
    * @return minimum value the controller can reduce the swing time to when recovering
    * @see #allowDisturbanceRecoveryBySpeedingUpSwing()
    */
   public abstract double getMinimumSwingTimeForDisturbanceRecovery();

   /**
    * Determines the threshold on the ICP error that will cause the controller to speed up the swing
    * when in single support. Note that this will only have an effect if
    * {@link #allowDisturbanceRecoveryBySpeedingUpSwing()} returns {@code true}.
    *
    * @return the threshold on the ICP error to trigger swing speedup
    */
   public abstract double getICPErrorThresholdToSpeedUpSwing();

   /**
    * Specifies whether the controller will abort any arm trajectories when close to loosing its
    * balance. This is determined by the ICP tracking error.
    *
    * @return whether the robot will abort arm trajectories when the ICP error is large
    */
   public abstract boolean allowAutomaticManipulationAbort();

   /**
    * Determines the threshold for the ICP tracking error that will cause the robot do abort
    * manipulation if {@link #allowAutomaticManipulationAbort()} is returning {@code true}.
    *
    * @return the threshold on the ICP error that will trigger manipulation abort
    * @see #allowAutomaticManipulationAbort()
    */
   public double getICPErrorThresholdForManipulationAbort()
   {
      return 0.04;
   }

   /**
    * This parameter sets the buffer around the support polygon to constrain the offset ICP used in
    * {@link PelvisICPBasedTranslationManager}. It's defined in meters.
    */
   public double getPelvisTranslationICPSupportPolygonSafeMargin()
   {
      return 0.04;
   }

   /**
    * This method returns the gains used in the controller to regulate the center of mass height.
    */
   public abstract PDGains getCoMHeightControlGains();

   /**
    * Gains used to compute desired accelerations: joint acceleration = kp * (q_des - q) + kd * (v_des
    * - v) The feedback in acceleration-space is used as an objective in the whole-body QP
    * <p>
    * Each {@link GroupParameter} contains gains for one joint group:</br>
    * - The name of the joint group that the gain is used for (e.g. Arms).</br>
    * - The gains for the joint group.</br>
    * - The names of all rigid bodies in the joint group.
    * </p>
    * If a joint is not contained in the list, jointspace control is not supported for that joint.
    *
    * @return list containing jointspace PID gains and the corresponding joints
    */
   public List<GroupParameter<PIDGainsReadOnly>> getJointSpaceControlGains()
   {
      return new ArrayList<>();
   }

   /**
    * Returns a list of taskspace orientation control gains for groups of bodies.
    * <p>
    * Each {@link GroupParameter} contains gains for one body group:</br>
    * - The name of the body group that the gain is used for (e.g. Hands).</br>
    * - The gains for the body group.</br>
    * - The names of all rigid bodies in the body group.
    * </p>
    * If a body is not contained in the list, taskspace orientation or pose control is not supported
    * for that rigid body. These gains will be used by the controller for tracking taskspace
    * orientation trajectories (or the orientation part of a pose trajectory) for a rigid body.
    *
    * @return list containing orientation PID gains and the corresponding rigid bodies
    */
   public List<GroupParameter<PID3DConfiguration>> getTaskspaceOrientationControlGains()
   {
      return new ArrayList<>();
   }

   /**
    * Returns a list of taskspace position control gains for groups of bodies.
    * <p>
    * Each {@link GroupParameter} contains gains for one body group:</br>
    * - The name of the body group that the gain is used for (e.g. Hands).</br>
    * - The gains for the body group.</br>
    * - The names of all rigid bodies in the body group.
    * </p>
    * If a body is not contained in the list, taskspace position or pose control is not supported for
    * that rigid body. These gains will be used by the controller for tracking taskspace position
    * trajectories (or the position part of a pose trajectory) for a rigid body.
    *
    * @return list containing position PID gains and the corresponding rigid bodies
    */
   public List<GroupParameter<PID3DConfiguration>> getTaskspacePositionControlGains()
   {
      return new ArrayList<>();
   }

   /**
    * Returns a map with default control modes for each rigid body.
    * <p>
    * The key of the map is the rigid body name as defined in the joint map. Possible control modes are
    * defined in {@link RigidBodyControlMode}. By default (if a body is not contained in the map)
    * {@link RigidBodyControlMode#JOINTSPACE} will be used for the body. In some cases (e.g. the chest)
    * it makes more sense to use the default mode {@link RigidBodyControlMode#TASKSPACE}.
    * </p>
    *
    * @return the default control mode of the body
    */
   public Map<String, RigidBodyControlMode> getDefaultControlModesForRigidBodies()
   {
      return new HashMap<>();
   }

   /**
    * The map returned contains the default home joint angles. The key of the map is the joint name as
    * defined in the robot joint map.
    *
    * @return map containing home joint angles by joint name
    */
   public TObjectDoubleHashMap<String> getOrCreateJointHomeConfiguration()
   {
      return new TObjectDoubleHashMap<String>();
   }

   /**
    * If true, the rigid body jointspace control state for the given rigid body will be setup
    * with function generators
    */
   public boolean enableFunctionGeneratorMode(String rigidBodyName)
   {
      return false;
   }

   /**
    * The map returned contains the default rigid body poses in their respective base frame. For
    * example, if the base frame of the chest body is the pelvis z-up frame this should contain the
    * home pose of the chest in that frame. If the particular body does not support full pose control
    * but only orientation control the position part of the pose will be disregarded.
    * <p>
    * The key of the map is the name of the rigid body that can be obtained with
    * {@link RigidBodyBasics#getName()}. If a body is not contained in this map but a default control
    * mode of {@link RigidBodyControlMode#TASKSPACE} is not supported for that body.
    *
    * @return map containing home pose in base frame by body name
    */
   public Map<String, Pose3D> getOrCreateBodyHomeConfiguration()
   {
      return new HashMap<String, Pose3D>();
   }

   /**
    * Returns the gains used for the foot pose when in swing.
    */
   public abstract PIDSE3Configuration getSwingFootControlGains();

   /**
    * Returns the gains used for the foot when in support. Note that these gains are only used when the
    * foot is not loaded or close to tipping. Of that is not the case the foot pose when in support is
    * not controlled using a feedback controller.
    */
   public abstract PIDSE3Configuration getHoldPositionFootControlGains();

   /**
    * Returns the gains used for the foot when in the toe off state. Note that some parts of the foot
    * orientation will not use these gains. The foot pitch for example is usually not controlled
    * explicitly during tow off.
    */
   public abstract PIDSE3Configuration getToeOffFootControlGains();

   /**
    * Specifies if the arm controller should be switching to chest frame or jointspace only if
    * necessary. This is particularly useful when manipulation was performed with respect to world
    * during standing to prevent "leaving a hand behind" when the robot starts walking.
    *
    * @return whether the manipulation control should get prepared for walking.
    */
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

   /**
    * Specifies if the pelvis orientation controller should be initialized before starting to walk.
    * When the controller is initialized, the pelvis will smoothly cancel out the user orientation
    * offset on the first transfer of a walking sequence.
    *
    * @return whether the pelvis orientation control should get prepared for walking.
    */
   public boolean doPreparePelvisForLocomotion()
   {
      return true;
   }

   /**
    * Specifies whether upper-body motion is allowed when the robot is walking or during any exchange
    * support.
    *
    * @return whether the upper-body can be moved during walking or not.
    */
   public boolean allowUpperBodyMotionDuringLocomotion()
   {
      return false;
   }

   /**
    * The default transfer time used in the walking controller. This is the time interval spent in
    * double support shifting the weight from one foot to the other while walking.
    */
   public abstract double getDefaultTransferTime();

   /**
    * The default swing time used in the walking controller. This is the time interval spent in single
    * support moving the swing foot to the next foothold.
    */
   public abstract double getDefaultSwingTime();

   /**
    * This is the default transfer time used in the walking controller to shift the weight back to the
    * center of the feet after executing a footstep plan.
    */
   public double getDefaultFinalTransferTime()
   {
      return getDefaultTransferTime();
   }

   /**
    * This is the default transfer time used in the walking controller to shift the weight to the
    * initial stance foot when starting to execute a footstep plan.
    */
   public double getDefaultInitialTransferTime()
   {
      return 1.0;
   }

   /**
    * This is the minimum transfer time that the controller will allow when adjusting transfer times to
    * achieve certain step times in footstep plans.
    */
   public double getMinimumTransferTime()
   {
      return 0.1;
   }

   public abstract FootSwitchFactory getFootSwitchFactory();

   public SideDependentList<FootSwitchFactory> getFootSwitchFactories()
   {
      FootSwitchFactory footSwitchFactory = getFootSwitchFactory();
      return new SideDependentList<>(footSwitchFactory, footSwitchFactory);
   }

   /**
    * Returns a list of joints that will not be used by the controller.
    */
   public abstract String[] getJointsToIgnoreInController();

   /**
    * Returns a list of joints that cannot be controlled but should still be considered by the
    * controller.
    */
   public String[] getInactiveJoints()
   {
      return null;
   }

   /**
    * Returns the {@link MomentumOptimizationSettings} for this robot. These parameters define the
    * weights given to the objectives of the walking controller in the QP.
    */
   public abstract MomentumOptimizationSettings getMomentumOptimizationSettings();

   /**
    * Returns the {@link FeedbackControllerSettings} for this robot. These parameters additional
    * configuration options for the the {@code WholeBodyFeedbackController}.
    * 
    * @return the feedback controller settings.
    */
   public FeedbackControllerSettings getFeedbackControllerSettings()
   {
      return null;
   }

   /**
    * This parameter is used when the controller checks if it is safe to transition from transfer to
    * single support state when walking. The transition is considered safe if the ICP tracking error
    * lies within an ellipse with the axes aligned with the z-up ankle frame of the stance foot. This
    * parameter defines the radius of the ellipse along the x-axis of that frame.
    * </p>
    * Note that if the ICP leaves the support area the single support state will be started regardless
    * of the ICP error in the hope to recover by stepping.
    * </p>
    * 
    * @see #getMaxICPErrorBeforeSingleSupportBackwardX()
    * @see #getMaxICPErrorBeforeSingleSupportInnerY()
    * @see #getMaxICPErrorBeforeSingleSupportOuterY()
    */
   public abstract double getMaxICPErrorBeforeSingleSupportForwardX();

   /**
    * This parameter is used when the controller checks if it is safe to transition from transfer to
    * single support state when walking. The transition is considered safe if the ICP tracking error
    * lies within an ellipse with the axes aligned with the z-up ankle frame of the stance foot. This
    * parameter defines the radius of the ellipse along the x-axis of that frame.
    * </p>
    * Note that if the ICP leaves the support area the single support state will be started regardless
    * of the ICP error in the hope to recover by stepping.
    * </p>
    * 
    * @see #getMaxICPErrorBeforeSingleSupportForwardX()
    * @see #getMaxICPErrorBeforeSingleSupportInnerY()
    * @see #getMaxICPErrorBeforeSingleSupportOuterY()
    */
   public double getMaxICPErrorBeforeSingleSupportBackwardX()
   {
      return getMaxICPErrorBeforeSingleSupportForwardX();
   }

   /**
    * This parameter is used when the controller checks if it is safe to transition from transfer to
    * single support state when walking. The transition is considered safe if the ICP tracking error
    * lies within an ellipse with the axes aligned with the z-up ankle frame of the stance foot. This
    * parameter defines the radius of the ellipse along the y-axis of that frame.
    * </p>
    * Note that if the ICP leaves the support area the single support state will be started regardless
    * of the ICP error in the hope to recover by stepping.
    * </p>
    * 
    * @see #getMaxICPErrorBeforeSingleSupportForwardX()
    * @see #getMaxICPErrorBeforeSingleSupportBackwardX()
    * @see #getMaxICPErrorBeforeSingleSupportOuterY()
    */
   public abstract double getMaxICPErrorBeforeSingleSupportInnerY();

   /**
    * This parameter is used when the controller checks if it is safe to transition from transfer to
    * single support state when walking. The transition is considered safe if the ICP tracking error
    * lies within an ellipse with the axes aligned with the z-up ankle frame of the stance foot. This
    * parameter defines the radius of the ellipse along the y-axis of that frame.
    * </p>
    * Note that if the ICP leaves the support area the single support state will be started regardless
    * of the ICP error in the hope to recover by stepping.
    * </p>
    * 
    * @see #getMaxICPErrorBeforeSingleSupportForwardX()
    * @see #getMaxICPErrorBeforeSingleSupportBackwardX()
    * @see #getMaxICPErrorBeforeSingleSupportInnerY()
    */
   public double getMaxICPErrorBeforeSingleSupportOuterY()
   {
      return getMaxICPErrorBeforeSingleSupportInnerY();
   }

   /**
    * This parameter is used when the controller enters transfer state to check if the transfer
    * duration should be lengthened to give the controller more time to reduce the ICP tracking error
    * before going to single support.
    * 
    * @return the error threshold used to trigger longer transfer duration.
    */
   public double getInitialICPErrorToSlowDownTransfer()
   {
      return Double.POSITIVE_INFINITY;
   }

   /**
    * This parameter is used when the controller triggers a slow down of the transfer state, following
    * a large ICP error on entry. The new transfer duration is guaranteed to be greater or equal to
    * this parameter value.
    * 
    * @return the minimum transfer duration when slowing down the transfer state.
    */
   public double getMinimumSlowTransferDuration()
   {
      return 0.5;
   }

   /**
    * Determines whether the controller should always leave the single support state after the expected
    * single support time has passed. If set to {@code false} the controller will wait for the foot
    * switch to trigger the transition.
    */
   public boolean finishSingleSupportWhenICPPlannerIsDone()
   {
      return false;
   }

   /**
    * <ul>
    * </li>When {@code true}, single support continues until the ICP planner is done even if the swing
    * foot touches down, in which case the foot contact state is updated to be in contact. This kicks
    * in when touchdown occurs early and allows to transition to transfer as planned by the ICP planner
    * improving ICP plan continuity.
    * <li>When {@code false}, single support ends as soon as the swing foot touches down.
    * </ul>
    */
   public boolean waitInSingleSupportUntilICPPlannerIsDone()
   {
      return false;
   }

   /**
    * This is the duration for which the desired foot center of pressure will be drastically dampened
    * to calm shakies. This particularly useful when dealing with bad footholds. Set to -1.0 to
    * deactivate this feature.
    */
   public double getHighCoPDampingDurationToPreventFootShakies()
   {
      return -1.0;
   }

   /**
    * This is complimentary information to {@link #getHighCoPDampingDurationToPreventFootShakies()}.
    * The high CoP damping is triggered on large CoP tracking error. Set to
    * {@link Double#POSITIVE_INFINITY} to deactivate this feature.
    */
   public double getCoPErrorThresholdForHighCoPDamping()
   {
      return Double.POSITIVE_INFINITY;
   }

   /**
    * Determines whether the classes and variables for foothold exploration are created.
    */
   public boolean createFootholdExplorationTools()
   {
      return false;
   }

   /**
    * During normal execution the control algorithm computes a desired CMP. It is then projected in the
    * support polygon to avoid angular momentum of the upper body. When the robot is falling and
    * recovery is impossible otherwise, the support used for CMP projection can be increased and the
    * robot uses upper body momentum. This value defines the amount the support polygon for CMP
    * projection is increased in that case.
    *
    * @return maxAllowedDistanceCMPSupport
    */
   public double getMaxAllowedDistanceCMPSupport()
   {
      return Double.NaN;
   }

   /**
    * When true, some of the tracking performance will be degraded to reduce the generated angular
    * momentum rate around the vertical axis during swing only. Useful when the robot has heavy legs
    * and tends to slips during swing.
    */
   public boolean minimizeAngularMomentumRateZDuringSwing()
   {
      return false;
   }

   public boolean minimizeAngularMomentumRateZDuringTransfer()
   {
      return false;
   }

   /**
    * Returns a list of joint that should use the more restrictive joint limit enforcement in the QP.
    * If the list is not empty the method
    * {@link #getJointLimitParametersForJointsWithRestrictiveLimits(String)} must be overwritten to
    * define the limit parameters.
    */
   public String[] getJointsWithRestrictiveLimits()
   {
      return new String[0];
   }

   /**
    * Returns parameters for joint limits that will be used with the joints defined in
    * {@link #getJointsWithRestrictiveLimits()}.
    */
   public JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName)
   {
      return null;
   }

   /**
    * Determines whether the swing of the robot controls the toe point of the foot for better tracking
    * or not. (new feature to be tested with Atlas)
    */
   public boolean controlToeDuringSwing()
   {
      return false;
   }

   /**
    * Specifies whether or not the z-component of the swing initial angular velocity expressed in world
    * should be zeroed out.
    * <p>
    * This can be helpful in scenarios where a foot during toe-off causing a large velocity and
    * resulting in an undesired trajectory.
    * </p>
    *
    * @return whether the z-component swing initial angular velocity should be zeroed out or not.
    */
   public boolean ignoreSwingInitialAngularVelocityZ()
   {
      return false;
   }

   /**
    * Determines the maximum allowable magnitude for the swing initial linear velocity.
    * <p>
    * This can be helpful in scenarios where a foot during toe-off causing a large velocity and
    * resulting in an undesired trajectory.
    * </p>
    *
    * @return the swing initial linear velocity maximum magnitude.
    */
   public double getMaxSwingInitialLinearVelocityMagnitude()
   {
      return Double.POSITIVE_INFINITY;
   }

   /**
    * Determines the maximum allowable magnitude for the swing initial angular velocity.
    * <p>
    * This can be helpful in scenarios where a foot during toe-off causing a large velocity and
    * resulting in an undesired trajectory.
    * </p>
    *
    * @return the swing initial angular velocity maximum magnitude.
    */
   public double getMaxSwingInitialAngularVelocityMagnitude()
   {
      return Double.POSITIVE_INFINITY;
   }

   /**
    * Returns the parameters used in the privileged configuration handler.
    */
   public JointPrivilegedConfigurationParameters getJointPrivilegedConfigurationParameters()
   {
      return jointPrivilegedConfigurationParameters;
   }

   /**
    * Returns the parameters used for the knee privileged configuration
    */
   public OneDoFJointPrivilegedConfigurationParameters getKneePrivilegedConfigurationParameters()
   {
      return kneePrivilegedConfigurationParameters;
   }

   /**
    * Determines whether or not to attempt to directly control the height. If true, the height will be
    * controlled directly via a command to the controller core. This can be a linear momentum z command
    * or a feedback control command for the pelvis. If false, the height will be controlled inside the
    * nullspace of other objectives by trying to achieve the desired privileged configuration in the
    * legs.
    * 
    * @return boolean (true = control height, false = do not control height but leave it up to the
    *         optimization)
    */
   public boolean enableHeightFeedbackControl()
   {
      return true;
   }

   /**
    * Whether or not to use a secondary joint scaling factor during swing, where the secondary joint is
    * any joint located in the kinematic chain between the base and the optional primary base of a
    * SpatialAccelerationCommand and a SpatialVelocityCommand.
    */
   public boolean applySecondaryJointScaleDuringSwing()
   {
      return false;
   }

   /**
    * Returns {@link ToeOffParameters} that contain all parameters relevant to the toe off state when
    * walking.
    */
   public abstract ToeOffParameters getToeOffParameters();

   /**
    * Returns {@link SwingTrajectoryParameters} that contain all parameters relevant to the swing
    * trajectory.
    */
   public abstract SwingTrajectoryParameters getSwingTrajectoryParameters();

   public abstract ICPControllerParameters getICPControllerParameters();

   public abstract StepAdjustmentParameters getStepAdjustmentParameters();

   /**
    * Get the maximum leg length for the singularity avoidance control module.
    */
   public abstract double getMaximumLegLengthForSingularityAvoidance();

   /**
    * Parameter for the CoM height trajectory generation.
    */
   public abstract double minimumHeightAboveAnkle();

   /**
    * Parameter for the CoM height trajectory generation.
    */
   public abstract double nominalHeightAboveAnkle();

   /**
    * Parameter for the CoM height trajectory generation.
    */
   public abstract double maximumHeightAboveAnkle();

   /**
    * This is a reduction factor of {@link #maximumHeightAboveAnkle()} that is applied during the
    * exchange phase to the height trajectory when the robot is stepping down
    */
   public double getMaxLegLengthReductionSteppingDown()
   {
      return 0.0;
   }

   /**
    * If the step height change is above this value, it indicates that the foot should not be
    * considered "flat", and that the robot is either stepping up or down
    */
   public double getHeightChangeForNonFlatStep()
   {
      return 0.10;
   }

   /**
    * Whether the height of the pelvis should be controlled instead of the center of mass height.
    */
   public boolean controlPelvisHeightInsteadOfCoMHeight()
   {
      return true;
   }

   /**
    * Whether the height should be controlled with the rate of change of momentum or using a feedback
    * controller on the pelvis. Note that the height of the pelvis should be controlled to set this
    * flag to {@code false}, i.e. {@code controlPelvisHeightInsteadOfCoMHeight() == true}. Fixme does
    * this do what we think it does?
    */
   public boolean controlHeightWithMomentum()
   {
      return true;
   }

   /**
    * Returns parameters related to stepping such as maximum step length etc.
    */
   public abstract SteppingParameters getSteppingParameters();

   /**
    * If this return {@code false} the leg singularity avoidance module will not be created.
    */
   public boolean enableLegSingularityAndKneeCollapseAvoidanceModule()
   {
      return true;
   }

   /**
    * When taking steps that cause the swing foot to collide with the support leg this parameter can be
    * used to increase the clearance that the swing trajectory will have from the swing foot. More
    * specifically, when a straight line from swing start (at sole frame) to swing end point is closer
    * to the stance foot (sole frame) then this distance the swing trajectory will be adjusted.
    */
   public double getMinSwingTrajectoryClearanceFromStanceFoot()
   {
      return Double.NEGATIVE_INFINITY;
   }

   /**
    * A boolean to determine whether the CoM height manager should be created or not. If this returns
    * true the robot will use a rigid body manager to control the pelvis height only.
    */
   public boolean usePelvisHeightControllerOnly()
   {
      return false;
   }

   /**
    * Maximum velocity of the CoM height. Desired height velocity will be set to this if it is
    * exceeded. Not a very clean variable and probably should not be here, but here it is...
    */
   public double getMaximumVelocityCoMHeight()
   {
      return 0.25;
   }

   /**
    * When the swing state last longer than planned, i.e. touchdown is late, the height trajectory can
    * transition to a pseudo free fall trajectory meant to minimize the rate at which the capture point
    * diverges by reducing reducing the support foot ground reaction forces.
    * <p>
    * The transition to a free fall trajectory is initialized at
    * {@code t = swingTime + swingTimeOverrunDuration}, where {@code t} is the current time in the
    * swing state, {@code swingTime} is the planned swing duration, and {@code swingTimeOverrun} is the
    * fixed value obtained by this parameter.
    * </p>
    * <p>
    * Set this parameter to {@link Double#POSITIVE_INFINITY} to disable the transition to free fall.
    * </p>
    */
   public double getSwingTimeOverrunToInitializeFreeFall()
   {
      return Double.POSITIVE_INFINITY;
   }

   public NaturalPostureParameters getNaturalPostureParameters()
   {
      return null;
   }

   /**
    * When there's less than this fraction of swing remaining, the robot joints in swing should switch
    * to "load bearing". Essentially, for hydraulic robots, this switches them from velocity to
    * position controlled. This can be used to help alleviate heavy impacts at touchdown.
    */
   public double getFractionOfSwingToSwitchToLoaded()
   {
      return Double.NaN;
   }

   /**
    * Parameter for the duration of foot loading.
    */
   public double getLoadFootDuration()
   {
      return 1.2;
   }

   /**
    * Parameter for the duration of transfer from single support to double support.
    */
   public double getLoadFootTransferDuration()
   {
      return 0.8;
   }
}
