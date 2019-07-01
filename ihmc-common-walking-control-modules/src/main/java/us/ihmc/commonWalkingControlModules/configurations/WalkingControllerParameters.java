package us.ihmc.commonWalkingControlModules.configurations;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisICPBasedTranslationManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ToeSlippingDetector;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOffsetTrajectoryWhileWalking;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.dynamicReachability.DynamicReachabilityCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PID3DConfiguration;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.sensors.FootSwitchFactory;

public abstract class WalkingControllerParameters
{
   private final LegConfigurationParameters legConfigurationParameters;
   private final JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;
   private final DynamicReachabilityParameters dynamicReachabilityParameters;
   private final PelvisOffsetWhileWalkingParameters pelvisOffsetWhileWalkingParameters;
   private final LeapOfFaithParameters leapOfFaithParameters;

   public WalkingControllerParameters()
   {
      jointPrivilegedConfigurationParameters = new JointPrivilegedConfigurationParameters();
      dynamicReachabilityParameters = new DynamicReachabilityParameters();
      pelvisOffsetWhileWalkingParameters = new PelvisOffsetWhileWalkingParameters();
      leapOfFaithParameters = new LeapOfFaithParameters();
      legConfigurationParameters = new LegConfigurationParameters();
   }

   /**
    * Returns the value of sqrt(g / z0) which corresponds to omega0 in the Linear Inverted Pendulum
    * Model that the ICP is based on. Note, that this value is a tuning parameter for each robot and
    * is not computed from the actual CoM height.
    *
    * @return the value for omega0 that is used in the controller for ICP related computations.
    */
   public abstract double getOmega0();

   /**
    * Specifies if the desired ground reaction force for the force that is about to swing should
    * smoothly be brought to zero by adding a inequality constraint on the z-force.
    * 
    * @return whether to perform smooth unloading before swing or not. Default value is
    *         {@code false}.
    */
   public boolean enforceSmoothFootUnloading()
   {
      return false;
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

   /**
    * Method returns robot specific parameters for the {@link #ToeSlippingDetector}.
    * <p>
    * Must be overwritten if {@link #enableToeOffSlippingDetection()} returns {@code true}.
    * </p>
    * @return the parameters for slip detection during toe off.
    * @see ToeSlippingDetector#configure(double, double, double, double)
    */
   public ToeSlippingDetectorParameters getToeSlippingDetectorParameters()
   {
      return null;
   }

   /**
    * If the return value is {@code true} the controller will speed up the swing of a foot when walking to match
    * the desired ICP to the current ICP. See {@link #getICPErrorThresholdToSpeedUpSwing()} to specify the threshold
    * on the ICP error which will cause a swing speedup.
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
    * Determines the threshold on the ICP error that will cause the controller to speed up the swing when in single
    * support. Note that this will only have an effect if {@link #allowDisturbanceRecoveryBySpeedingUpSwing()} returns
    * {@code true}.
    *
    * @return the threshold on the ICP error to trigger swing speedup
    */
   public abstract double getICPErrorThresholdToSpeedUpSwing();

   /**
    * Specifies whether the controller will abort any arm trajectories when close to loosing its balance. This is
    * determined by the ICP tracking error.
    *
    * @return whether the robot will abort arm trajectories when the ICP error is large
    */
   public abstract boolean allowAutomaticManipulationAbort();

   /**
    * Determines the threshold for the ICP tracking error that will cause the robot do abort manipulation if
    * {@link #allowAutomaticManipulationAbort()} is returning {@code true}.
    *
    * @return the threshold on the ICP error that will trigger manipulation abort
    * @see #allowAutomaticManipulationAbort()
    */
   public double getICPErrorThresholdForManipulationAbort()
   {
      return 0.04;
   }

   /**
    * This parameter sets the buffer around the support polygon to constrain the offset ICP used in {@link PelvisICPBasedTranslationManager}. It's defined in meters.
    */
   public double getPelvisTranslationICPSupportPolygonSafeMargin()
   {
      return 0.04;
   }

   /**
    * The desired position of the CMP is computed based on a feedback control law on the ICP. This method returns
    * the gains used in this controller.
    */
   public abstract ICPControlGains createICPControlGains();

   /**
    * This method returns the gains used in the controller to regulate the center of mass height.
    */
   public abstract PDGains getCoMHeightControlGains();

   /**
    * Returns a list of joint control gains for groups of joints.
    * <p>
    * Each {@link GroupParameter} contains gains for one joint group:</br>
    *  - The name of the joint group that the gain is used for (e.g. Arms).</br>
    *  - The gains for the joint group.</br>
    *  - The names of all rigid bodies in the joint group.
    * </p>
    * If a joint is not contained in the list, jointspace control is not supported
    * for that joint.
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
    *  - The name of the body group that the gain is used for (e.g. Hands).</br>
    *  - The gains for the body group.</br>
    *  - The names of all rigid bodies in the body group.
    * </p>
    * If a body is not contained in the list, taskspace orientation or pose control is not
    * supported for that rigid body. These gains will be used by the controller for tracking
    * taskspace orientation trajectories (or the orientation part of a pose trajectory) for a
    * rigid body.
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
    *  - The name of the body group that the gain is used for (e.g. Hands).</br>
    *  - The gains for the body group.</br>
    *  - The names of all rigid bodies in the body group.
    * </p>
    * If a body is not contained in the list, taskspace position or pose control is not
    * supported for that rigid body. These gains will be used by the controller for tracking
    * taskspace position trajectories (or the position part of a pose trajectory) for a
    * rigid body.
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
    * The key of the map is the rigid body name as defined in the joint map. Possible
    * control modes are defined in {@link RigidBodyControlMode}. By default (if a body
    * is not contained in the map) {@link RigidBodyControlMode#JOINTSPACE} will be used
    * for the body. In some cases (e.g. the chest) it makes more sense to use the default
    * mode {@link RigidBodyControlMode#TASKSPACE}.
    * </p>
    *
    * @return the default control mode of the body
    */
   public Map<String, RigidBodyControlMode> getDefaultControlModesForRigidBodies()
   {
      return new HashMap<>();
   }

   /**
    * The map returned contains the default home joint angles. The key of the map is the joint name
    * as defined in the robot joint map.
    *
    * @return map containing home joint angles by joint name
    */
   public TObjectDoubleHashMap<String> getOrCreateJointHomeConfiguration()
   {
      return new TObjectDoubleHashMap<String>();
   }

   /**
    * The map returned contains the default rigid body poses in their respective base frame. For example, if the base
    * frame of the chest body is the pelvis z-up frame this should contain the home pose of the chest in that frame.
    * If the particular body does not support full pose control but only orientation control the position part of the
    * pose will be disregarded.
    * <p>
    * The key of the map is the name of the rigid body that can be obtained with {@link RigidBodyBasics#getName()}. If a
    * body is not contained in this map but a default control mode of {@link RigidBodyControlMode#TASKSPACE} is not
    * supported for that body.
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
    * Returns the gains used for the foot when in support. Note that these gains are only used when the foot
    * is not loaded or close to tipping. Of that is not the case the foot pose when in support is not controlled
    * using a feedback controller.
    */
   public abstract PIDSE3Configuration getHoldPositionFootControlGains();

   /**
    * Returns the gains used for the foot when in the toe off state. Note that some parts of the foot orientation
    * will not use these gains. The foot pitch for example is usually not controlled explicitly during tow off.
    */
   public abstract PIDSE3Configuration getToeOffFootControlGains();

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
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

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
    * Ramps up the maximum loading of the normal force of the toe contact points over time, if returns true. If returns false, it simply
    * immediately sets the normal force maximum to infinity.
    *
    * @return whether or not to ramp up.
    */
   public boolean rampUpAllowableToeLoadAfterContact()
   {
      return false;
   }

   /**
    * Defines the duration spent ramping up the allowable normal toe contact force if {@link #rampUpAllowableToeLoadAfterContact()} is true.
    *
    * @return duration (s)
    */
   public double getToeLoadingDuration()
   {
      return 0.2;
   }

   /**
    * The maximum normal force allowed in the toe if {@link #rampUpAllowableToeLoadAfterContact()} is true at the time returned by
    * {@link #getToeLoadingDuration()}. After this time, the maximum normal force goes to infinity.
    * @return
    */
   public double getFullyLoadedToeForce()
   {
      return 1.0e3;
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

   public abstract FootSwitchFactory getFootSwitchFactory();

   /**
    * Returns a list of joints that will not be used by the controller.
    */
   public abstract String[] getJointsToIgnoreInController();

   /**
    * Returns the {@link MomentumOptimizationSettings} for this robot. These parameters define the weights
    * given to the objectives of the walking controller in the QP.
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
    * Returns the {@link ICPAngularMomentumModifierParameters} for this robot. The parameters are used when
    * angular momentum rates are considered in the ICP planner.
    */
   public abstract ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters();

   /**
    * This parameter is used when the controller checks if it is safe to transition from transfer to single
    * support state when walking. The transition is considered safe if the ICP tracking error lies within
    * an ellipse with the axes aligned with the z-up ankle frame of the stance foot. This parameter defines
    * the radius of the ellipse along the x-axis of that frame.
    * </p>
    * Note that if the ICP leaves the support area the single support state will be started regardless of the
    * ICP error in the hope to recover by stepping.
    * </p>
    * @see #getMaxICPErrorBeforeSingleSupportY()
    */
   public abstract double getMaxICPErrorBeforeSingleSupportX();

   /**
    * This parameter is used when the controller checks if it is safe to transition from transfer to single
    * support state when walking. The transition is considered safe if the ICP tracking error lies within
    * an ellipse with the axes aligned with the z-up ankle frame of the stance foot. This parameter defines
    * the radius of the ellipse along the y-axis of that frame.
    * </p>
    * Note that if the ICP leaves the support area the single support state will be started regardless of the
    * ICP error in the hope to recover by stepping.
    * </p>
    * @see #getMaxICPErrorBeforeSingleSupportX()
    */
   public abstract double getMaxICPErrorBeforeSingleSupportY();

   /**
    * Determines whether the controller should always leave the single support state after the expected
    * single support time has passed. If set to {@code false} the controller will wait for the foot switch to
    * trigger the transition.
    */
   public boolean finishSingleSupportWhenICPPlannerIsDone()
   {
      return false;
   }

   /**
    * This is the duration for which the desired foot center of pressure will be
    * drastically dampened to calm shakies. This particularly useful when
    * dealing with bad footholds.
    * Set to -1.0 to deactivate this feature.
    */
   public double getHighCoPDampingDurationToPreventFootShakies()
   {
      return -1.0;
   }

   /**
    * This is complimentary information to {@link #getHighCoPDampingDurationToPreventFootShakies()}.
    * The high CoP damping is triggered on large CoP tracking error.
    * Set to {@link Double#POSITIVE_INFINITY} to deactivate this feature.
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
    * Determines whether the robot should use the velocity to be computed in the estimator, or just compute it from the robot state in the
    * controller (new feature to be tested with Atlas)
    */
   public boolean useCenterOfMassVelocityFromEstimator()
   {
      return false;
   }

   /**
    * Returns a list of joint that should use the more restrictive joint limit enforcement
    * in the QP. If the list is not empty the method {@link #getJointLimitParametersForJointsWithRestictiveLimits()}
    * must be overwritten to define the limit parameters.
    */
   public String[] getJointsWithRestrictiveLimits()
   {
      return new String[0];
   }

   /**
    * Returns parameters for joint limits that will be used with the joints defined in
    * {@link #getJointsWithRestrictiveLimits()}.
    */
   public JointLimitParameters getJointLimitParametersForJointsWithRestictiveLimits()
   {
      return null;
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
    * Returns the parameters used for straight leg walking
    */
   public LegConfigurationParameters getLegConfigurationParameters()
   {
      return legConfigurationParameters;
   }

   /**
    * Returns the parameters in the dynamic reachability calculator.
    */
   public DynamicReachabilityParameters getDynamicReachabilityParameters()
   {
      return dynamicReachabilityParameters;
   }

   /**
    * Determines whether or not to attempt to directly control the height.
    * If true, the height will be controlled directly via a command to the controller core. This can be
    * a linear momentum z command or a feedback control command for the pelvis.
    * If false, the height will be controlled inside the nullspace of other objectives by trying to achieve
    * the desired privileged configuration in the legs.
    * @return boolean (true = control height, false = do not control height but leave it up to the optimization)
    */
   public boolean enableHeightFeedbackControl()
   {
      return true;
   }

   /**
    * Sets whether or not the {@link DynamicReachabilityCalculator} will simply check whether or not the
    * upcoming step is reachable using the given step timing ({@return} is false), or will edit the step timings
    * to make sure that the step is reachable if ({@return} is true).
    *
    * @return whether or not to edit the timing based on the reachability of the step.
    */
   public boolean editStepTimingForReachability()
   {
      return false;
   }

   /**
    * Whether or not to use a secondary joint scaling factor during swing, where the secondary joint is any joint
    * located in the kinematic chain between the base and the optional primary base of a SpatialAccelerationCommand
    * and a SpatialVelocityCommand.
    */
   public boolean applySecondaryJointScaleDuringSwing()
   {
      return false;
   }

   /**
    * Parameters for the {@link PelvisOffsetTrajectoryWhileWalking}. These parameters can be used to
    * shape the pelvis orientation trajectory while walking to create a more natural motion and
    * improve foot reachability.
    */
   public PelvisOffsetWhileWalkingParameters getPelvisOffsetWhileWalkingParameters()
   {
      return pelvisOffsetWhileWalkingParameters;
   }

   /**
    * Parameters for the 'Leap of Faith' Behavior. This caused the robot to activly fall onto an upcoming
    * foothold when necessary to reach an upcoming foothold. This method returns the robot specific
    * implementation of the {@link LeapOfFaithParameters};
    */
   public LeapOfFaithParameters getLeapOfFaithParameters()
   {
      return leapOfFaithParameters;
   }

   /**
    * Returns {@link ToeOffParameters} that contain all parameters relevant to the toe off state when walking.
    */
   public abstract ToeOffParameters getToeOffParameters();

   /**
    * Returns {@link SwingTrajectoryParameters} that contain all parameters relevant to the swing trajectory.
    */
   public abstract SwingTrajectoryParameters getSwingTrajectoryParameters();

   public abstract ICPOptimizationParameters getICPOptimizationParameters();

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
    * Parameter for the CoM height trajectory generation.
    */
   @Deprecated // Remove this. It is not actually doing anything.
   public abstract double defaultOffsetHeightAboveAnkle();

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
    * When taking steps that cause the swing foot to collide with the support leg this parameter can
    * be used to increase the clearance that the swing trajectory will have from the swing foot. More
    * specifically, when a straight line from swing start (at sole frame) to swing end point is closer
    * to the stance foot (sole frame) then this distance the swing trajectory will be adjusted.
    */
   public double getMinSwingTrajectoryClearanceFromStanceFoot()
   {
      return Double.NEGATIVE_INFINITY;
   }

   /**
    * A robot can implement an ankle IK solver. Optionally, the walking controller will add desired
    * joint angles and velocities for the ankle to the output of the controller core. Depending on the
    * implementation of the robots joint control this can be used to better track the foot pose on a
    * robot. The desired torque and acceleration computed by the whole body controller will still
    * be available. Note, that the output of this module might be inconsistent with the output of the
    * whole body controller as it does not consider other objectives such as balancing.
    */
   public AnkleIKSolver getAnkleIKSolver()
   {
      return null;
   }

   /**
    * A boolean to determine whether the CoM height manager should be created or not. If this returns true
    * the robot will use a rigid body manager to control the pelvis height only.
    */
   public boolean usePelvisHeightControllerOnly()
   {
      return false;
   }
}
