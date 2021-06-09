package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOffsetTrajectoryWhileWalking;
import us.ihmc.commonWalkingControlModules.dynamicReachability.DynamicReachabilityCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.feedbackController.FeedbackControllerSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;

public abstract class PushRecoveryControllerParameters
{
   private final LegConfigurationParameters legConfigurationParameters;
   private final JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;
   private final DynamicReachabilityParameters dynamicReachabilityParameters;

   public PushRecoveryControllerParameters()
   {
      jointPrivilegedConfigurationParameters = new JointPrivilegedConfigurationParameters();
      dynamicReachabilityParameters = new DynamicReachabilityParameters();
      legConfigurationParameters = new LegConfigurationParameters();
   }

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

   /**
    * This is the minimum swing time that the controller will allow when adjusting transfer times to achieve certain step
    * times in footstep plans.
    */
   public double getMinimumSwingTime()
   {
      return 0.8;
   }

   /**
    * Returns the {@link MomentumOptimizationSettings} for this robot. These parameters define the weights
    * given to the objectives of the walking controller in the QP.
    */
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return null;
   }

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
    * The high CoP damping is triggered on large CoP tracking error.
    * Set to {@link Double#POSITIVE_INFINITY} to deactivate this feature.
    */
   public double getCoPErrorThresholdForHighCoPDamping()
   {
      return Double.POSITIVE_INFINITY;
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

   public boolean minimizeAngularMomentumRateZDuringTransfer()
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
    * in the QP. If the list is not empty the method {@link #getJointLimitParametersForJointsWithRestrictiveLimits(String)}
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
   public JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName)
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
    * Returns {@link ToeOffParameters} that contain all parameters relevant to the toe off state when walking.
    */
   public ToeOffParameters getToeOffParameters()
   {
      return null;
   }

   /**
    * Returns {@link SwingTrajectoryParameters} that contain all parameters relevant to the swing trajectory.
    */
   public SwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return null;
   }

   public ICPOptimizationParameters getICPOptimizationParameters()
   {
      return null;
   }

   /**
    * Get the maximum leg length for the singularity avoidance control module.
    */
   public double getMaximumLegLengthForSingularityAvoidance()
   {
      return 0.0;
   }

   /**
    * Parameter for the CoM height trajectory generation.
    */
   public double minimumHeightAboveAnkle()
   {
      return 0.0;
   }

   /**
    * Parameter for the CoM height trajectory generation.
    */
   public double nominalHeightAboveAnkle()
   {
      return 0.0;
   }

   /**
    * Parameter for the CoM height trajectory generation.
    */
   public double maximumHeightAboveAnkle()
   {
      return 0.0;
   }


   /**
    * Returns parameters related to stepping such as maximum step length etc.
    */
   public SteppingParameters getSteppingParameters()
   {
      return null;
   }

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

   /**
    * Maximum velocity of the CoM height. Desired height velocity will be set to this if it is exceeded.
    * Not a very clean variable and probably should not be here, but here it is...
    */
   public double getMaximumVelocityCoMHeight()
   {
      return 0.25;
   }
}
