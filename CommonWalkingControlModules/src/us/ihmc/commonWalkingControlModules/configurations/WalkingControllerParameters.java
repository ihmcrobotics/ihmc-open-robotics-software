package us.ihmc.commonWalkingControlModules.configurations;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.ToeSlippingDetector;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOffsetTrajectoryWhileWalking;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationSettings;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.dynamicReachability.DynamicReachabilityCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class WalkingControllerParameters implements HeadOrientationControllerParameters, SteppingParameters
{
   private final StraightLegWalkingParameters straightLegWalkingParameters;
   private final JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;
   private final DynamicReachabilityParameters dynamicReachabilityParameters;
   private final PelvisOffsetWhileWalkingParameters pelvisOffsetWhileWalkingParameters;
   private final LeapOfFaithParameters leapOfFaithParameters;

   private final double massScale;

   public WalkingControllerParameters()
   {
      this(1.0);
   }

   public WalkingControllerParameters(double massScale)
   {
      this.massScale = massScale;

      jointPrivilegedConfigurationParameters = new JointPrivilegedConfigurationParameters();
      dynamicReachabilityParameters = new DynamicReachabilityParameters();
      pelvisOffsetWhileWalkingParameters = new PelvisOffsetWhileWalkingParameters();
      leapOfFaithParameters = new LeapOfFaithParameters();
      straightLegWalkingParameters = new StraightLegWalkingParameters();
   }


   /**
    * Specifies if the controller should by default compute for all the robot joints desired
    * position and desired velocity from the desired acceleration.
    * <p>
    * It is {@code false} by default and this method should be overridden to return otherwise.
    * </p>
    *
    * @return {@code true} if the desired acceleration should be integrated into desired velocity
    *         and position for all the joints.
    */
   public boolean enableJointAccelerationIntegrationForAllJoints()
   {
      return false;
   }

   /**
    * Returns a map from joint name to joint acceleration integration parameters.
    * <p>
    * Note that this method is only called if
    * {@link #enableJointAccelerationIntegrationForAllJoints()} returns {@code true}.
    * </p>
    * <p>
    * This method is called by the controller to know the set of joints for which specific
    * parameters are to be used.
    * </p>
    * <p>
    * If a joint is not added to this map, the default parameters will be used.
    * </p>
    *
    * @param registry the controller registry allowing to create {@code YoVariable}s for the
    *           parameters.
    * @return the map from the names of the joints with their specific parameters to use.
    */
   public Map<String, JointAccelerationIntegrationParametersReadOnly> getJointAccelerationIntegrationParameters(YoVariableRegistry registry)
   {
      /* @formatter:off
       * Example a robot for which we want to provide specific parameters for the elbow joints only:
       * Map<String, JointAccelerationIntegrationParametersReadOnly> jointParameters = new HashMap<>();
       * JointAccelerationIntegrationParametersReadOnly elbowParameters = new YoJointAccelerationIntegrationParameters("elbow", 0.999, 0.95, 0.1, 0.1, registry);
       * jointParameters.put("leftElbow", elbowParameters);
       * jointParameters.put("rightElbow", elbowParameters);
       * return jointParameters;
       *
       * Note that it is better to save the created Map as a field such that the next time this method is called, the same instance of the map is used.
       * @formatter:on
       */
      return null;
   }

   public abstract SideDependentList<RigidBodyTransform> getDesiredHandPosesWithRespectToChestFrame();

   public abstract String[] getDefaultChestOrientationControlJointNames();

   public abstract double getOmega0();

   public abstract double getAnkleHeight();

   public abstract double getLegLength();

   public abstract double minimumHeightAboveAnkle();

   public abstract double nominalHeightAboveAnkle();

   public abstract double maximumHeightAboveAnkle();

   public abstract double defaultOffsetHeightAboveAnkle();

   public abstract double pelvisToAnkleThresholdForWalking();

   public abstract double getTimeToGetPreparedForLocomotion();

   public boolean enableToeOffSlippingDetection()
   {
      return false;
   }

   public void configureToeSlippingDetector(ToeSlippingDetector toeSlippingDetectorToConfigure)
   {
      // Override this method to configure the parameters as follows:
//      double forceMagnitudeThreshold = 25.0;
//      double velocityThreshold = 0.4;
//      double slippageDistanceThreshold = 0.04;
//      double filterBreakFrequency = 10.0;
//      toeSlippingDetectorToConfigure.configure(forceMagnitudeThreshold, velocityThreshold, slippageDistanceThreshold, filterBreakFrequency);
   }

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

   /**
    * The map returned contains all controller gains for tracking jointspace trajectories. The key of
    * the map is the joint name as defined in the robot joint map. If a joint is not contained in the
    * map, jointspace control is not supported for that joint.
    *
    * @param registry used to create the gains the first time this function is called during a run
    * @return map containing jointspace PID gains by joint name
    */
   public Map<String, YoPIDGains> getOrCreateJointSpaceControlGains(YoVariableRegistry registry)
   {
      return new HashMap<String, YoPIDGains>();
   }

   /**
    * The map returned contains all controller gains for tracking taskspace orientation trajectories
    * (or the orientation part of a pose trajectory) for a rigid body. The key of the map is the rigid
    * body name as defined in the robot joint map. If a joint is not contained in the map, taskspace
    * orientation or pose control is not supported for that rigid body.
    *
    * @param registry used to create the gains the first time this function is called during a run
    * @return map containing taskspace orientation PID gains by rigid body name
    */
   public Map<String, YoOrientationPIDGainsInterface> getOrCreateTaskspaceOrientationControlGains(YoVariableRegistry registry)
   {
      return new HashMap<String, YoOrientationPIDGainsInterface>();
   }

   /**
    * The map returned contains all controller gains for tracking taskspace position trajectories
    * (or the position part of a pose trajectory) for a rigid body. The key of the map is the rigid
    * body name as defined in the robot joint map. If a joint is not contained in the map, taskspace
    * position or pose control is not supported for that rigid body.
    *
    * @param registry used to create the gains the first time this function is called during a run
    * @return map containing taskspace position PID gains by rigid body name
    */
   public Map<String, YoPositionPIDGainsInterface> getOrCreateTaskspacePositionControlGains(YoVariableRegistry registry)
   {
      return new HashMap<String, YoPositionPIDGainsInterface>();
   }

   /**
    * Returns the default control mode for a rigid body. The modes are defined in {@link RigidBodyControlMode}
    * and by default the mode should be {@link RigidBodyControlMode#JOINTSPACE}. In some cases (e.g. the chest)
    * it makes more sense to use the default mode {@link RigidBodyControlMode#TASKSPACE}.
    *
    * @param bodyName is the name of the {@link RigidBody}
    * @return the default control mode of the body
    */
   public RigidBodyControlMode getDefaultControlModeForRigidBody(String bodyName)
   {
      return RigidBodyControlMode.JOINTSPACE;
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
    * The key of the map is the name of the rigid body that can be obtained with {@link RigidBody#getName()}. If a
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
    * The list of strings returned contains all joint names that are position controlled. The names
    * of the joints are defined in the robots joint map.
    *
    * @return list of position controlled joint names
    */
   public List<String> getOrCreatePositionControlledJoints()
   {
      return new ArrayList<String>();
   }

   /**
    * The map returned contains the integration settings for position controlled joints. The settings
    * define how the controller core integrated desired accelerations to find desired joint positions
    * and velocities. The key of the map is the joint name as defined in the robot joint map. If a
    * joint is not contained in the map, position control is not supported for that joint.
    *
    * @return map containing acceleration integration settings by joint name
    */
   public Map<String, JointAccelerationIntegrationSettings> getOrCreateIntegrationSettings()
   {
      return new HashMap<String, JointAccelerationIntegrationSettings>();
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

   public abstract double getContactThresholdForce();

   public abstract double getSecondContactThresholdForceIgnoringCoP();

   /** Returns a map of neck joint names and associated min/max value joint limits. */
   public abstract LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits();

   public abstract SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits();

   public abstract double getCoPThresholdFraction();

   public abstract String[] getJointsToIgnoreInController();

   public abstract MomentumOptimizationSettings getMomentumOptimizationSettings();

   public abstract ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters();

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
    * Usually the desired CMP will be projected into the support area to avoid the generation of large amounts of
    * angular momentum. This method determines whether the desired CMP is allowed to be in area that is larger then
    * the support. The size of the area is determined by the value {@link WalkingControllerParameters#getMaxAllowedDistanceCMPSupport()}
    *
    * @return alwaysAllowMomentum
    */
   public boolean alwaysAllowMomentum()
   {
      return false;
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
    * Determines whether the robot should use the 'support state' or the 'fully constrained' & 'hold position' states (new feature to be tested with Atlas)
    */
   public boolean useSupportState()
   {
      return false;
   }

   /**
    *
    * Determines whether the robot should use the velocity to be computed in the estimator, or just compute it from the robot state in the controller (new feature to be tested with Atlas)
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

   @Override
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
      return jointPrivilegedConfigurationParameters;
   }

   /**
    * Returns the parameters used for straight leg walking
    */
   public StraightLegWalkingParameters getStraightLegWalkingParameters()
   {
      return straightLegWalkingParameters;
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
    * Sets whether or not the {@link DynamicReachabilityCalculator} will simply check whether or not the
    * upcoming step is reachable using the given step timing ({@return} is false), or will edit the step timings to make sure that the step is reachable
    * if ({@return} is true).
    *
    * @return whether or not to edit the timing based on the reachability of the step.
    */
   public boolean editStepTimingForReachability()
   {
      return false;
   }

   /**
    * Whether or not to use a secondary joint scaling factor during swing, where the secondary joint is any joint located in the kinematic chain between
    * the base and the optional primary base of a SpatialAccelerationCommand and a SpatialVelocityCommand.
    */
   public boolean applySecondaryJointScaleDuringSwing()
   {
      return false;
   }

   /**
    * Parameters for the {@link PelvisOffsetTrajectoryWhileWalking}
    */
   public PelvisOffsetWhileWalkingParameters getPelvisOffsetWhileWalkingParameters()
   {
      return pelvisOffsetWhileWalkingParameters;
   }

   public LeapOfFaithParameters getLeapOfFaithParameters()
   {
      return leapOfFaithParameters;
   }

   public abstract ToeOffParameters getToeOffParameters();

   public abstract SwingTrajectoryParameters getSwingTrajectoryParameters();
}
