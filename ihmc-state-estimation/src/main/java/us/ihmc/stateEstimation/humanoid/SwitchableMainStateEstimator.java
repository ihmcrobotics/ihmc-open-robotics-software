package us.ihmc.stateEstimation.humanoid;

import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.stateEstimation.invariantEstimator.InvariantMainStateEstimator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Runs the DRC kinematics estimator and the invariant EKF side by side as selectable main estimators, so the
 * choice is a runtime switch instead of the compile-time flag it used to be.
 *
 * <p><b>Cold standby.</b> Both estimators are built at bring-up, but only the active one is ticked. That keeps
 * the cost identical to running one estimator (the joint-level KF is by far the most expensive thing in the
 * tick and only the invariant path owns one), at the price of the standby's state going stale — which the
 * hand-over at switch time repairs. Nothing needs to gate the model writes: both estimators rewrite the whole
 * shared {@code FullHumanoidRobotModel} every tick, and {@code HumanoidRobotContextTools.updateContext}
 * snapshots it after all controllers run, so "the active one is the only one that ran" is already "the active
 * one is the only writer".</p>
 *
 * <p><b>The hand-over.</b> Switching estimators mid-run means handing the robot from one filter to another
 * without stepping the state the controller sees. Two discontinuities have to be closed explicitly:</p>
 * <ul>
 *   <li><i>Heading.</i> The DRC estimator takes its yaw straight from the IMU, while the invariant filter
 *       integrates its own and drags it toward foot anchors; after minutes of walking the two disagree without
 *       bound. So the incoming DRC estimator is seeded with the outgoing yaw via
 *       {@link DRCKinematicsBasedStateEstimator#seedOrientation}. The other direction needs nothing: the
 *       invariant estimator takes orientation from the transform it is initialized with.</li>
 *   <li><i>Joint-level pre-filter.</i> The invariant path's KF carries a covariance across ticks, so after
 *       standing by cold its mean is stale against a converged (small) covariance. It is re-seeded from the
 *       outgoing estimator's state — joint q/q̇ plus its IMU bias estimates — with the covariance reset to its
 *       startup block-diagonal.</li>
 * </ul>
 *
 * <p><b>Gating.</b> A switch is applied only while the estimator is FROZEN (the controller already requests
 * FROZEN outside walking) and only after the previous switch has settled, because the incoming estimator's
 * foot switches are cold and need a contact window to fill. A request that arrives while the gate is shut is
 * <em>held, not dropped</em>, and applied as soon as the gate opens.</p>
 *
 * <p>{@link StateEstimatorMode} is orthogonal and is forwarded to <em>both</em> estimators, so the standby
 * never drifts out of sync with the controller's NORMAL/FROZEN requests.</p>
 *
 * @author Lucas Libshutz
 */
public class SwitchableMainStateEstimator implements StateEstimatorController
{
   /**
    * Ticks a freshly-activated estimator gets before another switch is allowed. Sized by the contact window of
    * the joint-torque foot switches, which are cold on the incoming estimator and need that long to be
    * meaningful.
    */
   private static final int DEFAULT_SWITCH_SETTLE_TICKS = 25;

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final EnumMap<MainStateEstimatorType, StateEstimatorController> estimators = new EnumMap<>(MainStateEstimatorType.class);
   private final DRCKinematicsBasedStateEstimator drcEstimator;
   private final InvariantMainStateEstimator invariantEstimator;

   /** The estimator selected at construction; owns the force-sensor accessors. See {@link #getForceSensorCalibrationModule()}. */
   private final StateEstimatorController bootEstimator;

   private final FloatingJointBasics rootJoint;
   private final RigidBodyTransform handoverTransform = new RigidBodyTransform();

   private final YoEnum<MainStateEstimatorType> activeMainEstimator;
   private final YoEnum<StateEstimatorMode> lastAppliedMode;
   private final YoBoolean allowSwitchWhileNotFrozen = new YoBoolean("allowMainEstimatorSwitchWhileNotFrozen", registry);
   private final YoBoolean mainEstimatorSwitchPending = new YoBoolean("mainEstimatorSwitchPending", registry);
   private final YoInteger ticksSinceMainEstimatorSwitch = new YoInteger("ticksSinceMainEstimatorSwitch", registry);
   private final YoInteger switchSettleTicks = new YoInteger("mainEstimatorSwitchSettleTicks", registry);

   /** Cross-thread inbox: requests arrive from the controller thread / ROS2 callbacks, applied on the estimator thread. */
   private final AtomicReference<MainStateEstimatorType> requestedMainEstimator = new AtomicReference<>(null);
   private final AtomicReference<StateEstimatorMode> requestedMode = new AtomicReference<>(null);

   /** Held request: survives a shut gate so a switch asked for mid-walk lands when the robot next freezes. */
   private MainStateEstimatorType heldRequest = null;

   public SwitchableMainStateEstimator(DRCKinematicsBasedStateEstimator drcEstimator,
                                       InvariantMainStateEstimator invariantEstimator,
                                       FloatingJointBasics rootJoint,
                                       MainStateEstimatorType initialEstimator,
                                       YoRegistry parentRegistry)
   {
      this.drcEstimator = drcEstimator;
      this.invariantEstimator = invariantEstimator;
      this.rootJoint = rootJoint;

      estimators.put(MainStateEstimatorType.DRC_KINEMATICS, drcEstimator);
      estimators.put(MainStateEstimatorType.INVARIANT_EKF, invariantEstimator);

      // No YoParameter may be read here: parameters are only loaded later, once the estimator registry is
      // assembled. Likewise nothing is initialized here -- initialize() runs on the first tick.
      activeMainEstimator = new YoEnum<>("activeMainStateEstimator", registry, MainStateEstimatorType.class, false);
      activeMainEstimator.set(initialEstimator);
      lastAppliedMode = new YoEnum<>("mainEstimatorLastAppliedMode", registry, StateEstimatorMode.class, false);
      lastAppliedMode.set(StateEstimatorMode.NORMAL);
      switchSettleTicks.set(DEFAULT_SWITCH_SETTLE_TICKS);
      ticksSinceMainEstimatorSwitch.set(Integer.MAX_VALUE / 2); // no switch yet: never block the first one

      bootEstimator = estimators.get(initialEstimator);

      registry.addChild(drcEstimator.getYoRegistry());
      registry.addChild(invariantEstimator.getYoRegistry());
      parentRegistry.addChild(registry);
   }

   /** The active estimator, i.e. the one that ticks and writes the shared robot model. */
   public StateEstimatorController getActiveEstimator()
   {
      return estimators.get(activeMainEstimator.getEnumValue());
   }

   /** Reaches a specific estimator regardless of which is active, for consumers that need the concrete instance. */
   public StateEstimatorController getEstimator(MainStateEstimatorType type)
   {
      return estimators.get(type);
   }

   public MainStateEstimatorType getActiveEstimatorType()
   {
      return activeMainEstimator.getEnumValue();
   }

   /**
    * Requests a switch. Thread-safe: callable from the controller thread or a ROS2 callback. The switch itself
    * happens on the estimator thread at the top of a tick, and only once the gate allows it.
    */
   public void requestMainStateEstimator(MainStateEstimatorType type)
   {
      requestedMainEstimator.set(type);
   }

   /**
    * Re-initializes the <em>active</em> estimator in place, from the pose it is currently reporting. This is
    * what the ROS2 re-initialize request should reach: routing it to a concrete estimator instead would set a
    * flag on one that may be standing by, which then fires an unexpected re-init whenever it is switched in.
    */
   public void requestReinitializeEstimator()
   {
      rootJoint.getJointConfiguration(handoverTransform);
      getActiveEstimator().initializeEstimator(handoverTransform, EMPTY_JOINT_POSITION_MAP);
   }

   @Override
   public void doControl()
   {
      applyRequestedMode();
      applyRequestedSwitch();

      getActiveEstimator().doControl();

      if (ticksSinceMainEstimatorSwitch.getValue() < Integer.MAX_VALUE / 2)
         ticksSinceMainEstimatorSwitch.increment();
   }

   /** Forwarded to both estimators so the standby never drifts out of sync with the controller's requests. */
   private void applyRequestedMode()
   {
      StateEstimatorMode requested = requestedMode.getAndSet(null);
      if (requested == null)
         return;

      lastAppliedMode.set(requested);
      drcEstimator.requestStateEstimatorMode(requested);
      invariantEstimator.requestStateEstimatorMode(requested);
   }

   private void applyRequestedSwitch()
   {
      MainStateEstimatorType requested = requestedMainEstimator.getAndSet(null);
      if (requested != null && requested != activeMainEstimator.getEnumValue())
         heldRequest = requested;

      mainEstimatorSwitchPending.set(heldRequest != null);

      if (heldRequest == null || !switchAllowed())
         return;

      switchTo(heldRequest);
      heldRequest = null;
      mainEstimatorSwitchPending.set(false);
   }

   /**
    * FROZEN means the base is being held rather than trusted for locomotion, which is the only safe moment to
    * hand the robot between filters; the settle count keeps two switches from overlapping while the incoming
    * estimator's foot switches are still filling.
    */
   private boolean switchAllowed()
   {
      boolean frozenOrOverridden = lastAppliedMode.getEnumValue() == StateEstimatorMode.FROZEN || allowSwitchWhileNotFrozen.getValue();
      return frozenOrOverridden && ticksSinceMainEstimatorSwitch.getValue() >= switchSettleTicks.getValue();
   }

   private void switchTo(MainStateEstimatorType incomingType)
   {
      // Read the hand-over pose FIRST: it is the outgoing estimator's last output, and seeding the incoming
      // estimator below will overwrite the root joint.
      rootJoint.getJointConfiguration(handoverTransform);

      StateEstimatorController incoming = estimators.get(incomingType);

      if (incomingType == MainStateEstimatorType.DRC_KINEMATICS)
      {
         // Close the heading gap before the re-init that initializeEstimator schedules consumes it.
         drcEstimator.seedOrientation(handoverTransform.getRotation());
      }
      else
      {
         // Hand the invariant path's joint-level KF a fresh mean (q, q̇ and the IMU biases the DRC estimator
         // has been maintaining) so it does not predict from a minutes-old state.
         invariantEstimator.seedPreFilterFromHandover(drcEstimator.getIMUBiasProvider());
      }

      // Only initializeEstimator, never initialize(): the DRC path's initialize() runs the full re-init a
      // second time and transiently writes a zeroed root pose into the shared model.
      incoming.initializeEstimator(handoverTransform, EMPTY_JOINT_POSITION_MAP);
      incoming.requestStateEstimatorMode(lastAppliedMode.getEnumValue());

      activeMainEstimator.set(incomingType);
      ticksSinceMainEstimatorSwitch.set(0);
   }

   @Override
   public void initialize()
   {
      getActiveEstimator().initialize();
   }

   @Override
   public void initializeEstimator(RigidBodyTransformReadOnly rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      getActiveEstimator().initializeEstimator(rootJointTransform, jointPositions);
   }

   @Override
   public void requestStateEstimatorMode(StateEstimatorMode operatingMode)
   {
      requestedMode.set(operatingMode);
   }

   /**
    * The boot estimator's, not the active one's. Only the DRC estimator implements these, and the
    * RobotConfigurationData publisher binds the result once at construction, so it cannot follow a runtime
    * switch anyway. Delegating to the boot estimator keeps force-sensor behaviour byte-identical to what the
    * previously compile-time-selected estimator produced.
    */
   @Override
   public ForceSensorCalibrationModule getForceSensorCalibrationModule()
   {
      return bootEstimator.getForceSensorCalibrationModule();
   }

   @Override
   public ForceSensorDataHolderReadOnly getForceSensorOutputWithGravityCancelled()
   {
      return bootEstimator.getForceSensorOutputWithGravityCancelled();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      List<YoGraphicDefinition> children = new ArrayList<>();
      for (MainStateEstimatorType type : MainStateEstimatorType.values)
      {
         YoGraphicDefinition graphics = estimators.get(type).getSCS2YoGraphics();
         if (graphics != null)
            children.add(graphics);
      }
      return children.isEmpty() ? null : new YoGraphicGroupDefinition(name, children);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }
}
