package us.ihmc.stateEstimation.invariantEstimator;

import gnu.trove.map.TObjectDoubleMap;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.jointLevel.JointLevelKFPreFilter;
import us.ihmc.stateEstimation.jointLevel.ProprioceptivePreFilter;
import us.ihmc.stateEstimation.jointLevel.ZeroIMUBiasProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;


import java.util.ArrayList;
import java.util.List;

/**
 * Runs the contact-aided right-invariant InEKF as the <em>main</em> floating-base estimator.
 *
 * <p>Unlike {@link InvariantEKFStateEstimator} (which runs as a secondary/evaluation estimator and only
 * logs), this owns the responsibilities a main {@link StateEstimatorController} must fulfil for the
 * controller:
 * <ol>
 *   <li><b>Joints</b> — sets every one-DoF joint's q/q̇/τ directly from the processed sensor output and
 *       updates the model frames. This is the "good-enough FK" path: the same thing the DRC estimator's
 *       {@code JointStateUpdater} does, with no joint-space KF (that is the later P-A pre-filter). The
 *       InEKF's contact FK measurement noise stays the constant {@code contactMeasurementVariance} until
 *       that pre-filter exists.</li>
 *   <li><b>Floating base</b> — runs the InEKF predict + soft-contact update via a wrapped
 *       {@link InvariantEKFStateEstimator}.</li>
 *   <li><b>Yaw seeding</b> — optional {@link FootReferencedYawCorrector}, since heading is unobservable.</li>
 *   <li><b>Output</b> — writes the estimated root-joint pose and twist, mirroring how the DRC pelvis
 *       updaters write {@code rootJoint} (orientation/position setters, angular twist in body, linear twist
 *       via {@code setMatchingFrame} from world). The published context then carries the InEKF base to the
 *       controller.</li>
 *   <li><b>Center of mass</b> — runs an {@link InvariantCenterOfMassUpdater} (the {@code MomentumStateUpdater}
 *       seam) <em>after</em> the root-joint write, so CoM position/velocity are derived purely from the InEKF
 *       base and published to the controller via the shared {@link CenterOfMassDataHolder}. Velocity is
 *       kinematic (no GRF/centroidal fusion yet); see that class for the caveat and the upgrade seam.</li>
 * </ol>
 * The force-sensor data holder is intentionally <em>not</em> updated here: the sensor reader fills the
 * force-sensor holder the controller reads, mirroring the division of labour the EKF main estimator
 * ({@code HumanoidRobotEKFWithSimpleJoints}) relies on.</p>
 */
public class InvariantMainStateEstimator implements StateEstimatorController
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final FullHumanoidRobotModel fullRobotModel;
   private final SensorOutputMapReadOnly processedSensorOutput;
   private final OneDoFJointBasics[] oneDoFJoints;
   private final FloatingJointBasics rootJoint;

   /**
    * The anti-damping filter on the hip/knee pitch states handed to the controller (see
    * {@link ControllerFacingJointFilter}). The invariant filter computes from the unfiltered states; the filtered ones
    * are written into the shared model only after it ran, right before the root joint is published.
    */
   private final ControllerFacingJointFilter controllerFacingFilter;
   /** The joint states the estimator computed this tick, per index of {@link #oneDoFJoints}. */
   private final double[] estimatorJointPositions;
   private final double[] estimatorJointVelocities;
   /** The controller-facing joint states, per index of {@link #oneDoFJoints}. */
   private final double[] controllerFacingJointPositions;
   private final double[] controllerFacingJointVelocities;

   /**
    * Tells the controller-facing filter whether an RL controller state is active, where it is off by default
    * ({@code controllerFacingQdFilterInRL}). Wire to the high-level controller state.
    */
   public void setRLControllerStateActive(boolean active)
   {
      controllerFacingFilter.setRLStateActive(active);
   }

   private final HumanoidReferenceFrames referenceFrames;

   private final InvariantEKFStateEstimator invariantEstimator;
   private final FootReferencedYawCorrector yawCorrector;
   // Foot-referenced yaw seeding, switchable at runtime. On the 2026-11-05 mocap walks it was the source of the
   // InEKF heading drift (+11/+31/+44 deg over three walks with it, +1.5/+2.8/+5.7 without, against mocap):
   // a stance foot that twists or slips, or leg-FK yaw error, is read as pelvis yaw and nudged in every tick.
   private final YoBoolean yoYawSeedingEnabled;
   private boolean yawSeedingWasEnabled;
   private final InvariantCenterOfMassUpdater centerOfMassUpdater;

   private final ProprioceptivePreFilter preFilter; // may be null: raw sensor pass-through
   /** p(contact) above which a foot is handed to the pre-filter's phase 2 as trusted. */
   private static final double TRUSTED_FOOT_CONTACT_PROBABILITY_THRESHOLD = 0.5;
   /**
    * p(contact) at/above which, on BOTH feet, the joint-level pre-filter is allowed to seed itself: "the
    * robot is firmly on the ground." Deliberately high (not the 0.5 trust threshold) so the base-IMU gyro
    * bias is observable at seed time; not a literal 1.0 because the smoothed/logistic contact-probability
    * sources asymptote just below 1.0. See {@link JointLevelKFPreFilter#setInitializationGate}.
    */
   private static final double ON_GROUND_INIT_CONTACT_PROBABILITY_THRESHOLD = 0.9;
   private final List<RigidBodyBasics> trustedFeetForBiasUpdate = new ArrayList<>(2); // reused, no per-tick allocation
   private final SideDependentList<RigidBodyBasics> feet = new SideDependentList<>();

   // Root-joint write temporaries.
   private final RotationMatrix rootOrientation = new RotationMatrix();
   private final Vector3D rootPosition = new Vector3D();
   private final Vector3D baseVelocityWorld = new Vector3D();
   private final Vector3D angularVelocityBody = new Vector3D();
   private final FrameVector3D rootLinearVelocityWorld = new FrameVector3D();

   /**
    * @param fullRobotModel             the estimator's full robot model (joints set here, root written here).
    * @param processedSensorOutput      processed sensor outputs (IMU + joints).
    * @param primaryImuName             sensor name of the pelvis (base) IMU to use, e.g.
    *                                   {@code sensorInformation.getPrimaryBodyImu()}.
    * @param centerOfMassDataHolder     shared CoM holder, written each tick from the InEKF base estimate and
    *                                   published to the controller; may be {@code null} to skip CoM output.
    * @param dt                         estimator timestep Δt (s).
    * @param gyroVariance               continuous gyro noise variance σ_ω².
    * @param accelVariance              continuous accel noise variance σ_a².
    * @param contactVariance            continuous contact-slip noise variance σ_c² (base value at full contact).
    * @param contactMeasurementVariance per-axis body-frame contact FK measurement variance (m²).
    * @param initialCovariance          scalar for the initial P = initialCovariance · I.
    * @param gravitationalAcceleration  the robot process's gravitational acceleration (m/s²), i.e.
    *                                   {@code AvatarEstimatorThreadFactory.getGravity()}. Sign not considered.
    * @param enableYawSeeding           if true, install the foot-referenced yaw corrector.
    * @param preFilter                  joint-level pre-filter: joint estimates overlay the raw sensor
    *                                   values in {@link #updateJoints()} and its bias estimates feed
    *                                   the InEKF propagation; may be {@code null} for the raw
    *                                   pass-through (pre-seam behavior).
    */
   public InvariantMainStateEstimator(FullHumanoidRobotModel fullRobotModel,
                                      SensorOutputMapReadOnly processedSensorOutput,
                                      String primaryImuName,
                                      CenterOfMassDataHolder centerOfMassDataHolder,
                                      double dt,
                                      double gyroVariance,
                                      double accelVariance,
                                      double contactVariance,
                                      double contactMeasurementVariance,
                                      double initialCovariance,
                                      double gravitationalAcceleration,
                                      boolean enableYawSeeding,
                                      ProprioceptivePreFilter preFilter)
   {
      this.fullRobotModel = fullRobotModel;
      this.processedSensorOutput = processedSensorOutput;
      this.oneDoFJoints = fullRobotModel.getOneDoFJoints();
      this.rootJoint = fullRobotModel.getRootJoint();
      this.preFilter = preFilter;

      this.referenceFrames = new HumanoidReferenceFrames(fullRobotModel);

      for (RobotSide side : RobotSide.values)
         feet.put(side, fullRobotModel.getFoot(side));

      invariantEstimator = new InvariantEKFStateEstimator(fullRobotModel,
                                                          processedSensorOutput,
                                                          primaryImuName,
                                                          preFilter != null ? preFilter : new ZeroIMUBiasProvider(),
                                                          dt,
                                                          gyroVariance,
                                                          accelVariance,
                                                          contactVariance,
                                                          contactMeasurementVariance,
                                                          initialCovariance,
                                                          gravitationalAcceleration);
      invariantEstimator.setRunningAsMain(true); // disables the invariantMinusMain* self-comparisons
      if (preFilter != null)
         invariantEstimator.setKinematicVelocityJointCovarianceSource(preFilter, fullRobotModel.getPelvis(), feet);
      registry.addChild(invariantEstimator.getYoRegistry());

      // 2026-07-16 anti-damping filter on the controller-facing hip/knee pitch states; see ControllerFacingJointFilter.
      List<String> filteredJointNames = new ArrayList<>();
      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         if (ControllerFacingJointFilter.isHipOrKneePitch(joint.getName()))
            filteredJointNames.add(joint.getName());
      }
      controllerFacingFilter = new ControllerFacingJointFilter(filteredJointNames, dt, registry);
      estimatorJointPositions = new double[oneDoFJoints.length];
      estimatorJointVelocities = new double[oneDoFJoints.length];
      controllerFacingJointPositions = new double[oneDoFJoints.length];
      controllerFacingJointVelocities = new double[oneDoFJoints.length];

      // Defer the joint-level KF's initialization until both feet are firmly in contact. Seeding it while the
      // robot hangs leaves the exported base-IMU gyro bias unobservable (its only anchor is the phase-2 stance
      // update, which is off with no trusted foot), so the bias wanders and this estimator integrates it into a
      // rotating base. The gate reads the same contact probability the trust decision uses; other pre-filter
      // types ignore setInitializationGate (default no-op), so only the JOINT_KF path is affected.
      ProprioceptivePreFilter unwrappedPreFilter =
            preFilter instanceof us.ihmc.stateEstimation.jointLevel.SwitchableJointLevelSource switchable ? switchable.getJointKF() : preFilter;
      if (unwrappedPreFilter instanceof JointLevelKFPreFilter jointLevelKF)
      {
         invariantEstimator.setKinematicVelocityLegViolationSource(jointLevelKF::getLegOffAxisViolation);
         jointLevelKF.setInitializationGate(() ->
                  invariantEstimator.getContactProbability(RobotSide.LEFT) >= ON_GROUND_INIT_CONTACT_PROBABILITY_THRESHOLD
               && invariantEstimator.getContactProbability(RobotSide.RIGHT) >= ON_GROUND_INIT_CONTACT_PROBABILITY_THRESHOLD);
      }

      // Always built so it can be switched on at runtime; enableYawSeeding only sets the initial state.
      SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         soleFrames.put(side, referenceFrames.getSoleFrame(side));
      yawCorrector = new FootReferencedYawCorrector(invariantEstimator.getInvariantEKF(),
                                                    referenceFrames.getPelvisFrame(),
                                                    soleFrames,
                                                    invariantEstimator::getContactProbability,
                                                    registry);
      yoYawSeedingEnabled = new YoBoolean("invariantYawSeedingEnabled", registry);
      yoYawSeedingEnabled.set(enableYawSeeding);
      yawSeedingWasEnabled = enableYawSeeding;

      centerOfMassUpdater = new InvariantCenterOfMassUpdater(rootJoint, centerOfMassDataHolder);
      registry.addChild(centerOfMassUpdater.getRegistry());
   }

   @Override
   public void doControl()
   {
      if (preFilter != null)
         preFilter.computeJointState(); // phase 1: before joint outputs are consumed

      updateJoints();
      referenceFrames.updateFrames();

      invariantEstimator.doControl();

      boolean yawSeeding = yoYawSeedingEnabled.getValue();
      if (yawSeeding && !yawSeedingWasEnabled)
         yawCorrector.reset(); // anchors taken while disabled are stale; re-anchor at the next touchdown
      yawSeedingWasEnabled = yawSeeding;
      if (yawSeeding)
         yawCorrector.correct();

      // The invariant filter has run on the estimator's joint states; from here on the model carries what the
      // controller reads.
      applyControllerFacingJointStates();
      writeRootJoint();

      // CoM position/velocity for the controller, derived from the InEKF base just written to the root joint.
      centerOfMassUpdater.update();

      // Phase 2: after the trust decision. On this pipeline the trust currency is the contact
      // probability just refreshed by invariantEstimator.doControl(). Corrections computed here are
      // consumed at the top of the NEXT tick — one tick of latency, mirroring the DRC estimator;
      // do not "fix" this into a same-tick circular dependency on the trust decision.
      if (preFilter != null)
      {
         trustedFeetForBiasUpdate.clear();
         for (RobotSide side : RobotSide.values)
         {
            if (invariantEstimator.getContactProbability(side) > TRUSTED_FOOT_CONTACT_PROBABILITY_THRESHOLD)
               trustedFeetForBiasUpdate.add(feet.get(side));
         }
         preFilter.computeImuBiases(trustedFeetForBiasUpdate);
      }
   }

   /**
    * Sets q/q̇/τ for every one-DoF joint from the processed sensor output — overlaid with the
    * pre-filter's estimates where available (NaN falls back to the raw value, same contract as
    * {@code JointStateUpdater}) — and refreshes the model frames.
    */
   private void updateJoints()
   {
      controllerFacingFilter.updateGate();
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         OneDoFJointStateReadOnly jointOutput = processedSensorOutput.getOneDoFJointOutput(joint);

         double position = jointOutput.getPosition();
         double velocity = jointOutput.getVelocity();

         if (preFilter != null && preFilter.containsJoint(joint))
         {
            double estimatedPosition = preFilter.getEstimatedJointPosition(joint);
            if (!Double.isNaN(estimatedPosition))
               position = estimatedPosition;

            double estimatedVelocity = preFilter.getEstimatedJointVelocity(joint);
            if (!Double.isNaN(estimatedVelocity))
               velocity = estimatedVelocity;
         }

         estimatorJointPositions[i] = position;
         estimatorJointVelocities[i] = velocity;
         controllerFacingFilter.update(joint.getName(), position, velocity);
         controllerFacingJointPositions[i] = controllerFacingFilter.getFilteredPosition();
         controllerFacingJointVelocities[i] = controllerFacingFilter.getFilteredVelocity();

         // The estimator computes from its own, full-bandwidth states. Only the A/B switch hands it the filtered ones.
         boolean estimatorReadsFiltered = controllerFacingFilter.isInvariantFilterUsingFilteredStates();
         joint.setQ(estimatorReadsFiltered ? controllerFacingJointPositions[i] : position);
         joint.setQd(estimatorReadsFiltered ? controllerFacingJointVelocities[i] : velocity);
         joint.setTau(jointOutput.getEffort());
      }
      fullRobotModel.getElevator().updateFramesRecursively();
   }

   /**
    * Writes the controller-facing joint states into the shared model, for everything the controller reads from here on
    * (root-joint frames, CoM, the published context). Called once the estimator is done with this tick's joint states.
    */
   private void applyControllerFacingJointStates()
   {
      if (!controllerFacingFilter.isActive() || controllerFacingFilter.isInvariantFilterUsingFilteredStates())
         return; // already in the model: the estimator's states, or the filtered ones under the A/B switch
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         if (!controllerFacingFilter.filtersJoint(joint.getName()))
            continue;
         joint.setQ(controllerFacingJointPositions[i]);
         joint.setQd(controllerFacingJointVelocities[i]);
      }
      fullRobotModel.getElevator().updateFramesRecursively();
   }

   /** Publishes the InEKF base estimate to the root joint (mirrors the DRC pelvis updaters). */
   private void writeRootJoint()
   {
      InvariantEKF ekf = invariantEstimator.getInvariantEKF();
      ekf.getRotation(rootOrientation);
      ekf.getBasePosition(rootPosition);
      ekf.getBaseVelocity(baseVelocityWorld);
      invariantEstimator.getMeasuredAngularVelocityInBody(angularVelocityBody);

      rootJoint.setJointOrientation(rootOrientation);
      rootJoint.setJointPosition(rootPosition);
      rootJoint.updateFrame(); // refresh the after-joint frame so the world→body twist conversion is correct

      // Angular part: body-frame gyro (same physical frame as the joint's after-joint/pelvis frame).
      rootJoint.getJointTwist().getAngularPart().set(angularVelocityBody);
      // Linear part: world-frame base velocity, converted into the twist's (body) frame.
      rootLinearVelocityWorld.setIncludingFrame(ReferenceFrame.getWorldFrame(), baseVelocityWorld);
      rootJoint.getJointTwist().getLinearPart().setMatchingFrame(rootLinearVelocityWorld);

      rootJoint.updateFramesRecursively();
   }

   @Override
   public void initialize()
   {
      if (preFilter != null)
      {
         preFilter.initialize();
         preFilter.computeJointState(); // phase 1 before every updateJoints()
      }
      updateJoints();
      referenceFrames.updateFrames();
      invariantEstimator.initialize();
      applyControllerFacingJointStates();
      centerOfMassUpdater.initialize();
   }

   @Override
   public void initializeEstimator(RigidBodyTransformReadOnly rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      if (preFilter != null)
         preFilter.computeJointState(); // phase 1 before every updateJoints()
      updateJoints();
      rootJoint.setJointOrientation(rootJointTransform.getRotation());
      rootJoint.setJointPosition(rootJointTransform.getTranslation());
      rootJoint.updateFramesRecursively();
      referenceFrames.updateFrames();

      invariantEstimator.initializeEstimator(rootJointTransform, jointPositions);

      yawCorrector.reset();

      applyControllerFacingJointStates();
      centerOfMassUpdater.initialize();
   }

   @Override
   public void requestStateEstimatorMode(StateEstimatorMode operatingMode)
   {
      invariantEstimator.requestStateEstimatorMode(operatingMode);
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

   @Override
   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(name);
      group.addChild(invariantEstimator.getSCS2YoGraphics());
      return group;
   }

   /** @return the wrapped InEKF evaluation estimator (base estimate + soft contact + YoVariables). */
   public InvariantEKFStateEstimator getInvariantEKFStateEstimator()
   {
      return invariantEstimator;
   }
}
