package us.ihmc.stateEstimation.invariant_estimator;

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

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

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

   // Anti-damping notch (2026-07-16): center at the measured mode, Q=2 → ~6 Hz-wide stopband,
   // <5° phase at 3 Hz. Applied to controller-facing hip/knee pitch qd only, gated on WALKING.
   private static final double NOTCH_CENTER_HZ = 11.8;
   private static final double NOTCH_Q = 2.0;
   private final YoBoolean yoControllerFacingNotchEnabled;
   private final Map<String, JointVelocityNotch> controllerFacingNotches = new HashMap<>();
   /** Supplies "high-level controller is in WALKING" (not RL_CONTROL); null = ungated. */
   private BooleanSupplier walkingGate = null;

   /** Wire to the high-level controller state: gate closes the notch outside WALKING (e.g. RL_CONTROL). */
   public void setControllerFacingNotchGate(BooleanSupplier walkingGate)
   {
      this.walkingGate = walkingGate;
   }

   /** RBJ biquad notch, direct form 1; fixed coefficients (f0, Q, dt known at construction). */
   private static final class JointVelocityNotch
   {
      private final double b0, b1, b2, a1, a2;
      private double x1, x2, y1, y2;

      JointVelocityNotch(double centerHz, double q, double dt)
      {
         double w0 = 2.0 * Math.PI * centerHz * dt;
         double alpha = Math.sin(w0) / (2.0 * q);
         double a0 = 1.0 + alpha;
         b0 = 1.0 / a0;
         b1 = -2.0 * Math.cos(w0) / a0;
         b2 = 1.0 / a0;
         a1 = -2.0 * Math.cos(w0) / a0;
         a2 = (1.0 - alpha) / a0;
      }

      double update(double x)
      {
         double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
         x2 = x1; x1 = x;
         y2 = y1; y1 = y;
         return y;
      }
   }

   private final HumanoidReferenceFrames referenceFrames;

   private final InvariantEKFStateEstimator invariantEstimator;
   private final FootReferencedYawCorrector yawCorrector; // null when yaw seeding is disabled
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
                                                          initialCovariance);
      invariantEstimator.setRunningAsMain(true); // disables the invariantMinusMain* self-comparisons
      registry.addChild(invariantEstimator.getYoRegistry());

      // 2026-07-16 anti-damping fix: notch the CONTROLLER-FACING qd of the pitch-plane leg joints
      // (hips + knees — the two measured energy injectors) around the measured 11.8 Hz mode. With
      // the measured ~19 ms loop delay, velocity feedback near f = 1/(4·τ_d) ≈ 13 Hz injects energy
      // (+2.7 W measured, L knee, log jointKF_Osc_0716). The old ALPHA_COMPLEMENTARY chain
      // attenuated this band implicitly; the JointKF passes it at unity gain. Estimator-internal
      // consumers (InEKF, anchors, biases) keep the full-bandwidth signal — only the hand-off to
      // the shared model the WBC reads is filtered, and only when the walking gate is active.
      yoControllerFacingNotchEnabled = new YoBoolean("controllerFacingQdNotchEnabled", registry);
      yoControllerFacingNotchEnabled.set(true);
      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         String name = joint.getName();
         if (name.contains("HIP_Y") || name.contains("KNEE_Y"))
            controllerFacingNotches.put(name, new JointVelocityNotch(NOTCH_CENTER_HZ, NOTCH_Q, dt));
      }

      // Defer the joint-level KF's initialization until both feet are firmly in contact. Seeding it while the
      // robot hangs leaves the exported base-IMU gyro bias unobservable (its only anchor is the phase-2 stance
      // update, which is off with no trusted foot), so the bias wanders and this estimator integrates it into a
      // rotating base. The gate reads the same contact probability the trust decision uses; other pre-filter
      // types ignore setInitializationGate (default no-op), so only the JOINT_KF path is affected.
      if (preFilter instanceof JointLevelKFPreFilter jointLevelKF)
      {
         jointLevelKF.setInitializationGate(() ->
                  invariantEstimator.getContactProbability(RobotSide.LEFT) >= ON_GROUND_INIT_CONTACT_PROBABILITY_THRESHOLD
               && invariantEstimator.getContactProbability(RobotSide.RIGHT) >= ON_GROUND_INIT_CONTACT_PROBABILITY_THRESHOLD);
      }

      if (enableYawSeeding)
      {
         SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
         for (RobotSide side : RobotSide.values)
            soleFrames.put(side, referenceFrames.getSoleFrame(side));

         yawCorrector = new FootReferencedYawCorrector(invariantEstimator.getInvariantEKF(),
                                                       referenceFrames.getPelvisFrame(),
                                                       soleFrames,
                                                       invariantEstimator::getContactProbability,
                                                       registry);
      }
      else
      {
         yawCorrector = null;
      }

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

      if (yawCorrector != null)
         yawCorrector.correct();

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

         // Anti-damping notch on the WBC-facing qd (see constructor comment). The filter always
         // runs so its state stays warm (bumpless engage); the output is only USED when the master
         // switch is on AND the walking gate reports the WALKING high-level state (not RL_CONTROL).
         JointVelocityNotch notch = controllerFacingNotches.get(joint.getName());
         if (notch != null)
         {
            double filtered = notch.update(velocity);
            if (yoControllerFacingNotchEnabled.getBooleanValue() && (walkingGate == null || walkingGate.getAsBoolean()))
               velocity = filtered;
         }

         joint.setQ(position);
         joint.setQd(velocity);
         joint.setTau(jointOutput.getEffort());
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

      if (yawCorrector != null)
         yawCorrector.reset();

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
