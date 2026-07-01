package us.ihmc.stateEstimation.invariant_estimator;

import java.util.Objects;

import gnu.trove.map.TObjectDoubleMap;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.euclid.YoQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Adapter that runs the {@link InvariantEKF} as an IHMC {@link StateEstimatorController}.
 *
 * <p>Intended as a <em>secondary</em> (evaluation) estimator: it runs alongside the main estimator,
 * reads the same sensors, produces its own floating-base estimate, and exposes it as YoVariables for
 * comparison. It does not write to the shared robot model and does not drive the controller.</p>
 *
 * <p>Per tick it (1) reads the IMU ω and a and expresses them in the pelvis (base) frame, predicts;
 * then (2) for each foot it forms the body-frame contact measurement y = sole-origin-in-pelvis-frame
 * (= Rᵀ(p_Cᵢ − p_B)) and applies a contact update. For this first standing version both feet are
 * treated as always in contact — no touchdown/lift-off bookkeeping.</p>
 *
 * <p>Forward kinematics use the shared estimator robot model, whose joints are updated upstream each
 * tick (the main estimator runs first); base-relative quantities like the sole-in-pelvis position are
 * independent of the base pose, so they are valid regardless of which estimator owns the root.</p>
 */
public class InvariantEKFStateEstimator implements StateEstimatorController
{
   private static final int NUMBER_OF_CONTACTS = 2; // left, right feet
   private static final SideDependentList<Integer> CONTACT_INDICES = new SideDependentList<>(0, 1);

   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final InvariantEKF ekf;
   private final double dt;
   private final double initialCovariance;

   private final HumanoidReferenceFrames referenceFrames;
   private final MovingReferenceFrame pelvisFrame;
   private final SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();

   private final SensorOutputMapReadOnly sensorOutputMap;
   private final IMUSensorReadOnly imuSensor;

   private final Matrix3D contactBodyCovariance = new Matrix3D();

   // Per-tick temporaries.
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector3D linearAcceleration = new FrameVector3D();
   private final FramePoint3D contactInBody = new FramePoint3D();
   private final FramePoint3D contactInWorld = new FramePoint3D();
   private final RotationMatrix tempRotation = new RotationMatrix();
   private final Vector3D tempVector = new Vector3D();
   private final FramePose3D mainEstimatePelvisPose = new FramePose3D();
   private final FrameVector3D mainLinearVelocity = new FrameVector3D();           // DRC root linear velocity → world
   private final Vector3D invariantLinearVelocityWorld = new Vector3D();           // invariant base velocity (world)
   private final Vector3D mainAngularVelocityBody = new Vector3D();                // DRC root angular velocity (body)
   private final Vector3D invariantAngularVelocityBody = new Vector3D();           // gyro the filter integrates (body)
   private final Vector3D velocityErrorTemp = new Vector3D();

   // Soft contact handling: a per-foot contact probability p ∈ [0,1] drives two covariance knobs each tick.
   // The default provider is forward-kinematics only (runs on hardware); ContactNet replaces it later.
   private ContactProbabilityProvider contactProbabilityProvider;
   private final double baseContactVariance;                  // σ_c² at full contact (p = 1)
   private double swingMeasurementInflation = 1.0e6;          // R_i  ×= inflation^(1−p): muted swing foot at p → 0
   private double swingSlipInflation = 1.0e6;                 // σ_{c,i}² ×= inflation^(1−p): forgetful anchor at p → 0
   private final Matrix3D inflatedContactCovariance = new Matrix3D();
   private final SideDependentList<YoDouble> yoContactProbability;

   // Estimate outputs for logging/comparison.
   private final YoFramePoint3D yoBasePosition = new YoFramePoint3D("invariantFilterPelvisBasePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoQuaternion yoBaseOrientation = new YoQuaternion("invariantFilterPelvisBaseOrientation", registry);
   private final YoFrameVector3D yoBaseVelocity = new YoFrameVector3D("invariantFilterPelvisBaseVelocity", ReferenceFrame.getWorldFrame(), registry);

   // Main (DRC) estimator's pelvis pose, kept only for the 3D marker (position is unobservable so it drifts).
   private final YoFramePoint3D yoMainBasePosition = new YoFramePoint3D("mainFilterPelvisBasePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoQuaternion yoMainBaseOrientation = new YoQuaternion("mainFilterPelvisBaseOrientation", registry);

   // Observable comparisons against the DRC estimator: orientation, and root-joint linear/angular velocity.
   private final YoDouble yoBaseOrientationErrorAngle = new YoDouble("invariantMinusMainOrientationErrorAngle", registry);

   private final YoFrameVector3D yoMainRootLinearVelocity = new YoFrameVector3D("mainRootLinearVelocityWorld", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoInvariantRootLinearVelocity = new YoFrameVector3D("invariantRootLinearVelocityWorld",
                                                                                     ReferenceFrame.getWorldFrame(),
                                                                                     registry);
   private final YoDouble yoLinearVelocityErrorMagnitude = new YoDouble("invariantMinusMainLinearVelocityErrorMagnitude", registry);

   // Body-frame numbers stored in world-frame-labelled vars (set raw); names carry the frame.
   private final YoFrameVector3D yoMainRootAngularVelocity = new YoFrameVector3D("mainRootAngularVelocityBody", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D yoInvariantRootAngularVelocity = new YoFrameVector3D("invariantRootAngularVelocityBody",
                                                                                      ReferenceFrame.getWorldFrame(),
                                                                                      registry);
   private final YoDouble yoAngularVelocityErrorMagnitude = new YoDouble("invariantMinusMainAngularVelocityErrorMagnitude", registry);

   /**
    * @param fullRobotModel             the (shared) estimator robot model used for forward kinematics.
    * @param sensorOutputMap            processed sensor outputs (IMU + joints).
    * @param primaryImuName             sensor name of the pelvis (base) IMU to use, e.g.
    *                                   {@code sensorInformation.getPrimaryBodyImu()}. Matched against
    *                                   {@link IMUSensorReadOnly#getSensorName()} in {@code sensorOutputMap}.
    * @param dt                         estimator timestep Δt (s).
    * @param gyroVariance               continuous gyro noise variance σ_ω² for the propagator.
    * @param accelVariance              continuous accel noise variance σ_a² for the propagator.
    * @param contactVariance            continuous contact-slip noise variance σ_c² for the propagator.
    * @param contactMeasurementVariance per-axis body-frame contact measurement variance (m²).
    * @param initialCovariance          scalar for the initial P = initialCovariance · I.
    */
   public InvariantEKFStateEstimator(FullHumanoidRobotModel fullRobotModel,
                                     SensorOutputMapReadOnly sensorOutputMap,
                                     String primaryImuName,
                                     double dt,
                                     double gyroVariance,
                                     double accelVariance,
                                     double contactVariance,
                                     double contactMeasurementVariance,
                                     double initialCovariance)
   {
      this.dt = dt;
      this.initialCovariance = initialCovariance;
      this.sensorOutputMap = sensorOutputMap;

      ekf = new InvariantEKF(NUMBER_OF_CONTACTS, gyroVariance, accelVariance, contactVariance);

      referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      pelvisFrame = referenceFrames.getPelvisFrame();
      for (RobotSide side : RobotSide.values)
         soleFrames.put(side, referenceFrames.getSoleFrame(side));

      Objects.requireNonNull(primaryImuName, "primaryImuName must not be null");
      imuSensor = sensorOutputMap.getIMUOutputs()
                                 .stream()
                                 .filter(imu -> imu.getSensorName().equals(primaryImuName))
                                 .findFirst()
                                 .orElseThrow(() -> new IllegalArgumentException("No IMU named '" + primaryImuName
                                                                                 + "' found in sensor output map. Available: "
                                                                                 + sensorOutputMap.getIMUOutputs()
                                                                                                   .stream()
                                                                                                   .map(IMUSensorReadOnly::getSensorName)
                                                                                                   .toList()));

      contactBodyCovariance.setToZero();
      contactBodyCovariance.setM00(contactMeasurementVariance);
      contactBodyCovariance.setM11(contactMeasurementVariance);
      contactBodyCovariance.setM22(contactMeasurementVariance);

      this.baseContactVariance = contactVariance;
      yoContactProbability = new SideDependentList<>(new YoDouble("invariantContactProbabilityLeft", registry),
                                                     new YoDouble("invariantContactProbabilityRight", registry));
      // Default fallback: forward-kinematics-only contact detection on the estimator's own sole frames
      // (already refreshed each tick in doControl, so no frame-updater hook is needed here).
      contactProbabilityProvider = new KinematicContactDetector(soleFrames, null, dt);
   }

   @Override
   public void initializeEstimator(RigidBodyTransformReadOnly rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      referenceFrames.updateFrames();

      tempRotation.set(rootJointTransform.getRotation());
      Point3D basePosition = new Point3D(rootJointTransform.getTranslation());

      Tuple3DReadOnly[] contactPositions = new Tuple3DReadOnly[NUMBER_OF_CONTACTS];
      for (RobotSide side : RobotSide.values)
      {
         contactInWorld.setToZero(soleFrames.get(side));
         contactInWorld.changeFrame(ReferenceFrame.getWorldFrame());
         contactPositions[CONTACT_INDICES.get(side)] = new Point3D(contactInWorld);
      }

      ekf.initialize(tempRotation, new Vector3D(), basePosition, contactPositions, scaledIdentity(initialCovariance));
      updateYoVariables();
   }

   @Override
   public void doControl()
   {
      referenceFrames.updateFrames();

      // Refresh per-foot contact probabilities, then apply knob 2 (contact-position process noise) before
      // the prediction's covariance step consumes Q_c: a swing foot's anchor is given large slip noise so
      // it re-anchors softly on touchdown instead of kicking the base.
      contactProbabilityProvider.update();
      for (RobotSide side : RobotSide.values)
      {
         double contactProbability = clamp(contactProbabilityProvider.getContactProbability(side));
         yoContactProbability.get(side).set(contactProbability);
         ekf.setContactSlipVariance(CONTACT_INDICES.get(side), contactSlipVariance(contactProbability));
      }

      // IMU omega and a expressed in pelvis frame
      angularVelocity.setIncludingFrame(imuSensor.getMeasurementFrame(), imuSensor.getAngularVelocityMeasurement());
      angularVelocity.changeFrame(pelvisFrame);
      linearAcceleration.setIncludingFrame(imuSensor.getMeasurementFrame(), imuSensor.getLinearAccelerationMeasurement());
      linearAcceleration.changeFrame(pelvisFrame);

      ekf.predict(angularVelocity, linearAcceleration, dt);

      // Knob 1 (measurement covariance): inflate a swing foot's contact FK noise so it stops dragging the
      // base velocity, while a stance foot keeps constraining it.
      for (RobotSide side : RobotSide.values)
      {
         double contactProbability = yoContactProbability.get(side).getDoubleValue();
         contactInBody.setToZero(soleFrames.get(side));
         contactInBody.changeFrame(pelvisFrame);
         inflatedContactCovariance.set(contactBodyCovariance);
         inflatedContactCovariance.scale(measurementInflation(contactProbability));
         ekf.update(CONTACT_INDICES.get(side), contactInBody, inflatedContactCovariance);
      }

      updateYoVariables();
   }

   private void updateYoVariables()
   {
      ekf.getBasePosition(tempVector);
      yoBasePosition.set(tempVector);

      ekf.getRotation(tempRotation);
      yoBaseOrientation.set(tempRotation);

      ekf.getBaseVelocity(invariantLinearVelocityWorld);
      yoBaseVelocity.set(invariantLinearVelocityWorld);

      // Main estimator's pelvis pose = the shared model's pelvis frame (the main estimator ticks first).
      // Position is unobservable here, so it is kept only for the 3D marker; orientation is observable.
      mainEstimatePelvisPose.setToZero(pelvisFrame);
      mainEstimatePelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      yoMainBasePosition.set(mainEstimatePelvisPose.getPosition());
      yoMainBaseOrientation.set(mainEstimatePelvisPose.getOrientation());
      yoBaseOrientationErrorAngle.set(yoBaseOrientation.distance(mainEstimatePelvisPose.getOrientation()));

      // DRC estimator's estimated root-joint twist = the shared model's pelvis-frame twist.
      TwistReadOnly pelvisTwist = pelvisFrame.getTwistOfFrame();

      // Linear velocity (observable), compared in the world frame.
      mainLinearVelocity.setIncludingFrame(pelvisTwist.getLinearPart());
      mainLinearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
      yoMainRootLinearVelocity.set(mainLinearVelocity);
      yoInvariantRootLinearVelocity.set(invariantLinearVelocityWorld);
      velocityErrorTemp.sub(invariantLinearVelocityWorld, mainLinearVelocity);
      yoLinearVelocityErrorMagnitude.set(velocityErrorTemp.norm());

      // Angular velocity, compared in the pelvis (body) frame: the invariant filter's angular rate is the
      // gyro it integrates; the DRC value is the twist's angular part (both already in the pelvis frame).
      mainAngularVelocityBody.set(pelvisTwist.getAngularPart());
      invariantAngularVelocityBody.set(angularVelocity);
      yoMainRootAngularVelocity.set(mainAngularVelocityBody);
      yoInvariantRootAngularVelocity.set(invariantAngularVelocityBody);
      velocityErrorTemp.sub(invariantAngularVelocityBody, mainAngularVelocityBody);
      yoAngularVelocityErrorMagnitude.set(velocityErrorTemp.norm());
   }

   /**
    * Installs the contact-probability source (e.g. the controller-contact oracle, or a learned ContactNet
    * head). Replaces the default forward-kinematics detector.
    *
    * @param provider the new provider. Not null.
    */
   public void setContactProbabilityProvider(ContactProbabilityProvider provider)
   {
      this.contactProbabilityProvider = Objects.requireNonNull(provider, "contactProbabilityProvider");
   }

   /** @return the installed contact-probability source. */
   public ContactProbabilityProvider getContactProbabilityProvider()
   {
      return contactProbabilityProvider;
   }

   /** Sets the measurement-covariance inflation a fully-swing foot (p = 0) receives; 1 disables knob 1. */
   public void setSwingMeasurementInflation(double swingMeasurementInflation)
   {
      this.swingMeasurementInflation = swingMeasurementInflation;
   }

   /** Sets the contact-slip-variance inflation a fully-swing foot (p = 0) receives; 1 disables knob 2. */
   public void setSwingSlipInflation(double swingSlipInflation)
   {
      this.swingSlipInflation = swingSlipInflation;
   }

   /**
    * @param side the foot to query.
    * @return the most recent contact probability p ∈ [0,1] for {@code side} (set each {@code doControl}).
    *         Exposed for the yaw-seeding corrector when this filter drives the base as the main estimator.
    */
   public double getContactProbability(RobotSide side)
   {
      return yoContactProbability.get(side).getDoubleValue();
   }

   /** Knob 1: contact FK measurement covariance scale, inflation^(1−p) — 1 at p = 1, swing inflation at p = 0. */
   private double measurementInflation(double contactProbability)
   {
      return Math.pow(swingMeasurementInflation, 1.0 - contactProbability);
   }

   /** Knob 2: contact-position slip variance σ_{c,i}², base × inflation^(1−p). */
   private double contactSlipVariance(double contactProbability)
   {
      return baseContactVariance * Math.pow(swingSlipInflation, 1.0 - contactProbability);
   }

   private static double clamp(double value)
   {
      return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
   }

   private static org.ejml.data.DMatrixRMaj scaledIdentity(double scale)
   {
      int m = 9 + 3 * NUMBER_OF_CONTACTS;
      org.ejml.data.DMatrixRMaj matrix = new org.ejml.data.DMatrixRMaj(m, m);
      for (int i = 0; i < m; i++)
         matrix.set(i, i, scale);
      return matrix;
   }

   @Override
   public void requestStateEstimatorMode(StateEstimatorMode operatingMode)
   {
      // Evaluation estimator: nothing to switch.
   }

   @Override
   public void initialize()
   {
      referenceFrames.updateFrames();
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
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("invariantPelvisEstimate", yoBasePosition, 0.05, ColorDefinitions.Red()));
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("mainPelvisEstimate", yoMainBasePosition, 0.05, ColorDefinitions.Green()));
      return group;
   }

   /** @return the underlying filter, e.g. for tests or external inspection. */
   public InvariantEKF getInvariantEKF()
   {
      return ekf;
   }

   /**
    * @return the shared estimator robot model's pelvis frame, i.e. the <em>main</em> (DRC) estimator's
    *         pelvis pose/twist. This is the same frame used above to compute
    *         {@code invariantMinusMainOrientationErrorAngle} and the main-side velocity comparisons.
    *         Exposed so external comparators (e.g. against simulation ground truth) can read it directly
    *         instead of re-deriving it from the robot model.
    */
   public MovingReferenceFrame getMainPelvisFrame()
   {
      return pelvisFrame;
   }

   /**
    * Packs the body-frame angular velocity the filter integrates (the bias-free IMU gyro in the pelvis
    * frame). Exposed so external comparators can compare it against a reference angular velocity.
    *
    * @param angularVelocityToPack the body-frame angular velocity. Modified.
    */
   public void getMeasuredAngularVelocityInBody(us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics angularVelocityToPack)
   {
      angularVelocityToPack.set(angularVelocity);
   }
}
