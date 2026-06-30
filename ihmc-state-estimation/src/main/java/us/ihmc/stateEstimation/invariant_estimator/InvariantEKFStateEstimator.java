package us.ihmc.stateEstimation.invariant_estimator;

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

   // Estimate outputs for logging/comparison.
   private final YoFramePoint3D yoBasePosition = new YoFramePoint3D("invariantFilterPelvisBasePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoQuaternion yoBaseOrientation = new YoQuaternion("invariantFilterPelvisBaseOrientation", registry);
   private final YoFrameVector3D yoBaseVelocity = new YoFrameVector3D("invariantFilterPelvisBaseVelocity", ReferenceFrame.getWorldFrame(), registry);

   // Main (DRC) estimator's pelvis pose, read from the shared model, and the invariant-vs-main difference.
   private final YoFramePoint3D yoMainBasePosition = new YoFramePoint3D("mainFilterPelvisBasePosition", ReferenceFrame.getWorldFrame(), registry);
   private final YoQuaternion yoMainBaseOrientation = new YoQuaternion("mainFilterPelvisBaseOrientation", registry);
   private final YoFrameVector3D yoBasePositionError = new YoFrameVector3D("invariantMinusMainPositionError", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble yoBasePositionErrorMagnitude = new YoDouble("invariantMinusMainPositionErrorMagnitude", registry);
   private final YoDouble yoBaseOrientationErrorAngle = new YoDouble("invariantMinusMainOrientationErrorAngle", registry);

   /**
    * @param fullRobotModel             the (shared) estimator robot model used for forward kinematics.
    * @param sensorOutputMap            processed sensor outputs (IMU + joints).
    * @param dt                         estimator timestep Δt (s).
    * @param gyroVariance               continuous gyro noise variance σ_ω² for the propagator.
    * @param accelVariance              continuous accel noise variance σ_a² for the propagator.
    * @param contactVariance            continuous contact-slip noise variance σ_c² for the propagator.
    * @param contactMeasurementVariance per-axis body-frame contact measurement variance (m²).
    * @param initialCovariance          scalar for the initial P = initialCovariance · I.
    */
   public InvariantEKFStateEstimator(FullHumanoidRobotModel fullRobotModel,
                                     SensorOutputMapReadOnly sensorOutputMap,
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

      imuSensor = sensorOutputMap.getIMUOutputs().get(0); // primary IMU (check this)

      contactBodyCovariance.setToZero();
      contactBodyCovariance.setM00(contactMeasurementVariance);
      contactBodyCovariance.setM11(contactMeasurementVariance);
      contactBodyCovariance.setM22(contactMeasurementVariance);
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

      // IMU omega and a expressed in pelvis frame
      angularVelocity.setIncludingFrame(imuSensor.getMeasurementFrame(), imuSensor.getAngularVelocityMeasurement());
      angularVelocity.changeFrame(pelvisFrame);
      linearAcceleration.setIncludingFrame(imuSensor.getMeasurementFrame(), imuSensor.getLinearAccelerationMeasurement());
      linearAcceleration.changeFrame(pelvisFrame);


      ekf.predict(angularVelocity, linearAcceleration, dt);

      for (RobotSide side : RobotSide.values)
      {
         contactInBody.setToZero(soleFrames.get(side));
         contactInBody.changeFrame(pelvisFrame);
         ekf.update(CONTACT_INDICES.get(side), contactInBody, contactBodyCovariance);
      }

      updateYoVariables();

   }

   private void updateYoVariables()
   {
      ekf.getBasePosition(tempVector);
      yoBasePosition.set(tempVector);

      ekf.getRotation(tempRotation);
      yoBaseOrientation.set(tempRotation);

      ekf.getBaseVelocity(tempVector);
      yoBaseVelocity.set(tempVector);

      // Main estimator's pelvis pose = the shared model's pelvis frame (the main estimator ticks first).
      mainEstimatePelvisPose.setToZero(pelvisFrame);
      mainEstimatePelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      yoMainBasePosition.set(mainEstimatePelvisPose.getPosition());
      yoMainBaseOrientation.set(mainEstimatePelvisPose.getOrientation());

      // Invariant − main differences (should hover near zero if the filters agree).
      yoBasePositionError.sub(yoBasePosition, yoMainBasePosition);
      yoBasePositionErrorMagnitude.set(yoBasePositionError.norm());
      yoBaseOrientationErrorAngle.set(yoBaseOrientation.distance(mainEstimatePelvisPose.getOrientation()));
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
}
