package us.ihmc.stateEstimation.invariant_estimator;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.StateEstimatorControllerFactory;

/**
 * {@link StateEstimatorControllerFactory} that builds an {@link InvariantEKFStateEstimator}.
 *
 * <p>Register it as a secondary estimator on the simulation factory
 * ({@code avatarSimulationFactory.setSecondaryStateEstimatorFactory(...)}) to run the invariant EKF
 * alongside the main estimator for evaluation.</p>
 */
public class InvariantEKFStateEstimatorFactory implements StateEstimatorControllerFactory
{
   private final String primaryImuName;
   private final double estimatorDT;
   private final double gyroVariance;
   private final double accelVariance;
   private final double contactVariance;
   private final double contactMeasurementVariance;
   private final double initialCovariance;

   // These are the InvariantEKF's CONTINUOUS-time process-noise variances (rad^2/s on the orientation block,
   // m^2/s^3 on the velocity block, per InvariantPropagator), NOT sensor covariances — the InEKF never reads the
   // per-IMU getAngular/LinearAccelerationNoiseCovariance (those feed only the JointLevelKF). They are a TUNING
   // pair balancing how contact updates split innovation between orientation and velocity.
   //
   // 2026-07-10: REVERTED to the original 1e-7/1e-7. A hardware test (Alex_001/.../pitchDrift) with the
   // SDF-derived values below drove excessive pelvis PITCH drift after init: lowering gyro var (1e-7 -> 4e-8) and
   // raising accel var (1e-7 -> 2.89e-4) shifted the orientation:velocity process-noise ratio from 1:1 to ~1:7000,
   // so the contact update corrects velocity and leaves orientation "stiff" -> uncorrected gyro drift accumulates
   // in pitch. The 1e-7 pair was empirically tuned and works; do not retune it off a raw-sigma^2 units argument
   // (the SDF numbers are discrete 1 kHz stddevs, not continuous PSDs, and the measurement model / contact noise /
   // dt all factor into the "right" value). Retune only against NIS if ever needed.
   // SDF-density reference, if revisiting WITH a proper continuous-PSD conversion: gyro (2e-4 rad/s)^2 = 4e-8,
   // accel (1.7e-2 m/s^2)^2 = 2.89e-4.
   private static final double GYRO_VARIANCE = 1.0e-7;
   private static final double ACCEL_VARIANCE = 1.0e-7;
   private static final double CONTACT_VARIANCE = 1.0e-12;
   private static final double CONTACT_MEASUREMENT_VARIANCE = 1.0e-12;
   private static final double INITIAL_COVARIANCE = 1.0;


   /** The most recently created estimator, exposed so callers can add its YoGraphics to the SCS. */
   private InvariantEKFStateEstimator estimator;

   /**
    * Uses reasonable defaults for the noise terms; only the timestep and primary IMU name are required.
    *
    * @param estimatorDT    the estimator timestep Δt (s), e.g. {@code stateEstimatorParameters.getEstimatorDT()}.
    * @param primaryImuName sensor name of the pelvis (base) IMU to use, e.g.
    *                       {@code sensorInformation.getPrimaryBodyImu()}.
    */
   public InvariantEKFStateEstimatorFactory(double estimatorDT, String primaryImuName)
   {
      this(
            estimatorDT,
            primaryImuName,
            GYRO_VARIANCE,
            ACCEL_VARIANCE,
            CONTACT_VARIANCE,
            CONTACT_MEASUREMENT_VARIANCE,
            INITIAL_COVARIANCE
      );
   }

   /**
    * @param estimatorDT                the estimator timestep Δt (s).
    * @param primaryImuName             sensor name of the pelvis (base) IMU to use, e.g.
    *                                   {@code sensorInformation.getPrimaryBodyImu()}.
    * @param gyroVariance               continuous gyro noise variance σ_ω².
    * @param accelVariance              continuous accel noise variance σ_a².
    * @param contactVariance            continuous contact-slip noise variance σ_c².
    * @param contactMeasurementVariance per-axis body-frame contact measurement variance (m²).
    * @param initialCovariance          scalar for the initial P = initialCovariance · I.
    */
   public InvariantEKFStateEstimatorFactory(double estimatorDT,
                                            String primaryImuName,
                                            double gyroVariance,
                                            double accelVariance,
                                            double contactVariance,
                                            double contactMeasurementVariance,
                                            double initialCovariance)
   {
      this.estimatorDT = estimatorDT;
      this.primaryImuName = primaryImuName;
      this.gyroVariance = gyroVariance;
      this.accelVariance = accelVariance;
      this.contactVariance = contactVariance;
      this.contactMeasurementVariance = contactMeasurementVariance;
      this.initialCovariance = initialCovariance;
   }

   @Override
   public StateEstimatorController createStateEstimator(FullHumanoidRobotModel fullRobotModel, SensorReader sensorReader)
   {
     estimator = new InvariantEKFStateEstimator(fullRobotModel,
                                                sensorReader.getProcessedSensorOutputMap(),
                                                primaryImuName,
                                                estimatorDT,
                                                gyroVariance,
                                                accelVariance,
                                                contactVariance,
                                                contactMeasurementVariance,
                                                initialCovariance);
     return estimator;
   }

   /** @return the estimator built by the last {@link #createStateEstimator} call, or {@code null} if none yet. */
   public InvariantEKFStateEstimator getEstimator()
   {
      return estimator;
   }
}
