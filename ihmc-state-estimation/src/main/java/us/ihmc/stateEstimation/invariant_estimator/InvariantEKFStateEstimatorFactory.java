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

   private static final double GYRO_VARIANCE = 1.0e-4;
   private static final double ACCEL_VARIANCE = 1.0e-3;
   private static final double CONTACT_VARIANCE = 1.0e-6;
   private static final double CONTACT_MEASUREMENT_VARIANCE = 1.0e-4;
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
