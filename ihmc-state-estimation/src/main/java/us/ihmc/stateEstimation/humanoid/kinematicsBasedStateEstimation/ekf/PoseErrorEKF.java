package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.dense.row.mult.VectorVectorMult_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoLong;

/**
 * Six-dimensional pose error-state EKF: robot relative motion predicts, aligned pose measurements correct.
 * Noise is explicitly configured; the inputs can be correlated and are not assumed ground truth.
 * The error state is world-frame position, then left-multiplicative orientation.
 * <p>
 * The noise standard deviations and the measurement rejection limits are yo-variables, read every update, so they can be tuned
 * at runtime. The filter's state, the last measurement's innovation, the measurement counts and the filter's own standard
 * deviations are logged as yo-variables.
 * </p>
 */
public class PoseErrorEKF
{
   /** What the filter did on its last update. */
   public enum State
   {
      /** Not updated since it was created or reset. */
      UNINITIALIZED,
      /** Predicted from the kinematic motion, with no pose measurement this update. */
      PREDICTING,
      /** The first measurement in a new measurement world set the alignment to that world; it doesn't correct the filter. */
      MEASUREMENT_WORLD_ALIGNED,
      /** Predicted, then corrected by a pose measurement. */
      MEASUREMENT_ACCEPTED,
      /** Predicted; the pose measurement was outside the innovation limits and was ignored. */
      MEASUREMENT_REJECTED
   }

   private final YoDouble positionProcessStandardDeviation;
   private final YoDouble orientationProcessStandardDeviation;
   private final YoDouble positionMeasurementStandardDeviation;
   private final YoDouble orientationMeasurementStandardDeviation;
   private final YoDouble maximumPositionInnovation;
   private final YoDouble maximumOrientationInnovation;
   private final YoDouble maximumNormalizedInnovationSquared;

   private final YoEnum<State> state;
   private final YoDouble positionInnovation;
   private final YoDouble orientationInnovation;
   private final YoDouble normalizedInnovationSquared;
   private final YoLong acceptedMeasurementCount;
   private final YoLong rejectedMeasurementCount;
   private final YoVector3D positionStandardDeviation;
   private final YoVector3D orientationStandardDeviation;

   private final DMatrixRMaj processCovarianceRate = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj measurementCovariance = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj covariance = new DMatrixRMaj(6, 6);
   private final RigidBodyTransform previousRobot = new RigidBodyTransform();
   private final RigidBodyTransform fused = new RigidBodyTransform();
   private final RigidBodyTransform measurementWorldToRobotWorld = new RigidBodyTransform();
   private final RigidBodyTransform delta = new RigidBodyTransform();
   private final RigidBodyTransform measurement = new RigidBodyTransform();
   private final Vector3D displacement = new Vector3D();
   private final Vector3D translation = new Vector3D();
   private final Vector3D angle = new Vector3D();
   private final Quaternion rotation = new Quaternion();
   private final Quaternion dq = new Quaternion();
   private final Matrix3D skew = new Matrix3D();
   private final DMatrixRMaj f = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj temp = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj innovation = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj inverse = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj gain = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj a = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj posterior = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj reset = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj resetWork = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj residual = new DMatrixRMaj(6, 1);
   private final DMatrixRMaj correction = new DMatrixRMaj(6, 1);
   private final LinearSolverDense<DMatrixRMaj> innovationSolver = LinearSolverFactory_DDRM.symmPosDef(6);
   private boolean initialized;
   private boolean aligned;
   private long previousTime = Long.MIN_VALUE;

   /**
    * @param parameters the initial noise and rejection limits. Process standard deviations are per sqrt(second); measurement
    *                   standard deviations are per pose.
    * @param registry   the registry the tuning and logged yo-variables are added to.
    */
   public PoseErrorEKF(EKFPoseFusionParameters parameters, YoRegistry registry)
   {
      for (double value : new double[] {parameters.getPositionProcessStandardDeviation(),
                                        parameters.getOrientationProcessStandardDeviation(),
                                        parameters.getPositionMeasurementStandardDeviation(),
                                        parameters.getOrientationMeasurementStandardDeviation(),
                                        parameters.getMaximumPositionInnovation(),
                                        parameters.getMaximumOrientationInnovation(),
                                        parameters.getMaximumNormalizedInnovationSquared()})
         if (!Double.isFinite(value) || value <= 0)
            throw new IllegalArgumentException("EKF noise and innovation limits must be finite and positive");

      positionProcessStandardDeviation = new YoDouble("ekfPositionProcessStandardDeviation", registry);
      orientationProcessStandardDeviation = new YoDouble("ekfOrientationProcessStandardDeviation", registry);
      positionMeasurementStandardDeviation = new YoDouble("ekfPositionMeasurementStandardDeviation", registry);
      orientationMeasurementStandardDeviation = new YoDouble("ekfOrientationMeasurementStandardDeviation", registry);
      maximumPositionInnovation = new YoDouble("ekfMaximumPositionInnovation", registry);
      maximumOrientationInnovation = new YoDouble("ekfMaximumOrientationInnovation", registry);
      maximumNormalizedInnovationSquared = new YoDouble("ekfMaximumNormalizedInnovationSquared", registry);
      positionProcessStandardDeviation.set(parameters.getPositionProcessStandardDeviation());
      orientationProcessStandardDeviation.set(parameters.getOrientationProcessStandardDeviation());
      positionMeasurementStandardDeviation.set(parameters.getPositionMeasurementStandardDeviation());
      orientationMeasurementStandardDeviation.set(parameters.getOrientationMeasurementStandardDeviation());
      maximumPositionInnovation.set(parameters.getMaximumPositionInnovation());
      maximumOrientationInnovation.set(parameters.getMaximumOrientationInnovation());
      maximumNormalizedInnovationSquared.set(parameters.getMaximumNormalizedInnovationSquared());

      state = new YoEnum<>("ekfState", registry, State.class);
      positionInnovation = new YoDouble("ekfPositionInnovation", registry);
      orientationInnovation = new YoDouble("ekfOrientationInnovation", registry);
      normalizedInnovationSquared = new YoDouble("ekfNormalizedInnovationSquared", registry);
      acceptedMeasurementCount = new YoLong("ekfAcceptedMeasurementCount", registry);
      rejectedMeasurementCount = new YoLong("ekfRejectedMeasurementCount", registry);
      positionStandardDeviation = new YoVector3D("ekfPositionStandardDeviation", registry);
      orientationStandardDeviation = new YoVector3D("ekfOrientationStandardDeviation", registry);
      reset();
   }

   /** Sets the diagonal pose covariance from a position and an orientation standard deviation, without allocating. */
   private static void setDiagonalPoseCovariance(double positionStandardDeviation, double orientationStandardDeviation, DMatrixRMaj covarianceToPack)
   {
      double positionVariance = positionStandardDeviation * positionStandardDeviation;
      double orientationVariance = orientationStandardDeviation * orientationStandardDeviation;
      covarianceToPack.zero();
      for (int index = 0; index < 3; index++)
      {
         covarianceToPack.set(index, index, positionVariance);
         covarianceToPack.set(index + 3, index + 3, orientationVariance);
      }
   }

   /** Allocation-free update for the estimator thread. A null or NaN pose measurement means there is none this tick. */
   public void update(long timestamp,
                      RigidBodyTransformReadOnly kinematicPoseEstimate,
                      RigidBodyTransformReadOnly poseMeasurement,
                      RigidBodyTransform resultToPack)
   {
      if (kinematicPoseEstimate.containsNaN())
         throw new IllegalArgumentException("EKF requires finite robot pose");
      if (previousTime != Long.MIN_VALUE && timestamp <= previousTime)
         throw new IllegalArgumentException("EKF requires increasing sensor timestamps");
      if (poseMeasurement != null && poseMeasurement.containsNaN())
         poseMeasurement = null;

      // The noise yo-variables can change at runtime.
      setDiagonalPoseCovariance(positionProcessStandardDeviation.getValue(), orientationProcessStandardDeviation.getValue(), processCovarianceRate);
      setDiagonalPoseCovariance(positionMeasurementStandardDeviation.getValue(), orientationMeasurementStandardDeviation.getValue(), measurementCovariance);

      if (!initialized)
      {
         fused.set(kinematicPoseEstimate);
         initialized = true;
         covariance.set(measurementCovariance);
      }
      else
      {
         delta.set(previousRobot);
         delta.invert();
         delta.multiply(kinematicPoseEstimate);
         double dt = (timestamp - previousTime) * 1.0e-9;
         predict(delta, dt);
      }
      state.set(State.PREDICTING);
      if (poseMeasurement != null)
      {
         if (!aligned)
         {
            measurementWorldToRobotWorld.set(fused);
            measurementWorldToRobotWorld.multiplyInvertOther(poseMeasurement);
            aligned = true;
            state.set(State.MEASUREMENT_WORLD_ALIGNED);
         }
         else
         {
            measurement.set(measurementWorldToRobotWorld);
            measurement.multiply(poseMeasurement);
            correct(measurement);
         }
      }
      previousRobot.set(kinematicPoseEstimate);
      previousTime = timestamp;
      updateStandardDeviations();
      resultToPack.set(fused);
   }

   private void predict(RigidBodyTransformReadOnly delta, double dt)
   {
      // F is the identity, except F_p,theta = -skew(R * delta_translation).
      fused.getRotation().transform(delta.getTranslation(), displacement);
      CommonOps_DDRM.setIdentity(f);
      skew.setToTildeForm(displacement);
      skew.negate();
      skew.get(0, 3, f);

      // P = F P F^T + Q dt
      CommonOps_DDRM.mult(f, covariance, temp);
      CommonOps_DDRM.multTransB(temp, f, covariance);
      CommonOps_DDRM.addEquals(covariance, dt, processCovarianceRate);
      fused.multiply(delta);
   }

   private void correct(RigidBodyTransformReadOnly measurement)
   {
      translation.sub(measurement.getTranslation(), fused.getTranslation());
      rotation.set(measurement.getRotation());
      rotation.appendInvertOther(fused.getRotation());
      rotation.getRotationVector(angle);
      translation.get(0, residual);
      angle.get(3, residual);
      positionInnovation.set(translation.norm());
      orientationInnovation.set(angle.norm());

      // Innovation covariance S = P + R, and the normalized innovation squared r^T S^-1 r.
      CommonOps_DDRM.add(covariance, measurementCovariance, innovation);
      if (!innovationSolver.setA(innovation))
         throw new IllegalStateException("EKF innovation covariance is not positive definite");
      innovationSolver.invert(inverse);
      normalizedInnovationSquared.set(VectorVectorMult_DDRM.innerProdA(residual, inverse, residual));

      // Chi-square test on the normalized innovation squared, plus absolute jump limits.
      if (positionInnovation.getValue() > maximumPositionInnovation.getValue()
          || orientationInnovation.getValue() > maximumOrientationInnovation.getValue()
          || !Double.isFinite(normalizedInnovationSquared.getValue())
          || normalizedInnovationSquared.getValue() > maximumNormalizedInnovationSquared.getValue())
      {
         rejectedMeasurementCount.increment();
         state.set(State.MEASUREMENT_REJECTED);
         return;
      }
      // K = P S^-1, and the correction is K r.
      CommonOps_DDRM.mult(covariance, inverse, gain);
      CommonOps_DDRM.mult(gain, residual, correction);

      // Apply the full accepted measurement and Joseph update exactly once. Controller-output
      // rate limiting belongs to EKFOutputSmoother, not to the statistical filter or its gain.
      translation.set(0, correction);
      fused.getTranslation().add(translation);
      angle.set(3, correction);
      dq.setRotationVector(angle);
      fused.getRotation().prepend(dq);
      fused.getRotation().normalize();

      // Joseph covariance update: P = (I - K) P (I - K)^T + K R K^T.
      CommonOps_DDRM.setIdentity(a);
      CommonOps_DDRM.subtractEquals(a, gain);
      CommonOps_DDRM.mult(a, covariance, temp);
      CommonOps_DDRM.multTransB(temp, a, posterior);
      CommonOps_DDRM.mult(gain, measurementCovariance, temp);
      CommonOps_DDRM.multAddTransB(temp, gain, posterior);

      // First-order error-state reset Jacobian for the left error convention: G = I + 1/2 skew(injected rotation) in the
      // orientation block. Then P = G P G^T, made exactly symmetric.
      reset.zero();
      skew.setToTildeForm(angle);
      skew.scale(0.5);
      skew.get(3, 3, reset);
      MatrixTools.addDiagonal(reset, 1.0);
      CommonOps_DDRM.mult(reset, posterior, temp);
      CommonOps_DDRM.multTransB(temp, reset, resetWork);
      CommonOps_DDRM.transpose(resetWork, temp);
      CommonOps_DDRM.add(0.5, resetWork, 0.5, temp, covariance);
      acceptedMeasurementCount.increment();
      state.set(State.MEASUREMENT_ACCEPTED);
   }

   /** The filter's own position and orientation standard deviations, from the covariance diagonal. */
   private void updateStandardDeviations()
   {
      positionStandardDeviation.set(Math.sqrt(covariance.get(0, 0)), Math.sqrt(covariance.get(1, 1)), Math.sqrt(covariance.get(2, 2)));
      orientationStandardDeviation.set(Math.sqrt(covariance.get(3, 3)), Math.sqrt(covariance.get(4, 4)), Math.sqrt(covariance.get(5, 5)));
   }

   /** A copy of the error-state covariance: position, then orientation. */
   public DMatrixRMaj getCovariance()
   {
      return new DMatrixRMaj(covariance);
   }

   /** A new measurement world anchors to the retained EKF prediction, not back to the kinematic pose estimate. */
   public void resetMeasurementAlignment()
   {
      aligned = false;
   }

   public void reset()
   {
      initialized = false;
      aligned = false;
      previousTime = Long.MIN_VALUE;
      acceptedMeasurementCount.set(0);
      rejectedMeasurementCount.set(0);
      normalizedInnovationSquared.set(Double.NaN);
      positionInnovation.set(Double.NaN);
      orientationInnovation.set(Double.NaN);
      covariance.zero();
      updateStandardDeviations();
      state.set(State.UNINITIALIZED);
   }

   public String getStatus()
   {
      return state.getEnumValue() + "; accepted=" + acceptedMeasurementCount.getValue() + ", rejected=" + rejectedMeasurementCount.getValue() + ", NIS="
             + normalizedInnovationSquared.getValue();
   }
}
