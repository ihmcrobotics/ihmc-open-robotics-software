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

/**
 * Six-dimensional pose error-state EKF: robot relative motion predicts, aligned pose measurements correct.
 * Noise is explicitly configured; the inputs can be correlated and are not assumed ground truth.
 * The error state is world-frame position, then left-multiplicative orientation.
 */
public class PoseErrorEKF
{
   private final DMatrixRMaj processCovarianceRate, measurementCovariance;
   private final double translationGate, rotationGate;
   private final DMatrixRMaj covariance = new DMatrixRMaj(6, 6);
   private final RigidBodyTransform previousRobot = new RigidBodyTransform(), fused = new RigidBodyTransform();
   private final RigidBodyTransform measurementWorldToRobotWorld = new RigidBodyTransform();
   private final RigidBodyTransform delta = new RigidBodyTransform(), measurement = new RigidBodyTransform();
   private final Vector3D displacement = new Vector3D(), translation = new Vector3D(), angle = new Vector3D();
   private final Quaternion rotation = new Quaternion(), dq = new Quaternion();
   private final Matrix3D skew = new Matrix3D();
   private final DMatrixRMaj f = new DMatrixRMaj(6, 6), temp = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj innovation = new DMatrixRMaj(6, 6), inverse = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj gain = new DMatrixRMaj(6, 6), a = new DMatrixRMaj(6, 6), posterior = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj reset = new DMatrixRMaj(6, 6), resetWork = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj residual = new DMatrixRMaj(6, 1), correction = new DMatrixRMaj(6, 1);
   private final LinearSolverDense<DMatrixRMaj> innovationSolver = LinearSolverFactory_DDRM.symmPosDef(6);
   private boolean initialized, aligned;
   private long previousTime = Long.MIN_VALUE;
   private volatile String status = "initializing";
   private volatile double lastNis = Double.NaN;
   private volatile int accepted, rejected;

   /** Process standard deviations are per sqrt(second); measurement standard deviations are per pose. */
   public PoseErrorEKF(double positionProcessStd,
                           double angleProcessStd,
                           double positionMeasurementStd,
                           double angleMeasurementStd,
                           double translationGate,
                           double rotationGate)
   {
      for (double value : new double[] {positionProcessStd, angleProcessStd, positionMeasurementStd, angleMeasurementStd, translationGate, rotationGate})
         if (!Double.isFinite(value) || value <= 0)
            throw new IllegalArgumentException("EKF noise and gates must be finite and positive");
      processCovarianceRate = setupDiagonalPoseCovariance(positionProcessStd, angleProcessStd);
      measurementCovariance = setupDiagonalPoseCovariance(positionMeasurementStd, angleMeasurementStd);
      this.translationGate = translationGate;
      this.rotationGate = rotationGate;
   }

   private static DMatrixRMaj setupDiagonalPoseCovariance(double positionStd, double angleStd)
   {
      double positionVariance = positionStd * positionStd;
      double angleVariance = angleStd * angleStd;
      return CommonOps_DDRM.diag(positionVariance, positionVariance, positionVariance, angleVariance, angleVariance, angleVariance);
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

      if (!initialized)
      {
         fused.set(kinematicPoseEstimate);
         initialized = true;
         covariance.set(measurementCovariance);
         status = "EKF initialized in robot world";
      }
      else
      {
         delta.set(previousRobot);
         delta.invert();
         delta.multiply(kinematicPoseEstimate);
         double dt = (timestamp - previousTime) * 1.0e-9;
         predict(delta, dt);
         status = "EKF prediction only: no pose measurement";
      }
      if (poseMeasurement != null)
      {
         if (!aligned)
         {
            measurementWorldToRobotWorld.set(fused);
            measurementWorldToRobotWorld.multiplyInvertOther(poseMeasurement);
            aligned = true;
            status = "EKF measurement world aligned; awaiting next measurement";
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

      // Innovation covariance S = P + R, and the normalized innovation squared r^T S^-1 r.
      CommonOps_DDRM.add(covariance, measurementCovariance, innovation);
      if (!innovationSolver.setA(innovation))
         throw new IllegalStateException("EKF innovation covariance is not positive definite");
      innovationSolver.invert(inverse);
      lastNis = VectorVectorMult_DDRM.innerProdA(residual, inverse, residual);

      // Six-degree-of-freedom chi-square gate (99.9%), plus absolute jump limits.
      if (translation.norm() > translationGate || angle.norm() > rotationGate || !Double.isFinite(lastNis) || lastNis > 22.458)
      {
         rejected++;
         status = "EKF prediction only: pose measurement rejected";
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
      accepted++;
      status = "EKF robot prediction + pose measurement correction";
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
      initialized = aligned = false;
      previousTime = Long.MIN_VALUE;
      accepted = rejected = 0;
      lastNis = Double.NaN;
      covariance.zero();
      status = "initializing";
   }

   public String getStatus()
   {
      return status + "; accepted=" + accepted + ", rejected=" + rejected + ", NIS=" + lastNis;
   }
}
