package us.ihmc.stateEstimation.invariantEstimator;

import java.util.Objects;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.stateEstimation.jointLevel.OneDoFJointStateSource;

/**
 * {@link ContactMeasurementNoiseProvider} that routes the joint-level filter's own position covariance into the
 * contact FK measurement noise: {@code R_i = scale * J_i Sigma_q J_i^T}, the step-8 covariance-routing provider
 * that {@link InvariantEKFStateEstimator#setContactMeasurementNoiseProvider} exists for.
 *
 * <p>This is the honest measurement model for the contact update. The contact "measurement" is the sole position
 * computed by forward kinematics from estimated joint angles, so its uncertainty IS the joint uncertainty pushed
 * through the FK Jacobian — not a hand-picked isotropic constant. It is also anisotropic and configuration
 * dependent in exactly the way a leg is: near a knee singularity the along-leg direction is far better known than
 * the perpendicular ones, and {@code J Sigma_q J^T} says so on its own.</p>
 *
 * <h2>Frames and column ordering</h2>
 * <p>{@code J_i} is the 3xn linear block of the pelvis-to-sole geometric Jacobian, taken in the sole frame (so it
 * is the velocity of the sole ORIGIN, which is what the measurement is about) and then rotated into the pelvis
 * frame, since {@link ContactMeasurementNoiseProvider} is specified in the body/pelvis frame. The Jacobian is
 * built from the same {@code OneDoFJointBasics[]} path that {@code Sigma_q} is packed against, so column i of
 * {@code J} and row/column i of {@code Sigma_q} refer to the same joint by construction rather than by
 * convention.</p>
 *
 * <h2>The learned scale</h2>
 * <p>{@code scale} is the {@code contact_fk_r} channel of the offline distributed-IMU noise calibration: a global
 * multiplier on the propagated joint covariance, equivalent to scaling the correction-side copy of {@code Sigma_q}
 * (since {@code J (s Sigma_q) J^T == s J Sigma_q J^T}). It does NOT touch the joint filter's actual covariance or
 * its joint means — only this correction-side noise. Pass 1.0 for the unscaled baseline.</p>
 *
 * <h2>Fallback</h2>
 * <p>Whenever the joint source reports no covariance yet ({@code hasCovariance() == false}, e.g. before the joint
 * filter initializes, or when the selected joint-level source is the pass-through), this delegates to the supplied
 * fallback provider rather than fabricating a covariance. That keeps the boot transient on the previously-shipping
 * constant model instead of on a zero or garbage R.</p>
 */
public class JointCovarianceContactMeasurementNoiseProvider implements ContactMeasurementNoiseProvider
{
   private final OneDoFJointStateSource jointSource;
   private final ContactMeasurementNoiseProvider fallback;
   private final double encoderFallbackVariance;
   private final double scale;
   private final double conditioningFloor;

   private final SideDependentList<GeometricJacobian> jacobians = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics[]> legJoints = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final ReferenceFrame pelvisFrame;

   // Scratch, pre-sized per side at construction so packContactCovariance allocates nothing on the estimator tick.
   private final SideDependentList<DMatrixRMaj> jacobianInPelvis = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> jointCovariance = new SideDependentList<>();
   private final SideDependentList<DMatrixRMaj> jacobianTimesCovariance = new SideDependentList<>();
   private final DMatrixRMaj contactCovariance = new DMatrixRMaj(3, 3);
   private final RotationMatrix soleToPelvis = new RotationMatrix();

   /**
    * @param pelvis                  the base body of the FK chain; also supplies the frame the covariance is
    *                                expressed in, per {@link ContactMeasurementNoiseProvider}.
    * @param feet                    the end-effector body per side.
    * @param soleFrames              the sole frame per side — the point whose position is the contact measurement.
    * @param jointSource             the joint-level filter supplying {@code Sigma_q}; consulted through
    *                                {@link OneDoFJointStateSource#hasCovariance()} every tick.
    * @param encoderFallbackVariance per-joint variance substituted for any leg joint the filter does not carry in
    *                                state (rad^2); handed straight to
    *                                {@link OneDoFJointStateSource#packPositionCovariance}.
    * @param scale                   the learned {@code contact_fk_r} multiplier; 1.0 for the unscaled baseline.
    * @param conditioningFloor       additive {@code floor * I3} applied AFTER scaling (m^2), purely to keep R
    *                                invertible at a singular leg configuration where {@code J Sigma_q J^T} loses
    *                                rank. Keep at 0.0 for exact parity with the offline model, which has no such
    *                                term; a nonzero value is a deliberate Java-side conditioning deviation.
    * @param fallback                consulted whenever {@code jointSource.hasCovariance()} is false.
    */
   public JointCovarianceContactMeasurementNoiseProvider(RigidBodyBasics pelvis,
                                                         SideDependentList<? extends RigidBodyBasics> feet,
                                                         SideDependentList<? extends ReferenceFrame> soleFrames,
                                                         OneDoFJointStateSource jointSource,
                                                         double encoderFallbackVariance,
                                                         double scale,
                                                         double conditioningFloor,
                                                         ContactMeasurementNoiseProvider fallback)
   {
      this.jointSource = Objects.requireNonNull(jointSource);
      this.fallback = Objects.requireNonNull(fallback);
      this.pelvisFrame = Objects.requireNonNull(pelvis).getBodyFixedFrame();

      if (!(encoderFallbackVariance > 0.0) || !Double.isFinite(encoderFallbackVariance))
         throw new IllegalArgumentException("encoderFallbackVariance must be positive and finite, was " + encoderFallbackVariance);
      if (!(scale > 0.0) || !Double.isFinite(scale))
         throw new IllegalArgumentException("contact FK noise scale must be positive and finite, was " + scale);
      if (!(conditioningFloor >= 0.0) || !Double.isFinite(conditioningFloor))
         throw new IllegalArgumentException("conditioningFloor must be non-negative and finite, was " + conditioningFloor);
      this.encoderFallbackVariance = encoderFallbackVariance;
      this.scale = scale;
      this.conditioningFloor = conditioningFloor;

      for (RobotSide side : RobotSide.values)
      {
         ReferenceFrame soleFrame = Objects.requireNonNull(soleFrames.get(side), "no sole frame for " + side);
         // The Jacobian is built from the ONE-DOF path specifically, and Sigma_q is packed against this very
         // array below, so J's columns and Sigma_q's indices cannot drift apart.
         OneDoFJointBasics[] joints = MultiBodySystemTools.createOneDoFJointPath(pelvis, feet.get(side));
         if (joints.length == 0)
            throw new IllegalArgumentException("no one-DoF joint path from pelvis to the " + side + " foot");

         this.soleFrames.put(side, soleFrame);
         this.legJoints.put(side, joints);
         // jacobianFrame = soleFrame: the LINEAR block of a twist expressed in F is the velocity of F's ORIGIN,
         // and the sole origin is exactly the point being measured. Taking it in the pelvis frame instead would
         // give the velocity of the pelvis origin, which is not the measured quantity.
         this.jacobians.put(side, new GeometricJacobian(joints, soleFrame));
         this.jacobianInPelvis.put(side, new DMatrixRMaj(3, joints.length));
         this.jointCovariance.put(side, new DMatrixRMaj(joints.length, joints.length));
         this.jacobianTimesCovariance.put(side, new DMatrixRMaj(3, joints.length));
      }
   }

   @Override
   public void packContactCovariance(RobotSide side, Matrix3D covarianceToPack)
   {
      if (!jointSource.hasCovariance())
      {
         fallback.packContactCovariance(side, covarianceToPack);
         return;
      }

      GeometricJacobian jacobian = jacobians.get(side);
      jacobian.compute();

      // Linear rows (3..5) of the 6xn geometric Jacobian, rotated sole -> pelvis.
      soleToPelvis.set(soleFrames.get(side).getTransformToDesiredFrame(pelvisFrame).getRotation());
      DMatrixRMaj jacobianMatrix = jacobian.getJacobianMatrix();
      DMatrixRMaj J = jacobianInPelvis.get(side);
      for (int column = 0; column < J.getNumCols(); column++)
      {
         double x = jacobianMatrix.get(3, column);
         double y = jacobianMatrix.get(4, column);
         double z = jacobianMatrix.get(5, column);
         J.set(0, column, soleToPelvis.getM00() * x + soleToPelvis.getM01() * y + soleToPelvis.getM02() * z);
         J.set(1, column, soleToPelvis.getM10() * x + soleToPelvis.getM11() * y + soleToPelvis.getM12() * z);
         J.set(2, column, soleToPelvis.getM20() * x + soleToPelvis.getM21() * y + soleToPelvis.getM22() * z);
      }

      DMatrixRMaj sigmaQ = jointCovariance.get(side);
      jointSource.packPositionCovariance(legJoints.get(side), encoderFallbackVariance, sigmaQ);

      // R = scale * J Sigma_q J^T. The scale rides on Sigma_q's copy, which is identical to scaling the product.
      DMatrixRMaj JSigma = jacobianTimesCovariance.get(side);
      CommonOps_DDRM.mult(J, sigmaQ, JSigma);
      CommonOps_DDRM.multTransB(scale, JSigma, J, contactCovariance);

      covarianceToPack.set(contactCovariance.get(0, 0) + conditioningFloor, contactCovariance.get(0, 1), contactCovariance.get(0, 2),
                           contactCovariance.get(1, 0), contactCovariance.get(1, 1) + conditioningFloor, contactCovariance.get(1, 2),
                           contactCovariance.get(2, 0), contactCovariance.get(2, 1), contactCovariance.get(2, 2) + conditioningFloor);
   }
}
