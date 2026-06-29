package us.ihmc.stateEstimation.invariant_estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Tests for {@link InvariantUpdater} using a synthetic single-contact measurement:
 * H = [+I at the base-position block, −I at the contact block], residual r = R̂·y − (d̂ − p̂),
 * with y the exact forward-kinematics measurement generated from a "truth" state.
 */
public class InvariantUpdaterTest
{
   private static final int CONTACTS = 1;
   private static final int GROUP_SIZE = 5 + CONTACTS;      // n = 6
   private static final int TANGENT_SIZE = 9 + 3 * CONTACTS; // m = 12
   private static final int POSITION_BLOCK = 6;             // base-position tangent index
   private static final int CONTACT_BLOCK = 9;              // contact 0 tangent index

   /** Small error, tiny measurement noise: one update drives the residual to ~0 (validates sign/H). */
   @Test
   public void testUpdateDrivesResidualToZero()
   {
      Random random = new Random(101L);

      InvariantState truth = randomState(random);
      Vector3D measurement = forwardKinematicsFromTruth(truth); // exact, noise-free

      // estimate = exp(ξ₀)·truth with a small error
      InvariantState estimate = perturb(truth, randomError(random, 1.0e-4));
      CommonOps_DDRM.setIdentity(estimate.getCovariance()); // confident-ish prior P = I

      DMatrixRMaj Jc = buildContactJacobian();
      DMatrixRMaj measurementCovariance = scaledIdentity(3, 1.0e-10);

      DMatrixRMaj residual = new DMatrixRMaj(3, 1);
      double residualBefore = computeResidual(estimate, measurement, residual);

      InvariantUpdater updater = new InvariantUpdater(TANGENT_SIZE);
      updater.update(estimate, Jc, residual, measurementCovariance);

      DMatrixRMaj residualAfterVector = new DMatrixRMaj(3, 1);
      double residualAfter = computeResidual(estimate, measurement, residualAfterVector);

      assertTrue(residualAfter < 1.0e-6, "residual not driven to zero: " + residualAfter);
      assertTrue(residualAfter < residualBefore);

   }

   /** Moderate error: one update reduces the residual by a large factor (decisively rules out a sign flip). */
   @Test
   public void testUpdateReducesResidualForLargerError()
   {
      Random random = new Random(202L);

      InvariantState truth = randomState(random);
      Vector3D measurement = forwardKinematicsFromTruth(truth);

      InvariantState estimate = perturb(truth, randomError(random, 0.05));
      CommonOps_DDRM.setIdentity(estimate.getCovariance());

      DMatrixRMaj Jc = buildContactJacobian();
      DMatrixRMaj measurementCovariance = scaledIdentity(3, 1.0e-8);

      DMatrixRMaj residual = new DMatrixRMaj(3,1);
      double residualBefore = computeResidual(estimate, measurement, residual);

      InvariantUpdater updater = new InvariantUpdater(TANGENT_SIZE);
      updater.update(estimate, Jc, residual, measurementCovariance);

      DMatrixRMaj residualAfterVector = new DMatrixRMaj(3,1);
      double residualAfter = computeResidual(estimate, measurement, residualAfterVector);

      assertTrue(residualAfter < 0.1 * residualBefore, "residual not reduced: " + residualBefore + " -> " + residualAfter);
   }

   /** After an update, the covariance stays symmetric and its trace decreases (information gained). */
   @Test
   public void testCovarianceShrinksAndStaysSymmetric()
   {
      Random random = new Random(303L);

      InvariantState truth = randomState(random);
      Vector3D measurement = forwardKinematicsFromTruth(truth);

      InvariantState estimate = perturb(truth, randomError(random, 0.02));
      CommonOps_DDRM.setIdentity(estimate.getCovariance());
      double traceBefore = CommonOps_DDRM.trace(estimate.getCovariance());

      DMatrixRMaj Jc = buildContactJacobian();
      DMatrixRMaj measurementCovariance = scaledIdentity(3, 1.0e-4);

      DMatrixRMaj residual = new DMatrixRMaj(3,1);
      computeResidual(estimate, measurement, residual);

      InvariantUpdater updater = new InvariantUpdater(TANGENT_SIZE);
      updater.update(estimate, Jc, residual, measurementCovariance);

      assertSymmetric(estimate.getCovariance(), 1.0e-9);
      assertTrue(CommonOps_DDRM.trace(estimate.getCovariance()) < traceBefore);
   }

   private static InvariantState randomState(Random random)
   {
      InvariantState state = new InvariantState(CONTACTS);
      state.setRotation(EuclidCoreRandomTools.nextRotationMatrix(random));
      state.setBaseVelocity(EuclidCoreRandomTools.nextVector3D(random));
      state.setBasePosition(EuclidCoreRandomTools.nextVector3D(random));
      state.setContactPosition(0,EuclidCoreRandomTools.nextVector3D(random));
      return state;
   }

   private static Vector3D forwardKinematicsFromTruth(InvariantState truth)
   {
      RotationMatrix rotation = new RotationMatrix();
      Vector3D position = new Vector3D();
      Vector3D contact = new Vector3D();
      truth.getRotation(rotation);
      truth.getBasePosition(position);
      truth.getContactPosition(0, contact);

      Vector3D measurement = new Vector3D();
      measurement.sub(contact, position); // d - p
      rotation.inverseTransform(measurement); // R^T (d - p)
      return measurement;

   }

   private static double[] randomError(Random random, double scale)
   {
      double[] xi = new double[TANGENT_SIZE];
      for (int i = 0; i < xi.length; i++)
         xi[i] = scale * (2.0 * random.nextDouble() - 1.0);
      return xi;
   }

   private static InvariantState perturb(InvariantState truth, double[] xi)
   {
      DMatrixRMaj expXi = new DMatrixRMaj(GROUP_SIZE, GROUP_SIZE);
      SEK3_Utils.exp(xi, expXi);

      InvariantState estimate = new InvariantState(CONTACTS);
      CommonOps_DDRM.mult(expXi, truth.getGroupElement(), estimate.getGroupElement());
      return estimate;
   }

   /** Builds the synthetic single-contact Jacobian H (3×m): +I at position, −I at contact. */
   private static DMatrixRMaj buildContactJacobian()
   {
      DMatrixRMaj Jc = new DMatrixRMaj(3, TANGENT_SIZE);
      for (int r = 0; r < 3; r++)
      {
         Jc.set(r, POSITION_BLOCK + r, 1.0); // +I at base position
         Jc.set(r, CONTACT_BLOCK + r, -1.0); // -I at contact 0
      }
      return Jc;
   }

   private static DMatrixRMaj scaledIdentity(int size, double scale)
   {
      DMatrixRMaj matrix = new DMatrixRMaj(size, size);
      CommonOps_DDRM.setIdentity(matrix);
      CommonOps_DDRM.scale(scale, matrix);
      return matrix;
   }

   /**
    * Computes r = R̂·y − (d̂ − p̂) for the estimate's current X, packs it, and returns its norm.
    *
    * @param estimate        the estimate state (read). Not modified.
    * @param measurement     the body-frame forward-kinematics measurement y. Not modified.
    * @param residualToPack  the 3×1 residual to fill. Modified.
    */
   private static double computeResidual(InvariantState estimate, Vector3DReadOnly measurement, DMatrixRMaj residualToPack)
   {
      RotationMatrix rotation = new RotationMatrix();
      Vector3D position = new Vector3D();
      Vector3D contact = new Vector3D();
      estimate.getRotation(rotation);
      estimate.getBasePosition(position);
      estimate.getContactPosition(0,contact);

      Vector3D rotatedMeasurement = new Vector3D();
      rotation.transform(measurement, rotatedMeasurement); // R * y

      Vector3D residual = new Vector3D();
      residual.sub(contact, position); // \hat{d} - \hat{p}
      residual.negate();
      residual.add(rotatedMeasurement); // R * y - (d - p)

      residualToPack.set(0, 0, residual.getX());
      residualToPack.set(1, 0, residual.getY());
      residualToPack.set(2, 0, residual.getZ());
      return residual.norm();
   }

   /** Asserts P_ij == P_ji within eps. */
   private static void assertSymmetric(DMatrixRMaj matrix, double epsilon)
   {
      for (int r = 0; r < matrix.getNumRows(); r++)
         for (int c = r + 1; c < matrix.getNumCols(); c++)
            assertEquals(matrix.get(r, c), matrix.get(c, r), epsilon);
   }
}
