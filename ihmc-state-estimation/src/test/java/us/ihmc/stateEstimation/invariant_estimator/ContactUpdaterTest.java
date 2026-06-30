package us.ihmc.stateEstimation.invariant_estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.apache.commons.lang3.NotImplementedException;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Tests for the contact forward-kinematics measurement subpiece {@link ContactUpdater} and its
 * orchestration through {@link InvariantUpdater#update(InvariantState, int, Vector3DReadOnly, Matrix3DReadOnly, boolean)}.
 *
 * <p>The measurement is the body-frame contact position y = R̂ᵀ(d − p); the right-invariant residual is
 * r = R̂·y − (d̂ − p̂). Truth states generate exact, noise-free measurements; the estimate is a small
 * left-multiplicative perturbation exp(ξ)·truth.</p>
 *
 * <p>Helper plumbing below is complete; fill in the {@code @Test} bodies.</p>
 */
public class ContactUpdaterTest
{
   private static final int CONTACTS = 1;
   private static final int GROUP_SIZE = 5 + CONTACTS;       // n = 6
   private static final int TANGENT_SIZE = 9 + 3 * CONTACTS; // m = 12
   private static final int POSITION_BLOCK = 6;              // base-position tangent index
   private static final int CONTACT_BLOCK = 9;               // contact 0 tangent index

   /** Small error, tiny measurement noise: one contact update drives the residual to ~0. */
   @Test
   public void testContactUpdateDrivesResidualToZero()
   {
      Random random = new Random(101L);

      InvariantState truth = randomState(random);
      Vector3D measurement = forwardKinematicsFromTruth(truth); // exact, no noise

      InvariantState estimate = perturb(truth, randomError(random, 1.0e-4));
      CommonOps_DDRM.setIdentity(estimate.getCovariance()); // confident ish prior of P = I

      Matrix3D bodyCovariance = bodyScaledIdentity(1.0e-10);

      InvariantUpdater updater = new InvariantUpdater(TANGENT_SIZE);
      updater.setContactUpdater(new ContactUpdater(CONTACTS));

      double residualBefore = worldResidualNorm(estimate, 0, measurement);
      updater.update(estimate, 0, measurement, bodyCovariance, false);
      double residualAfter = worldResidualNorm(estimate, 0, measurement);

      assertTrue(residualAfter < 1.0e-6, "residual not driven to zero :" + residualAfter);
      assertTrue(residualAfter < residualBefore, "residual not decreasing. Went from " + residualBefore + " to " + residualAfter);


   }

   /** Moderate error: one update reduces the residual by a large factor (rules out a sign flip). */
   @Test
   public void testContactUpdateReducesResidualForLargerError()
   {
      Random random = new Random(202L);

      InvariantState truth = randomState(random);
      Vector3D measurement = forwardKinematicsFromTruth(truth);

      InvariantState estimate = perturb(truth, randomError(random, 0.05));
      CommonOps_DDRM.setIdentity(estimate.getCovariance());

      Matrix3D bodyCovariance = bodyScaledIdentity(1.0e-8);

      InvariantUpdater updater = new InvariantUpdater(TANGENT_SIZE);
      updater.setContactUpdater(new ContactUpdater(CONTACTS));

      double residualBefore = worldResidualNorm(estimate, 0, measurement);
      updater.update(estimate, 0, measurement, bodyCovariance, false);
      double residualAfter = worldResidualNorm(estimate, 0, measurement);

      assertTrue(residualAfter < 0.1 * residualBefore, "residual not reduced enough: " + residualBefore + " -> " + residualAfter);

   }

   /** After a contact update the covariance stays symmetric and its trace decreases (information gained). */
   @Test
   public void testCovarianceShrinksAndStaysSymmetric()
   {
      Random random = new Random(303L);

      InvariantState truth = randomState(random);
      Vector3D measurement = forwardKinematicsFromTruth(truth);

      InvariantState estimate = perturb(truth, randomError(random, 0.02));
      CommonOps_DDRM.setIdentity(estimate.getCovariance());
      double traceBefore = CommonOps_DDRM.trace(estimate.getCovariance());

      Matrix3D bodyCovariance = bodyScaledIdentity(1.0e-4);

      InvariantUpdater updater = new InvariantUpdater(TANGENT_SIZE);
      updater.setContactUpdater(new ContactUpdater(CONTACTS));
      updater.update(estimate, 0, measurement, bodyCovariance, false);

      assertSymmetric(estimate.getCovariance(), 1.0e-9);
      assertTrue(CommonOps_DDRM.trace(estimate.getCovariance()) < traceBefore);
   }

   /** H is constant: +I at the base-position block, −I at the contact block, zeros elsewhere, regardless of state. */
   @Test
   public void testJacobianStructureAndStateIndependence()
   {
      Random random = new Random(404L);
      ContactUpdater contactUpdater = new ContactUpdater(CONTACTS);

      DMatrixRMaj H1 = new DMatrixRMaj(3, TANGENT_SIZE);
      DMatrixRMaj H2 = new DMatrixRMaj(3, TANGENT_SIZE);
      contactUpdater.computeJacobian(randomState(random), 0, H1);
      contactUpdater.computeJacobian(randomState(random), 0, H2);

      // State independence check
      for (int r = 0; r < H1.getNumRows(); r++)
         for (int c = 0; c < H1.getNumCols(); c++)
            assertEquals(H1.get(r,c), H2.get(r,c), 0.0, "H differs at (" + r + "," + c + ")");

      // Structure check
      for (int r = 0; r < 3; r++)
         for (int c = 0; c < 3; c++)
         {
            double expected = 0.0;
            if (c == POSITION_BLOCK + r)
               expected = 1.0;
            else if (c == CONTACT_BLOCK + r)
               expected = -1.0;
            assertEquals(expected, H1.get(r,c), 0.0, "H wrong at (" + r + "," + c + ")");
         }

   }

   /** Residual matches r = R̂·y − (d̂ − p̂) exactly. */
   @Test
   public void testResidualFormula()
   {
      Random random = new Random(505L);
      InvariantState state = randomState(random);
      Vector3D measurement = EuclidCoreRandomTools.nextVector3D(random);

      DMatrixRMaj residual = new DMatrixRMaj(3,1);
      new ContactUpdater(CONTACTS).computeResidual(state, 0, measurement, residual);

      RotationMatrix rotation = new RotationMatrix();
      Vector3D basePosition = new Vector3D();
      Vector3D contactPosition = new Vector3D();
      state.getRotation(rotation);
      state.getBasePosition(basePosition);
      state.getContactPosition(0,contactPosition);

      Vector3D expected = new Vector3D();
      rotation.transform(measurement, expected);
      Vector3D difference = new Vector3D();
      difference.sub(contactPosition, basePosition);
      expected.sub(difference);

      assertEquals(expected.getX(), residual.get(0,0), 1.0e-12);
      assertEquals(expected.getY(), residual.get(1,0), 1.0e-12);
      assertEquals(expected.getZ(), residual.get(2,0), 1.0e-12);
   }

   /** Measurement covariance is rotated to the world frame: R_world = R̂·N·R̂ᵀ. */
   @Test
   public void testMeasurementCovarianceIsRotatedToWorld()
   {
      Random random = new Random(606L);
      InvariantState state = randomState(random);
      Matrix3D bodyCovariance = symmetricMatrix3D(random);

      DMatrixRMaj actual = new DMatrixRMaj(3,3);
      new ContactUpdater(CONTACTS).computeMeasurementCovariance(state, bodyCovariance, actual);

      RotationMatrix rotation = new RotationMatrix();
      state.getRotation(rotation);
      DMatrixRMaj R = toDMatrix(rotation);
      DMatrixRMaj N = toDMatrix(bodyCovariance);
      DMatrixRMaj RN = new DMatrixRMaj(3,3);
      DMatrixRMaj expected = new DMatrixRMaj(3,3);
      CommonOps_DDRM.mult(R, N, RN);
      CommonOps_DDRM.multTransB(RN,R,expected);

      assertMatricesEqual(expected, actual, 1.0e-12);

   }

   /** mapEncoderNoise builds N = J·Σ·Jᵀ. */
   @Test
   public void testMapEncoderNoise()
   {
      Random random = new Random(707L);
      int nJoints = 6;
      DMatrixRMaj contactJacobian = randomMatrix(3, nJoints, random);
      DMatrixRMaj jointCovariance = symmetricMatrix(nJoints, random);

      Matrix3D actual = new Matrix3D();
      ContactUpdater.mapEncoderNoise(contactJacobian, jointCovariance, actual);

      DMatrixRMaj jSigma = new DMatrixRMaj(3, nJoints);
      DMatrixRMaj expected = new DMatrixRMaj(3,3);
      CommonOps_DDRM.mult(contactJacobian, jointCovariance, jSigma);
      CommonOps_DDRM.multTransB(jSigma, contactJacobian, expected);

      assertMatricesEqual(expected, toDMatrix(actual), 1.0e-12);
   }

   /** Calling the contact update with no ContactUpdater installed throws. */
   @Test
   public void testUpdateWithoutContactUpdaterThrows()
   {
      Random random = new Random(808L);
      InvariantState state = randomState(random);
      Vector3D measurement = EuclidCoreRandomTools.nextVector3D(random);
      Matrix3D bodyCovariance = bodyScaledIdentity(1.0e-6);

      InvariantUpdater updater = new InvariantUpdater(TANGENT_SIZE);

      assertThrows(IllegalStateException.class, () -> updater.update(state, 0, measurement, bodyCovariance, false));
   }

   /** The learned module is not implemented yet: requesting it throws. */
   @Test
   public void testLearnedModuleNotImplementedThrows()
   {
      Random random = new Random(909L);
      InvariantState state = randomState(random);
      Vector3D measurement = EuclidCoreRandomTools.nextVector3D(random);
      Matrix3D bodyCovariance = bodyScaledIdentity(1.0e-6);

      InvariantUpdater updater = new InvariantUpdater(TANGENT_SIZE);
      updater.setContactUpdater(new ContactUpdater(CONTACTS));

      assertThrows(NotImplementedException.class, () -> updater.update(state, 0, measurement, bodyCovariance, true));
   }

   private static InvariantState randomState(Random random)
   {
      InvariantState state = new InvariantState(CONTACTS);
      state.setRotation(EuclidCoreRandomTools.nextRotationMatrix(random));
      state.setBaseVelocity(EuclidCoreRandomTools.nextVector3D(random));
      state.setBasePosition(EuclidCoreRandomTools.nextVector3D(random));
      state.setContactPosition(0, EuclidCoreRandomTools.nextVector3D(random));
      return state;
   }

   /** Exact body-frame forward-kinematics measurement y = Rᵀ(d − p) for the truth state. */
   private static Vector3D forwardKinematicsFromTruth(InvariantState truth)
   {
      RotationMatrix rotation = new RotationMatrix();
      Vector3D position = new Vector3D();
      Vector3D contact = new Vector3D();
      truth.getRotation(rotation);
      truth.getBasePosition(position);
      truth.getContactPosition(0, contact);

      Vector3D measurement = new Vector3D();
      measurement.sub(contact, position);     // d − p
      rotation.inverseTransform(measurement); // Rᵀ(d − p)
      return measurement;
   }

   private static double[] randomError(Random random, double scale)
   {
      double[] xi = new double[TANGENT_SIZE];
      for (int i = 0; i < xi.length; i++)
         xi[i] = scale * (2.0 * random.nextDouble() - 1.0);
      return xi;
   }

   /** estimate = exp(ξ)·truth (left-multiplicative perturbation). */
   private static InvariantState perturb(InvariantState truth, double[] xi)
   {
      DMatrixRMaj expXi = new DMatrixRMaj(GROUP_SIZE, GROUP_SIZE);
      SEK3_Utils.exp(xi, expXi);

      InvariantState estimate = new InvariantState(CONTACTS);
      CommonOps_DDRM.mult(expXi, truth.getGroupElement(), estimate.getGroupElement());
      return estimate;
   }

   /** Norm of the world-frame residual r = R̂·y − (d̂ − p̂) for the given estimate. */
   private static double worldResidualNorm(InvariantState estimate, int contactIndex, Vector3DReadOnly measurement)
   {
      RotationMatrix rotation = new RotationMatrix();
      Vector3D position = new Vector3D();
      Vector3D contact = new Vector3D();
      estimate.getRotation(rotation);
      estimate.getBasePosition(position);
      estimate.getContactPosition(contactIndex, contact);

      Vector3D residual = new Vector3D();
      rotation.transform(measurement, residual); // R̂·y
      Vector3D difference = new Vector3D();
      difference.sub(contact, position);          // d̂ − p̂
      residual.sub(difference);                   // R̂·y − (d̂ − p̂)
      return residual.norm();
   }

   private static Matrix3D bodyScaledIdentity(double scale)
   {
      Matrix3D matrix = new Matrix3D();
      matrix.setIdentity();
      matrix.scale(scale);
      return matrix;
   }

   private static Matrix3D symmetricMatrix3D(Random random)
   {
      DMatrixRMaj symmetric = symmetricMatrix(3, random);
      Matrix3D matrix = new Matrix3D();
      matrix.set(symmetric.get(0, 0), symmetric.get(0, 1), symmetric.get(0, 2),
                 symmetric.get(1, 0), symmetric.get(1, 1), symmetric.get(1, 2),
                 symmetric.get(2, 0), symmetric.get(2, 1), symmetric.get(2, 2));
      return matrix;
   }

   private static DMatrixRMaj randomMatrix(int rows, int cols, Random random)
   {
      DMatrixRMaj matrix = new DMatrixRMaj(rows, cols);
      for (int r = 0; r < rows; r++)
         for (int c = 0; c < cols; c++)
            matrix.set(r, c, 2.0 * random.nextDouble() - 1.0);
      return matrix;
   }

   /** A symmetric size×size matrix, ½(A + Aᵀ). */
   private static DMatrixRMaj symmetricMatrix(int size, Random random)
   {
      DMatrixRMaj a = randomMatrix(size, size, random);
      DMatrixRMaj symmetric = new DMatrixRMaj(size, size);
      CommonOps_DDRM.transpose(a, symmetric);
      CommonOps_DDRM.addEquals(symmetric, a);
      CommonOps_DDRM.scale(0.5, symmetric);
      return symmetric;
   }

   private static DMatrixRMaj toDMatrix(Matrix3DReadOnly matrix)
   {
      DMatrixRMaj result = new DMatrixRMaj(3, 3);
      for (int r = 0; r < 3; r++)
         for (int c = 0; c < 3; c++)
            result.set(r, c, matrix.getElement(r, c));
      return result;
   }

   private static void assertMatricesEqual(DMatrixRMaj expected, DMatrixRMaj actual, double epsilon)
   {
      assertEquals(expected.getNumRows(), actual.getNumRows());
      assertEquals(expected.getNumCols(), actual.getNumCols());
      for (int r = 0; r < expected.getNumRows(); r++)
         for (int c = 0; c < expected.getNumCols(); c++)
            assertEquals(expected.get(r, c), actual.get(r, c), epsilon, "mismatch at (" + r + "," + c + ")");
   }

   private static void assertSymmetric(DMatrixRMaj matrix, double epsilon)
   {
      for (int r = 0; r < matrix.getNumRows(); r++)
         for (int c = r + 1; c < matrix.getNumCols(); c++)
            assertEquals(matrix.get(r, c), matrix.get(c, r), epsilon);
   }
}
