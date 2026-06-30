package us.ihmc.stateEstimation.invariant_estimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Tests for the {@link InvariantEKF} orchestrator. Since the filter is pure wiring, the delegation tests
 * check that {@link InvariantEKF#predict} and {@link InvariantEKF#update} reproduce, bit-for-bit, a
 * standalone {@link InvariantPropagator} / {@link InvariantUpdater} run on an identical state. A
 * predict-then-update sanity test confirms the assembled loop reduces estimation error.
 *
 * <p>Helper plumbing below is complete; fill in the {@code @Test} bodies.</p>
 */
public class InvariantEKFTest
{
   private static final int CONTACTS = 1;
   private static final int GROUP_SIZE = 5 + CONTACTS;       // n = 6
   private static final int TANGENT_SIZE = 9 + 3 * CONTACTS; // m = 12

   private static final double GYRO_VARIANCE = 1.0e-4;
   private static final double ACCEL_VARIANCE = 1.0e-3;
   private static final double CONTACT_VARIANCE = 1.0e-6;

   /** create(...) wires a consistent state/propagator/updater and an installed ContactUpdater. */
   @Test
   public void testCreateWiresConsistentSizes()
   {
      InvariantEKF ekf = InvariantEKF.create(2, GYRO_VARIANCE, ACCEL_VARIANCE, CONTACT_VARIANCE);

      assertEquals(2, ekf.getNumberOfContacts());
      assertEquals(5 + 2, ekf.getState().getGroupSize());
      assertEquals(9 + 3 * 2, ekf.getState().getTangentSize());

      // A contact update must not throw IllegalStateException, meaning that the updater is wired properly
      ekf.initialize(new RotationMatrix(), new Vector3D(), new Vector3D(), new Tuple3DReadOnly[] {new Vector3D(), new Vector3D()}, scaledIdentity(9 + 3 * 2, 1.0));
      ekf.update(0, new Vector3D(0.1, 0.0, -0.5), bodyScaledIdentity(1.0e-6));


   }

   /** initialize(...) sets X and P; the getters read them back. */
   @Test
   public void testInitializeSetsEstimate()
   {
      Random random = new Random(11L);
      InvariantEKF ekf = InvariantEKF.create(CONTACTS, GYRO_VARIANCE, ACCEL_VARIANCE, CONTACT_VARIANCE);

      RotationMatrix rotation = EuclidCoreRandomTools.nextRotationMatrix(random);
      Vector3D velocity = EuclidCoreRandomTools.nextVector3D(random);
      Vector3D position = EuclidCoreRandomTools.nextVector3D(random);
      Vector3D contact = EuclidCoreRandomTools.nextVector3D(random);
      DMatrixRMaj covariance = scaledIdentity(TANGENT_SIZE, 2.5);

      ekf.initialize(rotation, velocity, position, new Tuple3DReadOnly[] {contact}, covariance);

      RotationMatrix rotationOut = new RotationMatrix();
      Vector3D velocityOut = new Vector3D();
      Vector3D positionOut = new Vector3D();
      Vector3D contactOut = new Vector3D();
      ekf.getRotation(rotationOut);
      ekf.getBaseVelocity(velocityOut);
      ekf.getBasePosition(positionOut);
      ekf.getContactPosition(0,contactOut);

      assertTrue(rotation.epsilonEquals(rotationOut, 1.0e-12));
      assertTrue(velocity.epsilonEquals(velocityOut, 1.0e-12));
      assertTrue(position.epsilonEquals(positionOut, 1.0e-12));
      assertTrue(contact.epsilonEquals(contactOut, 1.0e-12));
      assertMatricesEqual(covariance, ekf.getState().getCovariance(), 1.0e-12);

   }

   /** initialize(...) rejects a wrong number of contact positions. */
   @Test
   public void testInitializeRejectsWrongContactCount()
   {
      InvariantEKF ekf = InvariantEKF.create(CONTACTS, GYRO_VARIANCE, ACCEL_VARIANCE, CONTACT_VARIANCE);
      assertThrows(IllegalArgumentException.class, () -> ekf.initialize(new RotationMatrix(), new Vector3D(), new Vector3D(), new Tuple3DReadOnly[0], scaledIdentity(TANGENT_SIZE,1.0)));
   }

   /** initialize(...) rejects a wrongly-sized covariance. */
   @Test
   public void testInitializeRejectsWrongCovarianceSize()
   {
      InvariantEKF ekf = InvariantEKF.create(CONTACTS, GYRO_VARIANCE, ACCEL_VARIANCE, CONTACT_VARIANCE);
      assertThrows(IllegalArgumentException.class, () -> ekf.initialize(new RotationMatrix(), new Vector3D(), new Vector3D(), new Tuple3DReadOnly[] {new Vector3D()}, scaledIdentity(3,1.0)));

   }

   /** predict(...) reproduces a standalone InvariantPropagator on an identical state. */
   @Test
   public void testPredictDelegatesToPropagator()
   {
      Random random = new Random(22L);
      InvariantEKF ekf = InvariantEKF.create(CONTACTS, GYRO_VARIANCE, ACCEL_VARIANCE, CONTACT_VARIANCE);
      initializeRandom(ekf, random);

      InvariantState reference = copyOf(ekf.getState());
      InvariantPropagator propagator = new InvariantPropagator(CONTACTS, GYRO_VARIANCE, ACCEL_VARIANCE, CONTACT_VARIANCE);

      Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random);
      Vector3D linearAcceleration = EuclidCoreRandomTools.nextVector3D(random);
      double dt = 1.0e-3;

      ekf.predict(angularVelocity, linearAcceleration, dt);
      propagator.predict(reference, angularVelocity, linearAcceleration, dt);

      assertStatesEqual(reference, ekf.getState(), 1.0e-12);
   }

   /** update(...) reproduces a standalone InvariantUpdater + ContactUpdater on an identical state. */
   @Test
   public void testUpdateDelegatesToUpdater()
   {
      Random random = new Random(33L);
      InvariantEKF ekf = InvariantEKF.create(CONTACTS, GYRO_VARIANCE, ACCEL_VARIANCE, CONTACT_VARIANCE);
      initializeRandom(ekf, random);

      InvariantState reference = copyOf(ekf.getState());
      InvariantUpdater updater = new InvariantUpdater(TANGENT_SIZE);
      updater.setContactUpdater(new ContactUpdater(CONTACTS));

      Vector3D measurement = EuclidCoreRandomTools.nextVector3D(random);
      Matrix3D bodyCovariance = bodyScaledIdentity(1.0e-6);

      ekf.update(0, measurement, bodyCovariance);
      updater.update(reference, 0, measurement, bodyCovariance, false);

      assertStatesEqual(reference, ekf.getState(), 1.0e-12);

   }

   /** End-to-end: a predict followed by a contact update reduces the contact residual. */
   @Test
   public void testPredictThenUpdateReducesError()
   {
      Random random = new Random(44L);

      InvariantState truth = randomState(random);
      InvariantState estimate = perturb(truth, randomError(random, 1.0e-3));

      InvariantEKF ekf = InvariantEKF.create(CONTACTS, GYRO_VARIANCE, ACCEL_VARIANCE, CONTACT_VARIANCE);
      initializeFromState(ekf, estimate, scaledIdentity(TANGENT_SIZE,1.0));

      Vector3D angularVelocity = EuclidCoreRandomTools.nextVector3D(random);
      Vector3D linearAcceleration = EuclidCoreRandomTools.nextVector3D(random);
      double dt = 1.0e-3;

      // Move truth and estimate along the same trajectory
      new InvariantPropagator(CONTACTS, GYRO_VARIANCE, ACCEL_VARIANCE, CONTACT_VARIANCE).predict(truth, angularVelocity, linearAcceleration, dt);
      ekf.predict(angularVelocity, linearAcceleration, dt);

      Vector3D measurement = forwardKinematicsFromTruth(truth);
      double residualBefore = worldResidualNorm(ekf.getState(), 0, measurement);

      ekf.update(0, measurement, bodyScaledIdentity(1.0e-10));
      double residualAfter = worldResidualNorm(ekf.getState(), 0, measurement);

      assertTrue(residualAfter < residualBefore, "residual not reduced: " + residualBefore + " -> " + residualAfter);


   }

   // ------
   // Helpers
   // ------

   private static void initializeRandom(InvariantEKF ekf, Random random)
   {
      Tuple3DReadOnly[] contacts = new Tuple3DReadOnly[CONTACTS];
      for (int i = 0; i < CONTACTS; i++)
         contacts[i] = EuclidCoreRandomTools.nextVector3D(random);

      ekf.initialize(EuclidCoreRandomTools.nextRotationMatrix(random),
                     EuclidCoreRandomTools.nextVector3D(random),
                     EuclidCoreRandomTools.nextVector3D(random),
                     contacts,
                     scaledIdentity(TANGENT_SIZE, 1.0));
   }

   /** Initializes the filter's X from an existing state's group element, with the given covariance. */
   private static void initializeFromState(InvariantEKF ekf, InvariantState source, DMatrixRMaj covariance)
   {
      RotationMatrix rotation = new RotationMatrix();
      Vector3D velocity = new Vector3D();
      Vector3D position = new Vector3D();
      source.getRotation(rotation);
      source.getBaseVelocity(velocity);
      source.getBasePosition(position);

      Tuple3DReadOnly[] contacts = new Tuple3DReadOnly[CONTACTS];
      for (int i = 0; i < CONTACTS; i++)
      {
         Vector3D contact = new Vector3D();
         source.getContactPosition(i, contact);
         contacts[i] = contact;
      }

      ekf.initialize(rotation, velocity, position, contacts, covariance);
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
      measurement.sub(contact, position);
      rotation.inverseTransform(measurement);
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

   /** Norm of the world-frame residual r = R̂·y − (d̂ − p̂). */
   private static double worldResidualNorm(InvariantState estimate, int contactIndex, Vector3DReadOnly measurement)
   {
      RotationMatrix rotation = new RotationMatrix();
      Vector3D position = new Vector3D();
      Vector3D contact = new Vector3D();
      estimate.getRotation(rotation);
      estimate.getBasePosition(position);
      estimate.getContactPosition(contactIndex, contact);

      Vector3D residual = new Vector3D();
      rotation.transform(measurement, residual);
      Vector3D difference = new Vector3D();
      difference.sub(contact, position);
      residual.sub(difference);
      return residual.norm();
   }

   private static InvariantState copyOf(InvariantState source)
   {
      InvariantState copy = new InvariantState(source.getNumberOfContacts());
      copy.getGroupElement().set(source.getGroupElement());
      copy.getCovariance().set(source.getCovariance());
      return copy;
   }

   private static void assertStatesEqual(InvariantState expected, InvariantState actual, double epsilon)
   {
      assertMatricesEqual(expected.getGroupElement(), actual.getGroupElement(), epsilon);
      assertMatricesEqual(expected.getCovariance(), actual.getCovariance(), epsilon);
   }

   private static DMatrixRMaj scaledIdentity(int size, double scale)
   {
      DMatrixRMaj matrix = new DMatrixRMaj(size, size);
      CommonOps_DDRM.setIdentity(matrix);
      CommonOps_DDRM.scale(scale, matrix);
      return matrix;
   }

   private static Matrix3D bodyScaledIdentity(double scale)
   {
      Matrix3D matrix = new Matrix3D();
      matrix.setIdentity();
      matrix.scale(scale);
      return matrix;
   }

   private static void assertMatricesEqual(DMatrixRMaj expected, DMatrixRMaj actual, double epsilon)
   {
      assertEquals(expected.getNumRows(), actual.getNumRows());
      assertEquals(expected.getNumCols(), actual.getNumCols());
      for (int r = 0; r < expected.getNumRows(); r++)
         for (int c = 0; c < expected.getNumCols(); c++)
            assertEquals(expected.get(r, c), actual.get(r, c), epsilon, "mismatch at (" + r + "," + c + ")");
   }
}
