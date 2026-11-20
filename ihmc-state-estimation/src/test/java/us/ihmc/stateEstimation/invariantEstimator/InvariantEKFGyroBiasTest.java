package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * The gyro bias state of the "imperfect" invariant EKF: the propagation coupling has the right sign and size, and a
 * standing robot's roll/pitch gyro bias is recovered from gravity and leg kinematics alone.
 */
public class InvariantEKFGyroBiasTest
{
   private static final double G = 9.81;
   private static final int CONTACTS = 2;

   /**
    * The bias-to-group block of one covariance step must equal the finite-difference effect of a bias change on the
    * propagated mean: X̂(b̂+δ) = exp(Φ_ξb δ)·X̂(b̂) with δb = b̂ − b.
    */
   @Test
   public void testPropagationCouplingMatchesFiniteDifference()
   {
      double dt = 1.0e-3;
      Vector3D omegaMeasured = new Vector3D(0.3, -0.7, 0.4);
      Vector3D accel = new Vector3D(0.5, -0.2, 9.9);
      Vector3D biasEstimate = new Vector3D(0.01, -0.02, 0.005);

      InvariantState prior = randomState();
      int mg = prior.getGroupTangentSize();
      int b = prior.gyroBiasTangentIndex();

      // Covariance route: P = σ²·I on the bias block only; after one step P[group, bias] = Φ_ξb σ².
      InvariantPropagator propagator = new InvariantPropagator(CONTACTS, 0.0, 0.0, 0.0, G, true);
      propagator.setGyroBias(true, 0.0);
      InvariantState covarianceState = copy(prior);
      double sigma2 = 1.0e-4;
      for (int i = 0; i < 3; i++)
         covarianceState.getCovariance().set(b + i, b + i, sigma2);
      Vector3D omega = new Vector3D(omegaMeasured);
      omega.sub(biasEstimate);
      propagator.predict(covarianceState, omega, accel, dt);

      // Mean route: perturb b̂ by δ along each axis.
      for (int axis = 0; axis < 3; axis++)
      {
         double delta = 1.0e-6;
         InvariantState nominal = copy(prior);
         InvariantState perturbed = copy(prior);
         Vector3D omegaPerturbed = new Vector3D(omega);
         omegaPerturbed.setElement(axis, omegaPerturbed.getElement(axis) - delta); // b̂ + δ is subtracted
         propagator.predict(nominal, omega, accel, dt);
         propagator.predict(perturbed, omegaPerturbed, accel, dt);

         DMatrixRMaj inverse = nominal.getGroupElement().copy();
         CommonOps_DDRM.invert(inverse);
         DMatrixRMaj error = new DMatrixRMaj(inverse.numRows, inverse.numCols);
         CommonOps_DDRM.mult(perturbed.getGroupElement(), inverse, error);
         double[] xi = new double[mg];
         SEK3Utils.log(error, xi);

         for (int r = 0; r < mg; r++)
         {
            double expected = covarianceState.getCovariance().get(r, b + axis) / sigma2 * delta;
            assertEquals(expected, xi[r], 2.0e-3 * delta + 1.0e-3 * Math.abs(expected), "row " + r + " axis " + axis);
         }
      }
   }

   /** Standing still with a constant gyro bias: roll and pitch bias are observable, yaw is not (no rest measurement). */
   @Test
   public void testStandingRobotRecoversRollPitchBias()
   {
      Vector3D trueBias = new Vector3D(0.003, 0.008, 0.002);
      InvariantEKF ekf = standingFilter();

      runStanding(ekf, trueBias, 30.0, false);
      assertEquals(trueBias.getX(), ekf.getGyroBias().getX(), 5.0e-4, "roll bias");
      assertEquals(trueBias.getY(), ekf.getGyroBias().getY(), 5.0e-4, "pitch bias");
      RotationMatrix rotation = new RotationMatrix();
      ekf.getRotation(rotation);
      assertTrue(Math.abs(rotation.getPitch()) < 2.0e-3 && Math.abs(rotation.getRoll()) < 2.0e-3, "tilt held: " + rotation);
   }

   /** A rest measurement pins all three axes, yaw included. */
   @Test
   public void testRestMeasurementPinsYaw()
   {
      Vector3D trueBias = new Vector3D(0.003, 0.008, 0.002);
      InvariantEKF ekf = standingFilter();
      runStanding(ekf, trueBias, 5.0, true);
      assertEquals(trueBias.getZ(), ekf.getGyroBias().getZ(), 2.0e-4, "yaw bias");
   }

   /** Inactive: the bias block neither moves nor couples, so the filter behaves as before. */
   @Test
   public void testInactiveBiasStaysPut()
   {
      InvariantEKF ekf = standingFilter();
      ekf.setGyroBiasEstimation(false, 0.0);
      int b = ekf.getState().gyroBiasTangentIndex();
      for (int i = 0; i < 3; i++)
         ekf.getState().getCovariance().set(b + i, b + i, 0.0);
      runStanding(ekf, new Vector3D(0.003, 0.008, 0.002), 2.0, true);
      assertEquals(0.0, ekf.getGyroBias().norm(), 0.0);
   }

   private static InvariantEKF standingFilter()
   {
      InvariantEKF ekf = new InvariantEKF(CONTACTS, 1.0e-6, 1.0e-4, 1.0e-6, G, true);
      ekf.setGyroBiasEstimation(true, 1.0e-10);
      int m = ekf.getState().getTangentSize();
      DMatrixRMaj p0 = CommonOps_DDRM.identity(m);
      CommonOps_DDRM.scale(1.0e-4, p0);
      int b = ekf.getState().gyroBiasTangentIndex();
      for (int i = 0; i < 3; i++)
         p0.set(b + i, b + i, 1.0e-4); // (10 mrad/s)²
      ekf.initialize(new RotationMatrix(), new Vector3D(), new Point3D(0.0, 0.0, 1.0),
                     new Tuple3DReadOnly[] {new Point3D(0.0, 0.1, 0.0), new Point3D(0.0, -0.1, 0.0)}, p0);
      return ekf;
   }

   /** True pose fixed at identity, 1 m above two fixed feet; gyro reads the bias, the accelerometer reads −g. */
   private static void runStanding(InvariantEKF ekf, Vector3D trueBias, double seconds, boolean restMeasurement)
   {
      double dt = 1.0e-3;
      Vector3D accel = new Vector3D(0.0, 0.0, G);
      Point3D[] feetInBody = {new Point3D(0.0, 0.1, -1.0), new Point3D(0.0, -0.1, -1.0)};
      Matrix3D contactNoise = new Matrix3D();
      contactNoise.setIdentity();
      contactNoise.scale(1.0e-6);
      Matrix3D velocityNoise = new Matrix3D();
      velocityNoise.setIdentity();
      velocityNoise.scale(1.0e-4);
      Vector3D corrected = new Vector3D();
      Vector3D velocity = new Vector3D();
      int ticks = (int) Math.round(seconds / dt);
      for (int k = 0; k < ticks; k++)
      {
         corrected.sub(trueBias, ekf.getGyroBias()); // ω_m − b̂ with ω_m = b
         ekf.predict(corrected, accel, dt);
         for (int c = 0; c < CONTACTS; c++)
         {
            velocity.cross(corrected, feetInBody[c]); // y = −(ω̂ × r)
            velocity.negate();
            ekf.updateBodyVelocity(velocity, velocityNoise, feetInBody[c]);
            ekf.update(c, feetInBody[c], contactNoise);
         }
         if (restMeasurement && k % 2000 == 1999)
            ekf.updateGyroBias(trueBias, 1.0e-8);
      }
   }

   private static InvariantState randomState()
   {
      InvariantState state = new InvariantState(CONTACTS, true);
      state.setRotation(new RotationMatrix(0.4, -0.2, 0.3));
      state.setBaseVelocity(new Vector3D(0.3, -0.1, 0.05));
      state.setBasePosition(new Vector3D(1.0, 2.0, 0.9));
      state.setContactPosition(0, new Vector3D(1.1, 2.1, 0.0));
      state.setContactPosition(1, new Vector3D(0.9, 1.9, 0.0));
      return state;
   }

   private static InvariantState copy(InvariantState source)
   {
      InvariantState copy = new InvariantState(CONTACTS, true);
      copy.getGroupElement().set(source.getGroupElement());
      copy.getCovariance().set(source.getCovariance());
      copy.getGyroBias().set(source.getGyroBias());
      return copy;
   }
}
