package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * The kinematic base-velocity update {@link InvariantEKF#updateBodyVelocity}: a body-frame measurement
 * {@code y = Rᵀv} pulls the velocity estimate to the truth whatever the orientation, touches nothing
 * uncorrelated with velocity, and reports a small NIS when the estimate already agrees.
 */
public class InvariantEKFBodyVelocityTest
{
   private static InvariantEKF filter(RotationMatrix rotation, Vector3D velocity, double priorVariance)
   {
      InvariantEKF ekf = new InvariantEKF(2, 1.0e-4, 1.0e-3, 1.0e-4, -9.81);
      ekf.getState().setRotation(rotation);
      ekf.getState().setBaseVelocity(velocity);
      CommonOps_DDRM.setIdentity(ekf.getState().getCovariance());
      CommonOps_DDRM.scale(priorVariance, ekf.getState().getCovariance());
      return ekf;
   }

   @Test
   public void testAWrongVelocityIsPulledToTheMeasuredOneAtAnyOrientation()
   {
      Random random = new Random(3L);
      for (int trial = 0; trial < 20; trial++)
      {
         RotationMatrix rotation = new RotationMatrix(EuclidCoreRandomTools.nextQuaternion(random));
         Vector3D truth = EuclidCoreRandomTools.nextVector3D(random, 1.0);
         Vector3D wrong = new Vector3D(truth);
         wrong.add(0.5, -0.3, 0.2);
         InvariantEKF ekf = filter(rotation, wrong, 1.0);

         Vector3D body = new Vector3D();
         rotation.inverseTransform(truth, body); // y = Rᵀ v
         Matrix3D noise = new Matrix3D();
         noise.setIdentity();
         noise.scale(1.0e-6);
         ekf.updateBodyVelocity(body, noise);

         Vector3D estimate = new Vector3D();
         ekf.getState().getBaseVelocity(estimate);
         estimate.sub(truth);
         assertTrue(estimate.norm() < 1.0e-5, "trial " + trial + ": velocity error after the update " + estimate.norm());
         assertTrue(ekf.wasLastUpdateApplied());

         // With a diagonal prior the rotation is uncorrelated with velocity and must not move.
         RotationMatrix after = new RotationMatrix();
         ekf.getState().getRotation(after);
         assertTrue(after.epsilonEquals(rotation, 1.0e-12));
      }
   }

   @Test
   public void testAnAgreeingMeasurementHasASmallNISAndBarelyMovesTheEstimate()
   {
      RotationMatrix rotation = new RotationMatrix(0.3, -0.2, 1.1);
      Vector3D truth = new Vector3D(0.4, -0.1, 0.05);
      InvariantEKF ekf = filter(rotation, truth, 1.0e-4);
      Vector3D body = new Vector3D();
      rotation.inverseTransform(truth, body);
      Matrix3D noise = new Matrix3D();
      noise.setIdentity();
      noise.scale(0.01);
      ekf.updateBodyVelocity(body, noise);
      assertEquals(0.0, ekf.getLastNormalizedInnovationSquared(), 1.0e-12);
      Vector3D estimate = new Vector3D();
      ekf.getState().getBaseVelocity(estimate);
      assertTrue(estimate.epsilonEquals(truth, 1.0e-12));
   }

   /**
    * With the heading correlated with velocity in P (as propagation builds up through the gyro bias), a horizontal
    * velocity innovation turns the heading; with the heading considered it must not, while velocity is still corrected.
    */
   @Test
   public void testHeadingConsiderKeepsTheHeadingWhenItIsCorrelatedWithVelocity()
   {
      for (boolean consider : new boolean[] {false, true})
      {
         InvariantEKF ekf = filter(new RotationMatrix(), new Vector3D(0.3, 0.0, 0.0), 1.0);
         // Correlate the heading (rotation row 2) with the x velocity
         int r = ekf.getState().rotationTangentIndex();
         int v = ekf.getState().baseVelocityTangentIndex();
         ekf.getState().getCovariance().set(r + 2, v, 0.8);
         ekf.getState().getCovariance().set(v, r + 2, 0.8);
         ekf.setKinematicVelocityHeadingConsider(consider);

         Matrix3D noise = new Matrix3D();
         noise.setIdentity();
         noise.scale(1.0e-4);
         ekf.updateBodyVelocity(new Vector3D(0.0, 0.0, 0.0), noise);

         RotationMatrix after = new RotationMatrix();
         ekf.getState().getRotation(after);
         Vector3D velocity = new Vector3D();
         ekf.getState().getBaseVelocity(velocity);
         assertTrue(Math.abs(velocity.getX()) < 0.01, "velocity must still be corrected, got " + velocity.getX());
         if (consider)
            assertEquals(0.0, after.getYaw(), 1.0e-12, "the heading must not move");
         else
            assertTrue(Math.abs(after.getYaw()) > 0.05, "without consider the correlated heading moves, got " + after.getYaw());
      }
   }

   /** A wildly wrong measurement moves the velocity much less with the NIS gate than without it. */
   @Test
   public void testTheNISGateLimitsAWildMeasurement()
   {
      double[] moved = new double[2];
      for (int gated = 0; gated < 2; gated++)
      {
         InvariantEKF ekf = filter(new RotationMatrix(), new Vector3D(), 1.0e-3);
         if (gated == 1)
            ekf.setKinematicVelocityNISGate(16.0);
         Matrix3D noise = new Matrix3D();
         noise.setIdentity();
         noise.scale(1.0e-3);
         ekf.updateBodyVelocity(new Vector3D(1.0, 0.0, 0.0), noise);
         Vector3D velocity = new Vector3D();
         ekf.getState().getBaseVelocity(velocity);
         moved[gated] = velocity.getX();
      }
      // Prior and noise equal: ungated gain 0.5. NIS = 1 / 0.002 = 500 > 16, so R is inflated about 31 times.
      assertTrue(moved[0] > 0.4, "ungated: the update moves halfway, moved " + moved[0]);
      assertTrue(moved[1] < 0.1, "gated: the correction is cut, moved " + moved[1] + " against " + moved[0]);
   }
}
