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
}
