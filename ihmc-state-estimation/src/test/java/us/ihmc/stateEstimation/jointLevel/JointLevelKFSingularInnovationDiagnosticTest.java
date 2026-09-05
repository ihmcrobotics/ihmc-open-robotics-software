package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.identity;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

/**
 * Locks in the near-singular innovation-covariance diagnostic
 * ({@link JointLevelKFPreFilter#describeSingularInnovation}): when the stacked-gyro (or encoder) innovation
 * covariance S = H P H^T + R goes near-singular and the update is skipped, the message must name the physical
 * measurement — which IMU pair / stance anchor / encoder joint — carrying the degenerate direction, so a
 * hardware run reveals which part of the robot is driving the ill-conditioning (the Alex002 symptom where a
 * capped-but-large Qa inflated P until S went near-singular).
 */
public class JointLevelKFSingularInnovationDiagnosticTest
{
   /** A rank-deficient stacked-gyro block concentrated on pair 0 must be attributed to pair 0's IMUs. */
   @Test
   public void testDiagnosticNamesTheDegenerateGyroPair()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(1234L, 10, 1, 9); // n=8, m=2, one pair
      int dim = f.dim;
      f.filter.setStateForTest(new DMatrixRMaj(dim, 1), identity(dim)); // sane SPD prior so S is well-defined

      // 3 rows, all in pair 0's block [0,3). Rows 0 and 1 are identical => S is near-singular and the near-null
      // eigenvector concentrates on rows 0-1, both of which map to pair 0.
      int k = 3;
      DMatrixRMaj H = new DMatrixRMaj(k, dim);
      for (int c = 0; c < dim; c++)
      {
         double v = Math.sin(0.31 * (c + 1));
         H.set(0, c, v);
         H.set(1, c, v);                    // duplicate of row 0 -> rank deficiency
         H.set(2, c, Math.cos(0.17 * (c + 2)));
      }
      DMatrixRMaj R = new DMatrixRMaj(k, k);
      for (int i = 0; i < k; i++)
         R.set(i, i, 1.0e-6);

      String msg = f.filter.describeSingularInnovation("stackedGyroUpdate", "test-induced rank deficiency", H, R);

      assertTrue(msg.contains("near-singular"), msg);
      assertTrue(msg.contains("cond(S)"), msg);
      assertTrue(msg.contains("gyro pair 0"), msg);
      // pair 0's parent IMU is the base IMU; its name must appear in the mapped element.
      assertTrue(msg.contains(f.filter.getBaseIMU().getSensorName()), msg);
   }

   /** A rank-deficient encoder block must be attributed to the specific joint it observes (mapped via H's column). */
   @Test
   public void testDiagnosticNamesTheDegenerateEncoderJoint()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePair(4321L, 6, 1, 5); // n=4
      int dim = f.dim;
      f.filter.setStateForTest(new DMatrixRMaj(dim, 1), identity(dim));

      // Two identical encoder rows both observing joint state index 0 -> near-singular; the mapping reads the
      // observed joint from H's unit column, so it stays correct even under a future per-joint encoder gate.
      int k = 2;
      DMatrixRMaj H = new DMatrixRMaj(k, dim);
      H.set(0, 0, 1.0);
      H.set(1, 0, 1.0);
      DMatrixRMaj R = new DMatrixRMaj(k, k);
      R.set(0, 0, 1.0e-6);
      R.set(1, 1, 1.0e-6);

      String msg = f.filter.describeSingularInnovation("encoder", "test-induced rank deficiency", H, R);

      assertTrue(msg.contains("near-singular"), msg);
      assertTrue(msg.contains("encoder q of joint " + f.filteredJoints.get(0).getName()), msg);
   }
}
