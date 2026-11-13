package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;

import static org.junit.jupiter.api.Assertions.*;

public class PoseErrorEKFTest
{
   private static final long MONO = 10_000_000_000L, STEP = 5_000_000L;

   @Test
   public void statisticalUpdateIsNotTickClippedAndJosephCovarianceRemainsSymmetricAndNonnegative()
   {
      var filter = new PoseErrorEKF(1, 1, .001, .001, 1, 1);
      var result = new RigidBodyTransform();
      filter.update(MONO, pose(0), pose(0), result);
      filter.update(MONO + STEP, pose(0), pose(.1), result);
      assertTrue(result.getTranslationX() > .09, "The posterior must not be clipped to one estimator tick's output allowance");
      for (int tick = 2; tick < 200; tick++)
         filter.update(MONO + tick * STEP, pose(0), pose(.1), result);
      DMatrixRMaj covariance = filter.getCovariance();
      for (int i = 0; i < 6; i++)
      {
         assertTrue(Double.isFinite(covariance.get(i, i)) && covariance.get(i, i) >= 0);
         for (int j = 0; j < 6; j++)
            assertEquals(covariance.get(i, j), covariance.get(j, i), 1e-12);
      }
   }

   @Test
   public void retainsRobotPredictionDuringMeasurementDropoutAndRejectsRepeatedTimestamps()
   {
      var filter = new PoseErrorEKF(.1, .1, .1, .1, .2, .25);
      var pose = new RigidBodyTransform();
      var result = new RigidBodyTransform();
      filter.update(1_000_000_000L, pose, pose, result);
      pose.getTranslation().setX(.1);
      filter.update(1_020_000_000L, pose, null, result);
      assertEquals(.1, result.getTranslationX(), 1.e-9);
      assertThrows(IllegalArgumentException.class, () -> filter.update(1_020_000_000L, pose, null, result));
   }

   private static RigidBodyTransform pose(double x)
   {
      var pose = new RigidBodyTransform();
      pose.getTranslation().setX(x);
      return pose;
   }
}
