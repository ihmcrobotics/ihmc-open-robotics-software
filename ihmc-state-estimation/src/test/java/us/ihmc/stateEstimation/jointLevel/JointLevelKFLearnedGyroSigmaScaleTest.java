package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;
import java.util.function.ToDoubleFunction;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

/**
 * Deployment hook for the offline-learned {@code imu_gyro:<name>} distributed-IMU noise channel: a per-IMU
 * VARIANCE multiplier on the raw gyro noise covariance, applied once when Sigma is built and cached.
 *
 * <p>The multiplier is trained offline against the POST-floor covariance, so the tests below pin not just
 * "the number changed" but that it lands after flooring and touches only the named IMU's 3x3 block.</p>
 */
public class JointLevelKFLearnedGyroSigmaScaleTest
{
   private static final long SEED = 4242L;
   private static final int NUM_JOINTS = 4;
   private static final int PARENT_JOINT = 1;
   private static final int CHILD_JOINT = 2;

   private static ToDoubleFunction<String> scales(Map<String, Double> byName)
   {
      return name -> byName.getOrDefault(name, 1.0);
   }

   private static DMatrixRMaj builtSigma(ToDoubleFunction<String> gyroSigmaScaleByImuName)
   {
      JointLevelKFTestFixture fixture = JointLevelKFTestFixture.singlePairWithGyroSigmaScale(SEED,
                                                                                              NUM_JOINTS,
                                                                                              PARENT_JOINT,
                                                                                              CHILD_JOINT,
                                                                                              gyroSigmaScaleByImuName);
      fixture.filter.buildStackedMeasurementForTest(); // Sigma is built lazily here, not at construction
      return fixture.filter.getGyroNoiseSigmaForTest();
   }

   /** No override supplied ⇒ the filter must be bit-identical to the pre-hook baseline. */
   @Test
   public void testANullOverrideLeavesSigmaExactlyAtTheUnscaledBaseline()
   {
      DMatrixRMaj withoutHook = builtSigma(null);
      DMatrixRMaj withNeutralScales = builtSigma(scales(Map.of()));

      assertEquals(withoutHook.getNumRows(), withNeutralScales.getNumRows());
      for (int i = 0; i < withoutHook.getNumElements(); i++)
         assertEquals(withoutHook.get(i), withNeutralScales.get(i), 0.0, "element " + i + " must be bit-identical");
   }

   /**
    * The real contract: a learned multiplier on ONE IMU scales exactly that IMU's 3x3 diagonal block by the
    * requested factor, and leaves every other block untouched. Both together are what "per-IMU" means -- a
    * version that scaled the whole Sigma, or scaled the wrong ordinal, would pass only one of them.
    */
   @Test
   public void testAPerImuScaleMultipliesOnlyThatImusBlockAndByExactlyTheRequestedFactor()
   {
      double factor = 7.0;
      DMatrixRMaj baseline = builtSigma(null);
      DMatrixRMaj scaled = builtSigma(scales(Map.of("imuB", factor)));

      // Fixture builds imuA as ordinal 0 and imuB as ordinal 1, so imuB owns rows/cols [3, 6).
      for (int row = 0; row < baseline.getNumRows(); row++)
      {
         for (int col = 0; col < baseline.getNumCols(); col++)
         {
            boolean inImuBBlock = row >= 3 && row < 6 && col >= 3 && col < 6;
            double expected = inImuBBlock ? baseline.get(row, col) * factor : baseline.get(row, col);
            assertEquals(expected, scaled.get(row, col), 1.0e-12, "Sigma(" + row + "," + col + ")");
         }
      }

      // Guard against a vacuous pass: imuB's block must actually be nonzero, or "scaled by 7" is 0 == 0.
      double imuBTrace = baseline.get(3, 3) + baseline.get(4, 4) + baseline.get(5, 5);
      assertTrue(imuBTrace > 0.0, "fixture must give imuB a nonzero baseline gyro variance");
   }

   /**
    * The design contract is "scale the baseline AFTER its acquisition and flooring." The fixture's IMUs have no
    * SensorNoiseParameters assigned, so every block is floored -- and the learned factor must still multiply the
    * FLOORED value rather than being swallowed by a re-floor.
    */
   @Test
   public void testTheScaleIsAppliedAfterFlooringRatherThanBeingReflooredAway()
   {
      double shrink = 0.25;
      DMatrixRMaj baseline = builtSigma(null);
      DMatrixRMaj shrunk = builtSigma(scales(Map.of("imuA", shrink)));

      // A scale BELOW one is the case a re-floor would silently undo: if the code floored after scaling, the
      // value would snap back up to the floor and this would read equal to the baseline.
      assertEquals(baseline.get(0, 0) * shrink, shrunk.get(0, 0), 1.0e-12, "floored baseline must still be scalable downward");
      assertTrue(shrunk.get(0, 0) < baseline.get(0, 0), "a sub-unity learned scale must actually lower the variance");
   }

   /** A mis-keyed or corrupt artifact must fail loudly, not silently deploy an unscaled (or NaN) filter. */
   @Test
   public void testANonPositiveOrNonFiniteScaleIsRejectedRatherThanSilentlyIgnored()
   {
      assertThrows(IllegalArgumentException.class, () -> builtSigma(scales(Map.of("imuA", 0.0))));
      assertThrows(IllegalArgumentException.class, () -> builtSigma(scales(Map.of("imuA", -1.0))));
      assertThrows(IllegalArgumentException.class, () -> builtSigma(scales(Map.of("imuA", Double.NaN))));
      assertThrows(IllegalArgumentException.class, () -> builtSigma(scales(Map.of("imuA", Double.POSITIVE_INFINITY))));
   }
}
