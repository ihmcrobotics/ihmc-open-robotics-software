package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

/**
 * Deployment hook for the redundancy-observed measurement noise: a per-tick, per-pair ADDITIVE
 * variance on each pair's own 3x3 diagonal block of the stacked noise {@code Rg}.
 *
 * <p>The offline pipeline drives this from the array's off-axis residual -- the component of an IMU
 * pair's differential rate that a rigid link forbids, and therefore a direct observation of the
 * violation that corrupts the informative component. What this class pins is the deployment
 * contract, not that physics: that the default path is untouched, that the term lands on exactly
 * the pair rows, and that it is added rather than multiplied.</p>
 *
 * <h2>Why additive and not a block multiplier</h2>
 * <p>{@code Rg = L Sigma L^T} is correlated across pairs that share an IMU. Scaling one diagonal
 * block of a correlated positive-definite matrix can destroy positive-definiteness; adding a
 * non-negative diagonal cannot. The Python pipeline made the same choice for the same reason, which
 * is what lets a value fitted offline transfer here unchanged.</p>
 */
public class JointLevelKFPairExtraVarianceTest
{
   private static final long SEED = 4242L;
   private static final int NUM_JOINTS = 4;
   private static final int PARENT_JOINT = 1;
   private static final int CHILD_JOINT = 2;

   private static JointLevelKFTestFixture fixture()
   {
      return JointLevelKFTestFixture.singlePairWithGyroSigmaScale(SEED, NUM_JOINTS, PARENT_JOINT, CHILD_JOINT, null);
   }

   private static DMatrixRMaj assembled(double[] pairExtra)
   {
      JointLevelKFTestFixture fixture = fixture();
      fixture.filter.setPairExtraVariance(pairExtra);
      fixture.filter.buildStackedMeasurementForTest();
      return fixture.filter.getStackedNoiseForTest();
   }

   /**
    * The one that matters most: this class touches the filter the robot flies, so the default path
    * must be bit-identical to the code before the hook existed. Not "close" -- identical.
    */
   @Test
   public void testTheDefaultPathIsBitIdenticalToNoHookAtAll()
   {
      DMatrixRMaj withoutHook = assembled(null);
      DMatrixRMaj withZeros = assembled(new double[] {0.0});

      assertEquals(withoutHook.getNumRows(), withZeros.getNumRows());
      for (int i = 0; i < withoutHook.getNumElements(); i++)
         assertEquals(withoutHook.get(i), withZeros.get(i), 0.0,
                      "element " + i + " differs; a zero extra must not perturb Rg at all");
   }

   @Test
   public void testTheExtraLandsOnThePairDiagonalAndIsAdded()
   {
      double extra = 3.5e-3;
      DMatrixRMaj before = assembled(null);
      DMatrixRMaj after = assembled(new double[] {extra});

      for (int d = 0; d < 3; d++)
         assertEquals(before.get(d, d) + extra, after.get(d, d), 1.0e-15,
                      "pair row " + d + " must gain exactly the supplied variance");
   }

   /**
    * Off-diagonal terms carry the correlation between pairs sharing an IMU. A multiplier would
    * rescale them; an additive diagonal must leave every one of them alone.
    */
   @Test
   public void testNothingOffThePairDiagonalMoves()
   {
      DMatrixRMaj before = assembled(null);
      DMatrixRMaj after = assembled(new double[] {7.5e-3});

      for (int r = 0; r < before.getNumRows(); r++)
         for (int c = 0; c < before.getNumCols(); c++)
            if (r != c)
               assertEquals(before.get(r, c), after.get(r, c), 0.0,
                            "off-diagonal (" + r + "," + c + ") must not move");
   }

   /** Inflating the noise must keep Rg usable by the Joseph update: symmetric and positive on the diagonal. */
   @Test
   public void testTheInflatedMatrixStaysSymmetricAndPositiveDiagonal()
   {
      DMatrixRMaj after = assembled(new double[] {1.2e-2});

      for (int r = 0; r < after.getNumRows(); r++)
      {
         assertTrue(after.get(r, r) > 0.0, "diagonal " + r + " must stay positive");
         for (int c = r + 1; c < after.getNumCols(); c++)
            assertEquals(after.get(r, c), after.get(c, r), 0.0, "Rg must remain exactly symmetric");
      }
   }

   /**
    * A wrong-length array is a mis-wired caller, and silently applying the entries that happen to
    * fit would inflate the wrong pairs -- which reads as a filter change, not as a bug.
    */
   @Test
   public void testAWrongLengthArrayIsRejected()
   {
      JointLevelKFTestFixture fixture = fixture();
      assertThrows(IllegalArgumentException.class, () -> fixture.filter.setPairExtraVariance(new double[] {1.0, 2.0}));
   }

   /** A NaN or negative variance is not a conservative filter; it is a broken one. */
   @Test
   public void testANonFiniteOrNegativeVarianceFailsLoudlyRatherThanDegrading()
   {
      for (double bad : new double[] {Double.NaN, Double.POSITIVE_INFINITY, -1.0e-6})
      {
         JointLevelKFTestFixture fixture = fixture();
         fixture.filter.setPairExtraVariance(new double[] {bad});
         assertThrows(IllegalStateException.class, fixture.filter::buildStackedMeasurementForTest,
                      "extra variance " + bad + " must be refused");
      }
   }

   /** Clearing restores the frozen-R path, so an arm can be run twice in one process without leakage. */
   @Test
   public void testClearingWithNullRestoresTheFrozenPath()
   {
      JointLevelKFTestFixture fixture = fixture();
      fixture.filter.setPairExtraVariance(new double[] {5.0e-3});
      fixture.filter.buildStackedMeasurementForTest();

      fixture.filter.setPairExtraVariance(null);
      fixture.filter.buildStackedMeasurementForTest();

      DMatrixRMaj cleared = fixture.filter.getStackedNoiseForTest();
      DMatrixRMaj never = assembled(null);
      for (int i = 0; i < never.getNumElements(); i++)
         assertEquals(never.get(i), cleared.get(i), 0.0, "element " + i + " leaked from the previous tick");
   }
}
