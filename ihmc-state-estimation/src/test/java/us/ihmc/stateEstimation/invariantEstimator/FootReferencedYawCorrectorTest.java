package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.geometry.AngleTools;

/**
 * Property tests for {@link FootReferencedYawCorrector#signedAngleFromTo}, the wrapped signed-angle
 * helper the yaw corrector uses in place of its former {@code atan2}-based {@code wrapToPi}.
 *
 * <p>The helper delegates to {@link AngleTools#angleMinusPiToPi}, which builds its magnitude from an
 * <em>unclamped</em> {@code Math.acos(dot / normStart / normEnd)}. For (anti)parallel inputs round-off
 * pushes that quotient outside [-1, 1] and {@code acos} returns NaN. Because {@code AngleTools} is
 * shared toolkit code that is not modified here, the clamp is applied at this call site instead, by
 * substituting the exact limiting value.</p>
 *
 * <p>This is not a hypothetical: on the tick a foot is anchored the corrector assigns
 * {@code anchorRelativeFootYaw = relativeFootYaw}, so the two direction vectors are bit-identical and
 * the NaN would flow through {@code prependYawRotation} into the filter mean. The properties checked:
 * <ul>
 *   <li><b>Totality:</b> the result is finite for every input, including the degenerate ones.</li>
 *   <li><b>Correctness:</b> it equals the wrapped difference {@code endYaw - startYaw}, matching the
 *       {@code atan2} formulation it replaced.</li>
 *   <li><b>Coverage:</b> the clamp actually fires on the inputs that make the raw call return NaN, and
 *       returns the right value there.</li>
 * </ul>
 * </p>
 */
public class FootReferencedYawCorrectorTest
{
   /** Loose enough to absorb the acos conditioning loss near 0 and pi, tight enough to catch a sign flip. */
   private static final double EPSILON = 1.0e-8;
   private static final int ITERATIONS = 20000;

   private final Vector2D start = new Vector2D();
   private final Vector2D end = new Vector2D();

   private static void setToUnitVector(Vector2D vectorToPack, double angle)
   {
      vectorToPack.set(Math.cos(angle), Math.sin(angle));
   }

   /** The formulation this helper replaced; the reference the new one must agree with. */
   private static double wrapToPi(double angle)
   {
      return Math.atan2(Math.sin(angle), Math.cos(angle));
   }

   /** Signed distance between two angles on the circle, so the +pi/-pi seam is not a false failure. */
   private static double circularDistance(double angleA, double angleB)
   {
      return Math.abs(wrapToPi(angleA - angleB));
   }

   /**
    * Totality over random pairs: never NaN, never infinite, always inside [-pi, pi].
    */
   @Test
   public void testNeverReturnsNaNForRandomDirections()
   {
      Random random = new Random(4321L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double startYaw = EuclidRandomAngle(random);
         double endYaw = EuclidRandomAngle(random);
         setToUnitVector(start, startYaw);
         setToUnitVector(end, endYaw);

         double angle = FootReferencedYawCorrector.signedAngleFromTo(start, end);

         assertFalse(Double.isNaN(angle), "NaN for startYaw = " + startYaw + ", endYaw = " + endYaw);
         assertTrue(Math.abs(angle) <= Math.PI + EPSILON, "Outside [-pi, pi]: " + angle);
      }
   }

   /**
    * The degenerate case the corrector hits on every touchdown: the anchor direction and the current
    * direction are the same object value, so the vectors are bit-identical. The raw
    * {@link AngleTools#angleMinusPiToPi} returns NaN for a large fraction of these; the guarded helper
    * must return exactly zero for all of them.
    */
   @Test
   public void testIdenticalDirectionsGiveExactlyZero()
   {
      Random random = new Random(9876L);
      int rawNaNCount = 0;

      for (int i = 0; i < ITERATIONS; i++)
      {
         double yaw = EuclidRandomAngle(random);
         setToUnitVector(start, yaw);
         setToUnitVector(end, yaw);

         if (Double.isNaN(AngleTools.angleMinusPiToPi(start, end)))
            rawNaNCount++;

         double angle = FootReferencedYawCorrector.signedAngleFromTo(start, end);

         assertFalse(Double.isNaN(angle), "NaN for identical directions at yaw = " + yaw);
         assertEquals(0.0, angle, 0.0, "Identical directions must give exactly zero, got " + angle);
      }

      // Guards against this test silently going vacuous: if AngleTools is ever fixed upstream the
      // count drops to zero and the clamp in the helper becomes redundant rather than load-bearing.
      assertTrue(rawNaNCount > 0,
                 "Expected the unclamped AngleTools.angleMinusPiToPi to produce NaN for identical "
                 + "directions; it did not, so the clamp may no longer be needed.");
   }

   /**
    * The other end of the acos domain: antiparallel inputs, where round-off can push the cosine below
    * -1. The wrapped convention is (-pi, pi], so the magnitude must come back as exactly pi.
    */
   @Test
   public void testAntipodalDirectionsGiveExactlyPi()
   {
      Random random = new Random(2468L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double yaw = EuclidRandomAngle(random);
         setToUnitVector(start, yaw);
         end.set(-start.getX(), -start.getY());

         double angle = FootReferencedYawCorrector.signedAngleFromTo(start, end);

         assertFalse(Double.isNaN(angle), "NaN for antipodal directions at yaw = " + yaw);
         assertEquals(Math.PI, Math.abs(angle), 0.0, "Antipodal directions must give exactly pi");
      }
   }

   /**
    * Near-degenerate sweep: separations spanning many decades down to the point where the two unit
    * vectors collapse onto each other. This is the band where the unclamped acos misbehaves.
    */
   @Test
   public void testNeverReturnsNaNNearDegenerateSeparations()
   {
      Random random = new Random(1357L);
      double[] separations = {1.0e-2, 1.0e-4, 1.0e-6, 1.0e-8, 1.0e-10, 1.0e-14, 0.0};

      for (double separation : separations)
      {
         for (int i = 0; i < 200; i++)
         {
            double startYaw = EuclidRandomAngle(random);

            // Both near-parallel and near-antiparallel.
            for (double base : new double[] {0.0, Math.PI})
            {
               setToUnitVector(start, startYaw);
               setToUnitVector(end, startYaw + base + separation);

               double angle = FootReferencedYawCorrector.signedAngleFromTo(start, end);

               assertFalse(Double.isNaN(angle),
                           "NaN at separation " + separation + " about base " + base + ", startYaw = " + startYaw);
               assertTrue(Math.abs(angle) <= Math.PI + EPSILON, "Outside [-pi, pi]: " + angle);
            }
         }
      }
   }

   /**
    * Correctness and sign convention: the helper must reproduce the wrapped difference
    * {@code endYaw - startYaw} that the replaced {@code wrapToPi(a - b)} produced. A swapped argument
    * order would flip the sign of the yaw correction and drive the filter the wrong way, so this is
    * the property that actually protects the estimator.
    */
   @Test
   public void testMatchesWrappedDifferenceOfAngles()
   {
      Random random = new Random(1122L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double startYaw = EuclidRandomAngle(random);
         double endYaw = EuclidRandomAngle(random);
         setToUnitVector(start, startYaw);
         setToUnitVector(end, endYaw);

         double angle = FootReferencedYawCorrector.signedAngleFromTo(start, end);
         double expected = wrapToPi(endYaw - startYaw);

         assertTrue(circularDistance(angle, expected) < EPSILON,
                    "Expected " + expected + " but got " + angle + " for startYaw = " + startYaw + ", endYaw = " + endYaw);
      }
   }

   /**
    * Antisymmetry, which pins the sign convention independently of the reference implementation:
    * swapping the arguments must negate the result, except at the pi seam where both directions
    * legitimately map to +pi.
    */
   @Test
   public void testIsAntisymmetricUnderArgumentSwap()
   {
      Random random = new Random(3344L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double startYaw = EuclidRandomAngle(random);
         double endYaw = EuclidRandomAngle(random);
         setToUnitVector(start, startYaw);
         setToUnitVector(end, endYaw);

         double forward = FootReferencedYawCorrector.signedAngleFromTo(start, end);
         double backward = FootReferencedYawCorrector.signedAngleFromTo(end, start);

         assertFalse(Double.isNaN(forward) || Double.isNaN(backward), "NaN under argument swap");

         if (Math.abs(Math.abs(forward) - Math.PI) < 1.0e-6)
            continue; // the (-pi, pi] seam maps both orders to +pi by construction

         assertTrue(Math.abs(forward + backward) < EPSILON,
                    "Not antisymmetric: forward = " + forward + ", backward = " + backward);
      }
   }

   private static double EuclidRandomAngle(Random random)
   {
      return -Math.PI + 2.0 * Math.PI * random.nextDouble();
   }
}
