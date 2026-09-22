package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

/**
 * The off-axis noise law, exercised on hand-built residuals so each of its three claims can be
 * checked in isolation: that only the off-axis part drives it, that only the FAST off-axis part
 * does, and that what it emits at tick t was built without ever reading tick t.
 *
 * <p>The third is the one the method's credibility rests on. A measurement-driven R invites the
 * objection that it down-weights exactly the ticks whose innovations are large, and no amount of
 * held-out evaluation answers that -- only the structure does. So it is tested the only way that
 * settles it: replay two histories that differ ONLY in the current tick and assert the current
 * output is identical.</p>
 */
public class PairOffAxisNoiseProviderTest
{
   private static final String[] ONE_PAIR = {"imuA->imuB"};
   private static final int MEDIAN_WINDOW = 5;
   private static final int CAUSAL_WINDOW = 3;
   private static final double SIGMA_ZERO = 2.0;

   /** A 1-DoF pair whose whole observable span is the x axis: anything in y or z is off-axis. */
   private static DMatrixRMaj[] xAxisJacobian()
   {
      DMatrixRMaj j = new DMatrixRMaj(3, 1);
      j.set(0, 0, 1.0);
      return new DMatrixRMaj[] {j};
   }

   private static DMatrixRMaj z(double x, double y, double zz)
   {
      DMatrixRMaj m = new DMatrixRMaj(3, 1);
      m.set(0, 0, x);
      m.set(1, 0, y);
      m.set(2, 0, zz);
      return m;
   }

   private static PairOffAxisNoiseProvider provider()
   {
      return new PairOffAxisNoiseProvider(ONE_PAIR, new double[] {1.0}, new double[] {SIGMA_ZERO}, MEDIAN_WINDOW, CAUSAL_WINDOW);
   }

   /**
    * A measurement that lies entirely in the pair's observable span is a perfectly rigid link
    * reporting a joint rate. There is nothing to distrust, so the law must add exactly nothing --
    * which is also what keeps a standing window identical to the baseline arm.
    */
   @Test
   public void testAMeasurementInsideTheJacobianSpanProducesNoInflationEver()
   {
      PairOffAxisNoiseProvider provider = provider();
      for (int t = 0; t < 20; t++)
      {
         double[] extra = provider.update(z(3.0 + t, 0.0, 0.0), xAxisJacobian());
         assertEquals(0.0, extra[0], 0.0, "tick " + t + ": an on-axis measurement must not inflate R");
      }
   }

   /**
    * A CONSTANT off-axis component is a mount misalignment or a bias, not a rigidity violation: it
    * makes this tick's measurement no worse than the last one's. The trailing median must absorb it
    * entirely, or the law degenerates into the hand-tuned constant it exists to replace.
    */
   @Test
   public void testAConstantOffAxisOffsetIsAbsorbedAsSlowAndDoesNotInflate()
   {
      PairOffAxisNoiseProvider provider = provider();
      for (int t = 0; t < 20; t++)
      {
         double[] extra = provider.update(z(0.0, 0.7, -0.4), xAxisJacobian());
         assertEquals(0.0, extra[0], 1.0e-12, "tick " + t + ": a constant misalignment is slow, not a violation");
      }
   }

   /**
    * The timing contract in full, on a single impulse: it is INVISIBLE on its own tick, it is held
    * for exactly the causal window afterwards, and then it is gone. With scale 1 and a unit
    * off-axis impulse the held value is sigma0 exactly, so the magnitude is pinned too.
    */
   @Test
   public void testAnImpulseIsInvisibleOnItsOwnTickThenHeldForExactlyTheCausalWindow()
   {
      PairOffAxisNoiseProvider provider = provider();
      double[] seen = new double[8];
      for (int t = 0; t < 8; t++)
         seen[t] = provider.update(t == 3 ? z(0.0, 1.0, 0.0) : z(0.0, 0.0, 0.0), xAxisJacobian())[0];

      assertEquals(0.0, seen[3], 1.0e-12, "the impulse tick itself must not see its own residual");
      for (int t : new int[] {4, 5, 6})
         assertEquals(SIGMA_ZERO, seen[t], 1.0e-12, "tick " + t + " must still hold the impulse");
      assertEquals(0.0, seen[7], 1.0e-12, "the impulse must fall out of the window after exactly " + CAUSAL_WINDOW + " ticks");
   }

   /**
    * The circularity guarantee, stated as an experiment rather than as an argument: two runs whose
    * histories agree everywhere except the newest tick must emit the same value on that tick. If
    * the law could see the innovation it is weighting, this fails.
    */
   @Test
   public void testTheEmittedValueCannotDependOnTheCurrentTick()
   {
      PairOffAxisNoiseProvider quiet = provider();
      PairOffAxisNoiseProvider violent = provider();
      double[] lastQuiet = null;
      double[] lastViolent = null;

      for (int t = 0; t < 12; t++)
      {
         boolean last = t == 11;
         lastQuiet = quiet.update(z(0.0, last ? 0.0 : 0.05 * t, 0.0), xAxisJacobian()).clone();
         lastViolent = violent.update(z(0.0, last ? 500.0 : 0.05 * t, 0.0), xAxisJacobian()).clone();
      }

      assertEquals(lastQuiet[0], lastViolent[0], 0.0,
                   "a huge residual on the current tick changed the current tick's R -- the law is reading the innovation it weights");
   }

   /** Sanity guard on the test above: the law must actually respond to something, or it proves nothing. */
   @Test
   public void testAFastOffAxisBurstDoesRaiseTheNoiseOnLaterTicks()
   {
      PairOffAxisNoiseProvider provider = provider();
      double raised = 0.0;
      for (int t = 0; t < 10; t++)
         raised = Math.max(raised, provider.update(t == 4 ? z(0.0, 0.0, 9.0) : z(0.0, 0.0, 0.0), xAxisJacobian())[0]);

      assertTrue(raised > SIGMA_ZERO, "a large fast off-axis burst must inflate R well above the baseline, was " + raised);
   }

   /**
    * A dropped IMU frame is one bad tick. If it poisoned the trailing max the pair would stay out of
    * service for the whole causal window afterwards -- a momentary sensor glitch turning into a
    * sustained filter change, which is far worse than the glitch.
    */
   @Test
   public void testANonFiniteMeasurementDoesNotPoisonTheWindowBehindIt()
   {
      PairOffAxisNoiseProvider provider = provider();
      for (int t = 0; t < 10; t++)
      {
         double[] extra = provider.update(t == 2 ? z(0.0, Double.NaN, 0.0) : z(0.0, 0.0, 0.0), xAxisJacobian());
         assertTrue(Double.isFinite(extra[0]), "tick " + t + " emitted a non-finite variance after a bad frame");
      }
   }

   /** Two replays in one process must not share a tail, or the second one's opening ticks are the first one's. */
   @Test
   public void testResetClearsTheHistory()
   {
      PairOffAxisNoiseProvider provider = provider();
      for (int t = 0; t < 6; t++)
         provider.update(z(0.0, 4.0 * t, 0.0), xAxisJacobian());

      provider.reset();
      assertEquals(0.0, provider.update(z(0.0, 0.0, 0.0), xAxisJacobian())[0], 0.0, "the first tick after a reset must carry nothing forward");
   }

   /**
    * A zero or non-finite scale divides the residual into an infinity and takes the filter's whole
    * stacked channel out of service on the first moving tick -- silently, because an infinite R just
    * looks like an ignored measurement.
    */
   @Test
   public void testMalformedConstantsAreRejectedAtConstruction()
   {
      assertThrows(IllegalArgumentException.class,
                   () -> new PairOffAxisNoiseProvider(ONE_PAIR, new double[] {0.0}, new double[] {1.0}, 5, 3));
      assertThrows(IllegalArgumentException.class,
                   () -> new PairOffAxisNoiseProvider(ONE_PAIR, new double[] {Double.NaN}, new double[] {1.0}, 5, 3));
      assertThrows(IllegalArgumentException.class,
                   () -> new PairOffAxisNoiseProvider(ONE_PAIR, new double[] {1.0}, new double[] {-1.0}, 5, 3));
      assertThrows(IllegalArgumentException.class,
                   () -> new PairOffAxisNoiseProvider(ONE_PAIR, new double[] {1.0, 1.0}, new double[] {1.0}, 5, 3));
      assertThrows(IllegalArgumentException.class,
                   () -> new PairOffAxisNoiseProvider(ONE_PAIR, new double[] {1.0}, new double[] {1.0}, 0, 3));
   }

   /** A pair-count mismatch at call time means the caller wired a different filter than the artifact describes. */
   @Test
   public void testAJacobianCountMismatchIsRejected()
   {
      PairOffAxisNoiseProvider provider = provider();
      DMatrixRMaj[] twoJacobians = {new DMatrixRMaj(3, 1), new DMatrixRMaj(3, 1)};
      assertThrows(IllegalArgumentException.class, () -> provider.update(z(0.0, 1.0, 0.0), twoJacobians));
   }

   /**
    * A 2-DoF pair observes a plane, so only the one remaining direction is off-axis. Checking this
    * separately matters because the DoF split is the mechanism's a priori signature -- the offline
    * study found 1-DoF pairs inflating far harder than 2-DoF ones, and that only follows if the
    * projector really does track the rank of J.
    */
   @Test
   public void testATwoDofPairOnlyCountsTheOneDirectionOutsideItsPlane()
   {
      DMatrixRMaj j = new DMatrixRMaj(3, 2);
      j.set(0, 0, 1.0); // span = the xy plane
      j.set(1, 1, 1.0);
      DMatrixRMaj[] jacobians = {j};

      PairOffAxisNoiseProvider inPlane = provider();
      PairOffAxisNoiseProvider outOfPlane = provider();
      double inPlaneMax = 0.0;
      double outOfPlaneMax = 0.0;
      for (int t = 0; t < 8; t++)
      {
         boolean burst = t == 3;
         inPlaneMax = Math.max(inPlaneMax, inPlane.update(burst ? z(6.0, 6.0, 0.0) : z(0.0, 0.0, 0.0), jacobians)[0]);
         outOfPlaneMax = Math.max(outOfPlaneMax, outOfPlane.update(burst ? z(0.0, 0.0, 6.0) : z(0.0, 0.0, 0.0), jacobians)[0]);
      }

      assertEquals(0.0, inPlaneMax, 1.0e-12, "a burst inside the observable plane carries joint information and must not inflate");
      assertTrue(outOfPlaneMax > 0.0, "a burst normal to the observable plane is a rigidity violation and must inflate");
   }
}
