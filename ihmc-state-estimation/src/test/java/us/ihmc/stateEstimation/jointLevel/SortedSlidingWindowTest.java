package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.management.ManagementFactory;
import java.util.Arrays;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

/**
 * The streaming trailing median must be the re-sorted median it replaced, bit for bit -- the offline
 * reference and every figure computed so far use the sort -- and the off-axis provider built on it must
 * be usable inside a 1 kHz estimator thread: no allocation per tick.
 */
public class SortedSlidingWindowTest
{
   /** The implementation this replaces: copy the last min(n, capacity) samples and sort them. */
   private static double sortedMedian(double[] history, int count, int capacity)
   {
      int available = Math.min(count, capacity);
      double[] window = Arrays.copyOfRange(history, count - available, count);
      Arrays.sort(window);
      int half = available / 2;
      return available % 2 == 1 ? window[half] : 0.5 * (window[half - 1] + window[half]);
   }

   @Test
   public void testTheMedianIsBitIdenticalToSortingTheWindowIncludingTiesSignedZerosAndNaN()
   {
      Random random = new Random(1);
      for (int capacity : new int[] {1, 2, 3, 7, 500})
      {
         SortedSlidingWindow window = new SortedSlidingWindow(capacity);
         double[] history = new double[3000];
         for (int n = 0; n < history.length; n++)
         {
            double v;
            int kind = random.nextInt(20);
            if (kind == 0)
               v = Double.NaN;
            else if (kind == 1)
               v = -0.0;
            else if (kind == 2)
               v = 0.0;
            else if (kind < 8)
               v = random.nextInt(5) - 2;        // many exact ties
            else
               v = random.nextGaussian();
            history[n] = v;
            window.add(v);
            double expected = sortedMedian(history, n + 1, capacity);
            assertEquals(Double.doubleToRawLongBits(expected), Double.doubleToRawLongBits(window.median()),
                         "capacity " + capacity + ", sample " + n + ": expected " + expected + " got " + window.median());
            assertEquals(Math.min(n + 1, capacity), window.size());
         }
      }
   }

   @Test
   public void testClearStartsOver()
   {
      SortedSlidingWindow window = new SortedSlidingWindow(3);
      window.add(5.0);
      window.add(6.0);
      window.clear();
      window.add(1.0);
      assertEquals(1.0, window.median(), 0.0);
      assertEquals(1, window.size());
   }

   /**
    * Seven pairs at the deployed windows (W = 500, C = 100), 1- and 2-DoF chains as on Alex. After the
    * first tick has built each pair's SVD storage, update() must allocate nothing: garbage on the
    * estimator thread is what turns a fast method into a jittery one. The per-tick time is reported.
    */
   @Test
   public void testTheOffAxisProviderAllocatesNothingPerTickAtDeployedSize()
   {
      int pairs = 7;
      String[] chains = new String[pairs];
      double[] scale = new double[pairs];
      double[] sigmaZero = new double[pairs];
      DMatrixRMaj[] jacobians = new DMatrixRMaj[pairs];
      Random random = new Random(2);
      for (int e = 0; e < pairs; e++)
      {
         chains[e] = "imu" + e + "->imu" + (e + 1);
         scale[e] = 0.005;
         sigmaZero[e] = 2.0e-6;
         int dof = (e == 2 || e == 5) ? 2 : 1;
         jacobians[e] = new DMatrixRMaj(3, dof);
         for (int k = 0; k < 3 * dof; k++)
            jacobians[e].data[k] = random.nextGaussian();
      }
      PairOffAxisNoiseProvider provider = new PairOffAxisNoiseProvider(chains, scale, sigmaZero, 500, 100);
      DMatrixRMaj zg = new DMatrixRMaj(3 * pairs, 1);

      for (int t = 0; t < 2000; t++) // warm-up: JIT, and the per-shape SVD storage
         tick(provider, zg, jacobians, random);

      com.sun.management.ThreadMXBean threads = (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();
      long thread = Thread.currentThread().getId();
      int ticks = 20000;
      long[] nanos = new long[ticks];
      long before = threads.getThreadAllocatedBytes(thread);
      for (int t = 0; t < ticks; t++)
      {
         long start = System.nanoTime();
         tick(provider, zg, jacobians, random);
         nanos[t] = System.nanoTime() - start;
      }
      long allocated = threads.getThreadAllocatedBytes(thread) - before;

      Arrays.sort(nanos);
      System.out.printf("PairOffAxisNoiseProvider.update, 7 pairs, W=500: median %.1f us, p99 %.1f us, max %.1f us, %d bytes over %d ticks%n",
                        nanos[ticks / 2] / 1e3, nanos[(int) (ticks * 0.99)] / 1e3, nanos[ticks - 1] / 1e3, allocated, ticks);
      assertEquals(0L, allocated, "update() allocated " + allocated + " bytes over " + ticks + " ticks");
      assertTrue(nanos[ticks / 2] < 200_000, "median tick time " + nanos[ticks / 2] / 1e3 + " us is a fifth of the 1 ms budget");
   }

   private static void tick(PairOffAxisNoiseProvider provider, DMatrixRMaj zg, DMatrixRMaj[] jacobians, Random random)
   {
      for (int i = 0; i < zg.data.length; i++)
         zg.data[i] = 0.01 * random.nextGaussian();
      provider.update(zg, jacobians);
   }
}
