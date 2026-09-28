package us.ihmc.stateEstimation.jointLevel;

import java.util.Arrays;

/**
 * The last {@code capacity} samples of a stream, kept both in arrival order (a ring) and sorted, so the
 * trailing median costs a binary search and one array shift per sample instead of a sort of the whole
 * window. Allocation-free after construction, which is what a 1 kHz estimator thread needs.
 * <p>
 * Ordering is {@link Double#compare}, the order {@link Arrays#sort(double[])} uses (NaN last, -0.0 before
 * 0.0), so {@link #median()} is bit-identical to copying the window and sorting it -- the implementation
 * this replaces, and the one the offline reference computes.
 */
final class SortedSlidingWindow
{
   private final double[] ring;
   private final double[] sorted;
   private int size = 0;
   private int next = 0;

   SortedSlidingWindow(int capacity)
   {
      if (capacity < 1)
         throw new IllegalArgumentException("capacity must be at least 1, got " + capacity);
      ring = new double[capacity];
      sorted = new double[capacity];
   }

   /** Adds a sample, evicting the oldest once the window is full. */
   void add(double value)
   {
      if (size == ring.length)
      {
         int remove = indexOf(ring[next]);
         System.arraycopy(sorted, remove + 1, sorted, remove, size - remove - 1);
         size--;
      }
      ring[next] = value;
      next = (next + 1) % ring.length;

      int insert = insertionPoint(value);
      System.arraycopy(sorted, insert, sorted, insert + 1, size - insert);
      sorted[insert] = value;
      size++;
   }

   /** Median of the samples currently held; the mean of the two middle ones for an even count. NaN if empty. */
   double median()
   {
      if (size == 0)
         return Double.NaN;
      int half = size / 2;
      return size % 2 == 1 ? sorted[half] : 0.5 * (sorted[half - 1] + sorted[half]);
   }

   int size()
   {
      return size;
   }

   void clear()
   {
      size = 0;
      next = 0;
   }

   /** First index whose element is greater than {@code value} (stable: equal elements stay ahead). */
   private int insertionPoint(double value)
   {
      int low = 0, high = size;
      while (low < high)
      {
         int mid = (low + high) >>> 1;
         if (Double.compare(sorted[mid], value) <= 0)
            low = mid + 1;
         else
            high = mid;
      }
      return low;
   }

   /** Index of an element equal to {@code value} under {@link Double#compare}; it is known to be present. */
   private int indexOf(double value)
   {
      int low = 0, high = size - 1;
      while (low <= high)
      {
         int mid = (low + high) >>> 1;
         int c = Double.compare(sorted[mid], value);
         if (c < 0)
            low = mid + 1;
         else if (c > 0)
            high = mid - 1;
         else
            return mid;
      }
      throw new IllegalStateException("evicted sample " + value + " is not in the sorted window");
   }
}
