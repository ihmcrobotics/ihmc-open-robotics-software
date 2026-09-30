package us.ihmc.avatar.logProcessor.leRobot;

/**
 * Utility class to calculate statistics (mean and standard deviation) for integer record fields.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
public class LeRobotIntegerStatisticsCalculator
{
   private int count = 0;
   private long sum = 0L;
   private long sumSquares = 0L;
   private float mean = 0.0f;
   private float stddev = 0.0f;
   private long min = Long.MAX_VALUE;
   private long max = Long.MIN_VALUE;

   /**
    * Add a value to the statistics calculation
    * @param value the value to add
    */
   public void addValue(long value)
   {
      count++;
      sum += value;
      sumSquares += value * value;
      min = Math.min(min, value);
      max = Math.max(max, value);
   }

   /**
    * Calculate the mean and standard deviation from the collected values
    */
   public void calculate()
   {
      if (count > 0)
      {
         mean = (float) sum / count;
         // Standard deviation formula: sqrt((sum_of_squares/n) - (mean)²)
         stddev = (float) Math.sqrt(((float) sumSquares / count) - (mean * mean));
      }
   }

   /**
    * Get the count of values added
    * @return the count
    */
   public int getCount()
   {
      return count;
   }

   /**
    * Get the sum of all values added
    * @return the sum
    */
   public long getSum()
   {
      return sum;
   }

   /**
    * Get the calculated mean
    * @return the mean
    */
   public float getMean()
   {
      return mean;
   }

   /**
    * Get the calculated standard deviation
    * @return the standard deviation
    */
   public float getStddev()
   {
      return stddev;
   }

   /**
    * Get the minimum value seen
    * @return the minimum value
    */
   public long getMin()
   {
      return min;
   }

   /**
    * Get the maximum value seen
    * @return the maximum value
    */
   public long getMax()
   {
      return max;
   }

   public void mergeFrom(LeRobotIntegerStatisticsCalculator other)
   {
      count += other.count;
      sum += other.sum;
      sumSquares += other.sumSquares;
      min = Math.min(min, other.min);
      max = Math.max(max, other.max);
   }
}
