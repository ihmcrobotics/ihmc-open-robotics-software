package us.ihmc.avatar.logProcessor.leRobot;

/**
 * Utility class to calculate statistics (mean and standard deviation) for float record fields.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
public class LeRobotFloatStatisticsCalculator
{
   private int count = 0;
   private float sum = 0L;
   private float sumSquares = 0L;
   private float mean = 0.0f;
   private float stddev = 0.0f;
   private float min = Float.MAX_VALUE;
   private float max = -Float.MAX_VALUE;

   /**
    * Add a value to the statistics calculation
    * @param value the value to add
    */
   public void addValue(float value)
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
         mean = sum / count;
         // Standard deviation formula: sqrt((sum_of_squares/n) - (mean)²)
         stddev = (float) Math.sqrt((sumSquares / count) - (mean * mean));
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
   public float getSum()
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
   public float getMin()
   {
      return min;
   }

   /**
    * Get the maximum value seen
    * @return the maximum value
    */
   public float getMax()
   {
      return max;
   }

   public void mergeFrom(LeRobotFloatStatisticsCalculator other)
   {
      count += other.count;
      sum += other.sum;
      sumSquares += other.sumSquares;
      min = Math.min(min, other.min);
      max = Math.max(max, other.max);
   }
}
