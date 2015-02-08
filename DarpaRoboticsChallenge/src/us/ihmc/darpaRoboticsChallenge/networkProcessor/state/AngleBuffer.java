package us.ihmc.darpaRoboticsChallenge.networkProcessor.state;


public class AngleBuffer implements PendableBuffer
{
   private final int size;

   private long[] timestamps;
   private double[] angles;

   private int currentIndex;
   private long oldestTimestamp;
   private long newestTimestamp;

   public AngleBuffer(int size)
   {
      this.size = size;
      timestamps = new long[size];
      angles = new double[size];

      for (int i = 0; i < size; i++)
      {
         timestamps[i] = Long.MIN_VALUE;
      }
   }

   public synchronized void addAngle(long timestamp, double angle)
   {
      timestamps[currentIndex] = timestamp;
      angles[currentIndex] = angle;

      newestTimestamp = timestamp;
      currentIndex++;

      if (currentIndex >= size)
      {
         currentIndex = 0;
      }

      if (timestamps[currentIndex] == Long.MIN_VALUE)
      {
         oldestTimestamp = newestTimestamp;
      }
      else
      {
         oldestTimestamp = timestamps[currentIndex];
      }
   }

   public synchronized boolean isInRange(long timestamp)
   {
      return ((timestamp >= oldestTimestamp) && (timestamp <= newestTimestamp));
   }

   public synchronized double interpolate(long timestamp)
   {
      if (!isInRange(timestamp))
      {
         return Double.NaN;
      }

      for (int i = currentIndex - 1; i >= -size + currentIndex; i--)
      {
         int index = (i < 0) ? size + i : i;

         if (timestamps[index] == timestamp)
         {
            return angles[index];
         }
         else if (timestamps[index] < timestamp)
         {
            long floor = timestamps[index];
            double floorAngle = angles[index];
            index++;

            if (index >= size)
            {
               index = 0;
            }

            long ceiling = timestamps[index];
            double ceilingAngle = angles[index];


            double percentage = ((double) (timestamp - floor)) / ((double) (ceiling - floor));

            return floorAngle + percentage * (ceilingAngle - floorAngle);
         }

      }

      return 0.0;

   }

   public synchronized boolean isPending(long timestamp)
   {
      return timestamp > newestTimestamp;
   }
   
   public synchronized boolean isOlderThanOldestTimestamp(long timestamp)
   {
	   return timestamp > newestTimestamp;
   }

   @Override
   public long newestTimestamp()
   {
      return newestTimestamp;
   }
   
   public long oldestTimestamp()
   {
      return oldestTimestamp;
   }
}
