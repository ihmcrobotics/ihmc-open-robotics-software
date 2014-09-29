package us.ihmc.darpaRoboticsChallenge.networkProcessor.state;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;

import us.ihmc.utilities.kinematics.TimeStampedTransform3D;

public class HeadToSubFrameTransformBuffer implements PendableBuffer
{
   private final int size;
   private final TimeStampedTransform3D[] transforms;
   private int currentIndex;
   private long oldestTimestamp;
   private long newestTimestamp;

   public HeadToSubFrameTransformBuffer(int size)
   {
      this.size = size;
      transforms = new TimeStampedTransform3D[size];

      for (int i = 0; i < size; i++)
      {
         transforms[i] = new TimeStampedTransform3D();
         transforms[i].setTimeStamp(Long.MIN_VALUE);
      }
   }

   public synchronized void addRawTransformAndTimestamp(long timestamp, RigidBodyTransform transform3D)
   {
      transforms[currentIndex].setTimeStamp(timestamp);
      transforms[currentIndex].setTransform3D(transform3D);
      newestTimestamp = timestamp;
      currentIndex++;

      if (currentIndex >= size)
      {
         currentIndex = 0;
      }

      if (transforms[currentIndex].getTimeStamp() == Long.MIN_VALUE)
      {
         oldestTimestamp = newestTimestamp;
      }
      else
      {
         oldestTimestamp = transforms[currentIndex].getTimeStamp();
      }
   }

   public synchronized void addTimeStampedTransform(TimeStampedTransform3D timeStampedTransform3D)
   {
      transforms[currentIndex].setTransform3D(timeStampedTransform3D.getTransform3D());
      transforms[currentIndex].setTimeStamp(timeStampedTransform3D.getTimeStamp());
      newestTimestamp = timeStampedTransform3D.getTimeStamp();

      currentIndex++;

      if (currentIndex >= size)
      {
         currentIndex = 0;
      }

      if (transforms[currentIndex].getTimeStamp() == Long.MIN_VALUE)
      {
         oldestTimestamp = newestTimestamp;
      }
      else
      {
         oldestTimestamp = transforms[currentIndex].getTimeStamp();
      }
   }

   public synchronized boolean isInRange(long timestamp)
   {
      return ((timestamp >= oldestTimestamp) && (timestamp <= newestTimestamp));
   }

   public synchronized TimeStampedTransform3D interpolate(long timestamp)
   {
      if (!isInRange(timestamp))
      {
         return null;
      }

      for (int i = currentIndex - 1; i >= -size + currentIndex; i--)
      {
         int index = (i < 0) ? size + i : i;
         TimeStampedTransform3D floorData = transforms[index];

         if (floorData == null)
         {
            break;
         }

         if (floorData.getTimeStamp() == timestamp)
         {
            return floorData;
         }
         else if (floorData.getTimeStamp() < timestamp)
         {
            long floor = floorData.getTimeStamp();

            index++;

            if (index >= size)
            {
               index = 0;
            }

            TimeStampedTransform3D ceilingData = transforms[index];
            long ceiling = ceilingData.getTimeStamp();
            double percentage = ((double) (timestamp - floor)) / ((double) (ceiling - floor));

            return floorData.interpolateTo(ceilingData, percentage);
         }
      }

      return null;
   }

   @Override
   public synchronized boolean isPending(long timestamp)
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
