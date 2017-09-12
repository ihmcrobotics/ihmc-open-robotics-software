package us.ihmc.humanoidRobotics.communication.subscribers;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.kinematics.TransformInterpolationCalculator;

public class TimeStampedTransformBuffer
{
   private final int size;
   private final TimeStampedTransform3D[] buffer;
   private int currentIndex;
   private long oldestTimeStamp = Long.MAX_VALUE;
   private long newestTimestamp;
   private boolean filledBufferAtleastOnce;

   private final TransformInterpolationCalculator transformInterpolationCalculator = new TransformInterpolationCalculator();
   
   public TimeStampedTransformBuffer(int size)
   {
      this.size = size;
      this.buffer = new TimeStampedTransform3D[size];
      for(int i = 0; i < buffer.length; i++)
      {
         this.buffer[i] = new TimeStampedTransform3D();
         this.buffer[i].setTimeStamp(Long.MAX_VALUE);
      }
      currentIndex = 0;
   }

   /**
    * Try to find the transform at the given timestamp (key). Interpolate between two registered timestamps if the one asked for hasn't been registered.
    * @param timestamp
    * @param timeStampedTransform3DToPack
    * @return
    */
   public boolean findTransform(long timestamp, TimeStampedTransform3D timeStampedTransform3DToPack)
   {
      if (!isInRange(timestamp))
         return false;

      for (int i = currentIndex - 1; i >= -size + currentIndex; i--)
      {
         int index = (i < 0) ? size + i : i;

         TimeStampedTransform3D floorData = buffer[index];
         if (floorData.getTimeStamp() == timestamp)
         {
            timeStampedTransform3DToPack.set(floorData);
            return true;
         }
         else if (floorData.getTimeStamp() < timestamp)
         {
            index++;

            if (index >= size)
            {
               index = 0;
            }

            TimeStampedTransform3D ceilingData = buffer[index];

            transformInterpolationCalculator.interpolate(floorData, ceilingData, timeStampedTransform3DToPack, timestamp);
            return true;
         }
      }

      return false;
   }

   public void put(RigidBodyTransform newestTransform, long timeStamp)
   {
      TimeStampedTransform3D recycledTimestampedTransformFromBuffer = buffer[currentIndex];
      recycledTimestampedTransformFromBuffer.setTimeStamp(timeStamp);
      recycledTimestampedTransformFromBuffer.setTransform3D(newestTransform);
      
      newestTimestamp = timeStamp;
      currentIndex++;

      if (currentIndex >= size)
      {
         currentIndex = 0;
         filledBufferAtleastOnce = true;
      }

      if (oldestTimeStamp == Long.MAX_VALUE)
      {
         oldestTimeStamp = newestTimestamp;
      }
      else if(filledBufferAtleastOnce)
      {
         oldestTimeStamp = buffer[currentIndex].getTimeStamp();
      }
   }

   public boolean isInRange(long timestamp)
   {
      return ((timestamp >= oldestTimeStamp) && (timestamp <= newestTimestamp));
   }

   public long getNewestTimestamp()
   {
      return newestTimestamp;
   }

   public long getOldestTimestamp()
   {
      return oldestTimeStamp;
   }
}
