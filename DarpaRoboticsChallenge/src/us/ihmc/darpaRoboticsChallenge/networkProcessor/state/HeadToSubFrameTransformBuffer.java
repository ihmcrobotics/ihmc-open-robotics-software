package us.ihmc.darpaRoboticsChallenge.networkProcessor.state;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.kinematics.TimeStampedTransform3D;
import us.ihmc.robotics.kinematics.TransformInterpolationCalculator;

public class HeadToSubFrameTransformBuffer implements PendableBuffer
{
   private final int size;
   private final TimeStampedTransform3D[] transforms;
   private int currentIndex;
   private long oldestTimestamp;
   private long newestTimestamp;
   private TransformInterpolationCalculator transformInterpolationCalculator;

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

   public synchronized boolean interpolate(long timestamp, TimeStampedTransform3D timeStampedTransform3DToPack)
   {
      if (!isInRange(timestamp))
      {
         return false;
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

            TimeStampedTransform3D ceilingData = transforms[index];
            
            transformInterpolationCalculator.interpolate(floorData, ceilingData, timeStampedTransform3DToPack, timestamp);
            return true;
         }
      }

      return false;
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
   
   @Override
   public long oldestTimestamp()
   {
      return oldestTimestamp;
   }
}
