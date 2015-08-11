package us.ihmc.robotics.lidar;

import java.util.Random;

public class LidarScanParameters
{
   public long timestamp;

   public float sweepYawMax;
   public float sweepYawMin;

   public float timeIncrement;
   // ROS scanTime - time it took the do the sweep
   public float scanTime;

   public float minRange;
   public float maxRange;

   public int pointsPerSweep;



   // NEED this constructor for serialization
   public LidarScanParameters()
   {
   }

   public LidarScanParameters(LidarScanParameters p, long timestamp)
   {
      this(p.pointsPerSweep, p.sweepYawMin, p.sweepYawMax, p.timeIncrement, p.minRange, p.maxRange, p.scanTime, timestamp);
   }

   public LidarScanParameters(int pointsPerSweep, float sweepYawMin, float sweepYawMax, float timeIncrement, float minRange, float maxRange, float scanTime)
   {
      this(pointsPerSweep, sweepYawMin, sweepYawMax, timeIncrement, minRange, maxRange, scanTime, 0l);
   }

   public LidarScanParameters(int pointsPerSweep, float sweepYawMin, float sweepYawMax, float minRange, float maxRange)
   {
      this(pointsPerSweep, sweepYawMin, sweepYawMax, 0f, minRange, maxRange, 0f, 0l);
   }

   public LidarScanParameters(int pointsPerSweep, double sweepYawMin, double sweepYawMax, double timeIncrement, double minRange, double maxRange,
                              double scanTime)
   {
      this(pointsPerSweep, (float) sweepYawMin, (float) sweepYawMax, (float) timeIncrement, (float) minRange, (float) maxRange, (float) scanTime, 0l);
   }

   public LidarScanParameters(int pointsPerSweep, double sweepYawMin, double sweepYawMax, double minRange, double maxRange)
   {
      this(pointsPerSweep, (float) sweepYawMin, (float) sweepYawMax, 0, (float) minRange, (float) maxRange, 0, 0l);
   }

   public LidarScanParameters(int pointsPerSweep, double fieldOfView, double minRange, double maxRange)
   {
      this(pointsPerSweep, (float) (-fieldOfView / 2), (float) (fieldOfView / 2), 0, (float) minRange, (float) maxRange, 0, 0l);
   }

   public LidarScanParameters(int pointsPerSweep, float sweepYawMin, float sweepYawMax, float timeIncrement, float minRange, float maxRange, float scanTime,
                              long timestamp)
   {
      this.timestamp = timestamp;

      this.sweepYawMax = sweepYawMax;
      this.sweepYawMin = sweepYawMin;

      this.timeIncrement = timeIncrement;
      this.scanTime = scanTime;

      this.minRange = minRange;
      this.maxRange = maxRange;

      this.pointsPerSweep = pointsPerSweep;

   }

   public LidarScanParameters(Random random)
   {
      timestamp = random.nextLong();     
      
      sweepYawMax = random.nextFloat();  
      sweepYawMin = random.nextFloat();  
                    
      timeIncrement = random.nextFloat();
      scanTime = random.nextFloat();    
                    
      minRange = random.nextFloat();
      maxRange = random.nextFloat();
                    
     pointsPerSweep = random.nextInt();
   }

   public String toString()
   {
      return "PolarLidarScanParameters{" + "\n sweepYawMax=" + sweepYawMax + ",\n sweepYawMin=" + sweepYawMin + ",\n angleIncrement=" + ",\n timeIncrement="
             + timeIncrement + ",\n scanTime=" + scanTime + ",\n minRange=" + minRange + ",\n maxRange=" + maxRange + ",\n pointsPerSweep=" + pointsPerSweep
             + '}';
   }

   public float getFieldOfView()
   {
      return sweepYawMax - sweepYawMin;
   }
   
   public long getTimestamp()
   {
      return timestamp;
   }
   
   public float getTimeIncrement()
   {
      return timeIncrement;
   }
   
   public float getScanTime()
   {
      return scanTime;
   }

   public long getScanTimeNanos()
   {
      return (long) (scanTime * 1000000000);
   }

   public float getSweepYawMax()
   {
      return sweepYawMax;
   }

   public float getSweepYawMin()
   {
      return sweepYawMin;
   }
   
   public float getAngleIncrement()
   {
      return (sweepYawMax - sweepYawMin) / pointsPerSweep;
   }

   public float getMinRange()
   {
      return minRange;
   }

   public float getMaxRange()
   {
      return maxRange;
   }

   public int getPointsPerSweep()
   {
      return pointsPerSweep;
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + Float.floatToIntBits(maxRange);
      result = prime * result + Float.floatToIntBits(minRange);
      result = prime * result + pointsPerSweep;
      result = prime * result + Float.floatToIntBits(scanTime);
      result = prime * result + Float.floatToIntBits(sweepYawMax);
      result = prime * result + Float.floatToIntBits(sweepYawMin);
      result = prime * result + Float.floatToIntBits(timeIncrement);
      result = prime * result + (int) (timestamp ^ (timestamp >>> 32));
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      LidarScanParameters other = (LidarScanParameters) obj;
      if (Float.floatToIntBits(maxRange) != Float.floatToIntBits(other.maxRange))
         return false;
      if (Float.floatToIntBits(minRange) != Float.floatToIntBits(other.minRange))
         return false;
      if (pointsPerSweep != other.pointsPerSweep)
         return false;
      if (Float.floatToIntBits(scanTime) != Float.floatToIntBits(other.scanTime))
         return false;
      if (Float.floatToIntBits(sweepYawMax) != Float.floatToIntBits(other.sweepYawMax))
         return false;
      if (Float.floatToIntBits(sweepYawMin) != Float.floatToIntBits(other.sweepYawMin))
         return false;
      if (Float.floatToIntBits(timeIncrement) != Float.floatToIntBits(other.timeIncrement))
         return false;
      if (timestamp != other.timestamp)
         return false;
      return true;
   }
   
}
