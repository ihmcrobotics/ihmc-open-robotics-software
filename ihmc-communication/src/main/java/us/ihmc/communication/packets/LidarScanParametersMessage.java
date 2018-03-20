package us.ihmc.communication.packets;

import us.ihmc.commons.MathTools;

public class LidarScanParametersMessage extends Packet<LidarScanParametersMessage>
{
   public long timestamp;

   public float sweepYawMax;
   public float sweepYawMin;

   public float heightPitchMax;
   public float heightPitchMin;

   public float timeIncrement;
   public float scanTime;

   public float minRange;
   public float maxRange;

   public int pointsPerSweep;
   public int scanHeight;

   public LidarScanParametersMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(LidarScanParametersMessage other)
   {
      timestamp = other.timestamp;
      sweepYawMax = other.sweepYawMax;
      sweepYawMin = other.sweepYawMin;
      heightPitchMax = other.heightPitchMax;
      heightPitchMin = other.heightPitchMin;
      timeIncrement = other.timeIncrement;
      scanTime = other.scanTime;
      minRange = other.minRange;
      maxRange = other.maxRange;
      pointsPerSweep = other.pointsPerSweep;
      scanHeight = other.scanHeight;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public float getSweepYawMax()
   {
      return sweepYawMax;
   }

   public float getSweepYawMin()
   {
      return sweepYawMin;
   }

   public float getHeightPitchMax()
   {
      return heightPitchMax;
   }

   public float getHeightPitchMin()
   {
      return heightPitchMin;
   }

   public float getTimeIncrement()
   {
      return timeIncrement;
   }

   public float getScanTime()
   {
      return scanTime;
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

   public int getScanHeight()
   {
      return scanHeight;
   }

   public void setTimestamp(long timestamp)
   {
      this.timestamp = timestamp;
   }

   public void setSweepYawMax(float sweepYawMax)
   {
      this.sweepYawMax = sweepYawMax;
   }

   public void setSweepYawMin(float sweepYawMin)
   {
      this.sweepYawMin = sweepYawMin;
   }

   public void setHeightPitchMax(float heightPitchMax)
   {
      this.heightPitchMax = heightPitchMax;
   }

   public void setHeightPitchMin(float heightPitchMin)
   {
      this.heightPitchMin = heightPitchMin;
   }

   public void setTimeIncrement(float timeIncrement)
   {
      this.timeIncrement = timeIncrement;
   }

   public void setScanTime(float scanTime)
   {
      this.scanTime = scanTime;
   }

   public void setMinRange(float minRange)
   {
      this.minRange = minRange;
   }

   public void setMaxRange(float maxRange)
   {
      this.maxRange = maxRange;
   }

   public void setPointsPerSweep(int pointsPerSweep)
   {
      this.pointsPerSweep = pointsPerSweep;
   }

   public void setScanHeight(int scanHeight)
   {
      this.scanHeight = scanHeight;
   }

   @Override
   public boolean epsilonEquals(LidarScanParametersMessage other, double epsilon)
   {
      if (timestamp != other.timestamp)
         return false;
      if (!MathTools.epsilonEquals(sweepYawMax, other.sweepYawMax, epsilon))
         return false;
      if (!MathTools.epsilonEquals(sweepYawMin, other.sweepYawMin, epsilon))
         return false;
      if (!MathTools.epsilonEquals(heightPitchMax, other.heightPitchMax, epsilon))
         return false;
      if (!MathTools.epsilonEquals(heightPitchMin, other.heightPitchMin, epsilon))
         return false;
      if (!MathTools.epsilonEquals(timeIncrement, other.timeIncrement, epsilon))
         return false;
      if (!MathTools.epsilonEquals(scanTime, other.scanTime, epsilon))
         return false;
      if (!MathTools.epsilonEquals(minRange, other.minRange, epsilon))
         return false;
      if (!MathTools.epsilonEquals(maxRange, other.maxRange, epsilon))
         return false;
      if (pointsPerSweep != other.pointsPerSweep)
         return false;
      if (scanHeight != other.scanHeight)
         return false;
      return true;
   }
}
