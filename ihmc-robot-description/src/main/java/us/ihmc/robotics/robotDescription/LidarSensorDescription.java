package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class LidarSensorDescription extends SensorDescription
{
   private double sweepYawMin;
   private double sweepYawMax;

   private double heightPitchMin;
   private double heightPitchMax;

   private double minRange;
   private double maxRange;

   private int pointsPerSweep;
   private int scanHeight;

   public LidarSensorDescription(String name, RigidBodyTransform transformToJoint)
   {
      super(name, transformToJoint);
   }

   public double getSweepYawMin()
   {
      return sweepYawMin;
   }

   public double getSweepYawMax()
   {
      return sweepYawMax;
   }

   public double getHeightPitchMin()
   {
      return heightPitchMin;
   }

   public double getHeightPitchMax()
   {
      return heightPitchMax;
   }

   public double getMinRange()
   {
      return minRange;
   }

   public double getMaxRange()
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

   public void setSweepYawLimits(double min, double max)
   {
      sweepYawMin = min;
      sweepYawMax = max;
   }

   public void setSweepYawMin(double sweepYawMin)
   {
      this.sweepYawMin = sweepYawMin;
   }

   public void setSweepYawMax(double sweepYawMax)
   {
      this.sweepYawMax = sweepYawMax;
   }

   public void setHeightPitchLimits(double min, double max)
   {
      heightPitchMin = min;
      heightPitchMax = max;
   }

   public void setHeightPitchMin(double heightPitchMin)
   {
      this.heightPitchMin = heightPitchMin;
   }

   public void setHeightPitchMax(double heightPitchMax)
   {
      this.heightPitchMax = heightPitchMax;
   }

   public void setRangeLimits(double min, double max)
   {
      minRange = min;
      maxRange = max;
   }

   public void setMinRange(double minRange)
   {
      this.minRange = minRange;
   }

   public void setMaxRange(double maxRange)
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
}
