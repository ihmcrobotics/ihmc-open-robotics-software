package us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher;

import us.ihmc.communication.packets.ScanPointFilter;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class RangeScanPointFilter implements ScanPointFilter
{
   private double minRangeSquared = Double.NaN;
   private double maxRangeSquared = Double.NaN;
   private Point3DReadOnly sensorPosition;

   public RangeScanPointFilter()
   {
   }

   public void setMinRange(double minRange)
   {
      if (minRange <= 0.0 || Double.isNaN(minRange))
         minRangeSquared = Double.NaN;
      else
         minRangeSquared = minRange * minRange;
   }

   public void setMaxRange(double maxRange)
   {
      if (maxRange <= 0.0 || Double.isNaN(maxRange))
         maxRangeSquared = Double.NaN;
      else
         maxRangeSquared = maxRange * maxRange;
   }

   public void setSensorPosition(Point3DReadOnly sensorPosition)
   {
      this.sensorPosition = sensorPosition;
   }

   @Override
   public boolean test(int index, Point3DReadOnly point)
   {
      if (Double.isNaN(minRangeSquared) && Double.isNaN(maxRangeSquared))
         return true;

      double distanceSquared = sensorPosition.distanceSquared(point);
      return distanceSquared >= minRangeSquared && distanceSquared <= maxRangeSquared;
   }
}
