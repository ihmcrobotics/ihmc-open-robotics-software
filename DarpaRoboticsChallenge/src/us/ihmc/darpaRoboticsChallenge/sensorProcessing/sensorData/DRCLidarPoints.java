package us.ihmc.darpaRoboticsChallenge.sensorProcessing.sensorData;

import javax.vecmath.Point3d;

public class DRCLidarPoints
{
   public Point3d[] lidarPoints;

   public Point3d[] getLidarPoints()
   {
      return lidarPoints;
   }

   public void setLidarPoints(Point3d[] lidarPoints)
   {
      this.lidarPoints = lidarPoints;
   }

}
