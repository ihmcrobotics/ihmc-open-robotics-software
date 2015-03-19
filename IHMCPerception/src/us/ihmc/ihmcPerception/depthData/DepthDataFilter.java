package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.communication.packets.sensing.PointCloudPacket;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public class DepthDataFilter extends DepthDataStore
{
   // Adjustment which is applied to LIDAR points.  Must be applied while points are still in LIDAR frame.  Allows users correct for errors
   // See DRCManualLidarTransform and DRCLidarVisualizationManager. This is a bit of a hack but less likely to have unintended consequences.
   private final RigidBodyTransform worldToCorrected = new RigidBodyTransform();


   public DepthDataFilter()
   {
      super();
   }

   public void setWorldToCorrected(RigidBodyTransform adjustment)
   {
      this.worldToCorrected.set(adjustment);
   }

   public boolean addPoint(Point3d point, Point3d sensorOrigin)
   {
      boolean send = false;
      if(!pointInRange(point, sensorOrigin))
         return false;

      // This is here so the user can manually correct for calibration errors.  It should only be not identity in the user interface
      if (DepthDataFilterParameters.LIDAR_ADJUSTMENT_ACTIVE)
         worldToCorrected.transform(point);

      if (parameters.nearScan && isValidNearScan(point, sensorOrigin))
      {
         send = nearScan.add(point.x, point.y, point.z) || send;
      }

      if (isValidPoint(point, sensorOrigin))
      {
         if (isPossibleGround(point, sensorOrigin))
         {
            send = quadTree.addPoint(point.x, point.y, point.z) || send;
         }
      }

      return send;
   }

   protected boolean isValidNearScan(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = true;
      valid &= point.z < lidarOrigin.z + parameters.nearScanZMaxAboveHead;

      Point3d center = new Point3d(lidarOrigin.x, lidarOrigin.y, point.z);
      valid &= point.distance(center) < parameters.nearScanRadius;


      return valid;
   }

   protected boolean isValidPoint(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = true;

      valid &= point.getZ() < lidarOrigin.getZ() + parameters.octreeZMaxAboveHead;;

      return valid;
   }

   protected boolean isPossibleGround(Point3d point, Point3d lidarOrigin)
   {
      final double footZ = 0;

      return (point.z - footZ) < parameters.quadTreeZMax;
   }

   private boolean pointInRange(Point3d point, Point3d sensorOrigin)
   {
      double dist = sensorOrigin.distance(point);

      return (dist > parameters.minRange) && (dist < parameters.maxRange);
   }

   @Deprecated
   public PointCloudPacket filterPolarLidarScan(LidarScan lidarScan)
   {
      ArrayList<Point3d> points = new ArrayList<>();
      for (int i = 0; i < lidarScan.size(); i++)
      {
         addLidarScan(lidarScan, i, points);
      }
      Point3d origin = new Point3d();
      Vector3d worldVector = new Vector3d();
      lidarScan.getAverageTransform().get(worldVector);
      origin.set(worldVector);
      return new PointCloudPacket(origin, points.toArray(new Point3d[points.size()]), lidarScan.params.timestamp);

   }
   
   @Deprecated
   private void addLidarScan(LidarScan lidarScan, int i, ArrayList<Point3d> points)
   {
      Point3d lidarOrigin = new Point3d();
      lidarScan.getAverageTransform().transform(lidarOrigin);

  
         Point3d point = lidarScan.getPoint(i);
         if(addPoint(point, lidarOrigin))
         {
            points.add(point);
         }
   }
}
