package us.ihmc.humanoidBehaviors.tools.perception;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

public class LidarScanPointCloudAggregator
{
   private final int scanHistorySize = 50;
   private final ArrayDeque<ArrayList<Point3DReadOnly>> pointCloud = new ArrayDeque<>();

   public void addScan(ArrayList<Point3DReadOnly> scan)
   {
      pointCloud.addFirst(scan);

      while (pointCloud.size() > scanHistorySize)
      {
         pointCloud.removeLast();
      }
   }

   public List<Point3DReadOnly> getPointCloud()
   {
      ArrayList<Point3DReadOnly> pointCloudInstance = new ArrayList<>();
      synchronized (this)
      {
         for (ArrayList<Point3DReadOnly> point : pointCloud)
         {
            pointCloudInstance.addAll(point);
         }
      }
      return pointCloudInstance;
   }
}
