package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;

public class OcclussionTools
{
   public static boolean IsTheGoalIntersectingAnyObstacles(NavigableRegion region, Point3DReadOnly start, Point3DReadOnly goal)
   {
      for (Cluster cluster : region.getObstacleClusters())
      {
         ArrayList<Point2D> list2D = new ArrayList<>();

         for (Point3D point3d : cluster.getNonNavigableExtrusionsInWorld3D())
         {
            list2D.add(new Point2D(point3d));
         }

         boolean visible = VisibilityTools.isPointVisible(new Point2D(start), new Point2D(goal), list2D);

         if (!visible)
         {
            return true;
         }
      }

      return false;
   }

   public static List<Cluster> getListOfIntersectingObstacles(List<Cluster> obstacleClusters, Point3DReadOnly start, Point3DReadOnly goal)
   {
      List<Cluster> clustersTemp = new ArrayList<Cluster>();
      for (Cluster cluster : obstacleClusters)
      {
         ArrayList<Point2D> list2D = new ArrayList<>();

         for (Point3D point3d : cluster.getNonNavigableExtrusionsInWorld3D())
         {
            list2D.add(new Point2D(point3d));
         }

         boolean visible = VisibilityTools.isPointVisible(new Point2D(start), new Point2D(goal), list2D);

         if (!visible)
         {
            clustersTemp.add(cluster);
         }
      }
      return clustersTemp;
   }
}
