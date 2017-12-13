package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;

public class OcclussionTools
{
   public static boolean IsTheGoalIntersectingAnyObstacles(NavigableRegion region, Point3D start, Point3D goal)
   {
      for (Cluster cluster : region.getAllClusters())
      {
         if (!cluster.isHomeRegion())
         {
            ArrayList<Point2D> list2D = new ArrayList<>();

            for (Point3D point3d : cluster.getNonNavigableExtrusionsInWorld())
            {
               list2D.add(new Point2D(point3d.getX(), point3d.getY()));
            }

            boolean visible = VisibilityTools.isPointVisible(new Point2D(start.getX(), start.getY()), new Point2D(goal.getX(), goal.getY()), list2D);

            if (!visible)
            {
               return true;
            }
            else
            {
            }
         }
      }

      return false;
   }

   public static List<Cluster> getListOfIntersectingObstacles(List<Cluster> clusters, Point3D start, Point3D goal)
   {
      List<Cluster> clustersTemp = new ArrayList<Cluster>();
      for (Cluster cluster : clusters)
      {
         if (!cluster.isHomeRegion())
         {
            ArrayList<Point2D> list2D = new ArrayList<>();

            for (Point3D point3d : cluster.getNonNavigableExtrusionsInWorld())
            {
               list2D.add(new Point2D(point3d.getX(), point3d.getY()));
            }

            boolean visible = VisibilityTools.isPointVisible(new Point2D(start.getX(), start.getY()), new Point2D(goal.getX(), goal.getY()), list2D);

            if (!visible)
            {
               clustersTemp.add(cluster);
            }
         }
      }
      return clustersTemp;
   }
}
