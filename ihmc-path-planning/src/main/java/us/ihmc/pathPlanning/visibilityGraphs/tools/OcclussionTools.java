package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;

public class OcclussionTools
{
   public static boolean IsTheGoalIntersectingAnyObstacles(NavigableRegion region, Point3D start, Point3D goal)
   {
      System.out.println("Looking for a straighline path to goal");
      System.out.println("Clusters size: " + region.getClusters().size());
      if (region.getClusters().size() > 1) //1 cluster means outside boundary which is not an obstacle...
      {
         for (Cluster cluster : region.getClusters())
         {
            if(!cluster.isHomeRegion())
            {
               ArrayList<Point2D> list2D = new ArrayList<>();

               for (Point3D point3d : cluster.getNonNavigableExtrusionsInWorld())
               {
                  list2D.add(new Point2D(point3d.getX(), point3d.getY()));
               }

               System.out.println(start + "   " + goal + "   " + list2D.size());
               boolean visible = VisibilityTools.isPointVisible(new Point2D(start.getX(), start.getY()), new Point2D(goal.getX(), goal.getY()), list2D);

               if (!visible)
               {
                  System.out.println("Goal is not visible!!!!!!!!!!!!!");
                  return true;
               }
               else
               {
                  System.out.println("Goal is visible!!!!!!!!!!!!!");
               }
            }
         }
      }
      
      return false;
   }
}
