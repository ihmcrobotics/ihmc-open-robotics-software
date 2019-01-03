package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityTools
{
   public static boolean isPointVisible(Point2DReadOnly observer, Point2DReadOnly targetPoint, List<? extends Point2DReadOnly> listOfPointsInCluster,
                                        boolean closed)
   {
      int size = listOfPointsInCluster.size();
      int endIndex = size - 1;
      if (closed)
         endIndex++;

      for (int i = 0; i < endIndex; i++)
      {
         Point2DReadOnly first = listOfPointsInCluster.get(i);

         int nextIndex = i + 1;
         if (nextIndex == size)
            nextIndex = 0;

         Point2DReadOnly second = listOfPointsInCluster.get(nextIndex);

         if (EuclidGeometryTools.doLineSegment2DsIntersect(first, second, observer, targetPoint))
         {
            return false;
         }
      }
      return true;
   }

   private static boolean isNotInsideANonNavigableZone(Point2DReadOnly query, List<Cluster> clusters)
   {
      return clusters.stream().noneMatch(cluster -> cluster.isInsideNonNavigableZone(query));
   }

   public static boolean[] checkIfPointsInsidePlanarRegionAndOutsideNonNavigableZones(PlanarRegion homeRegion, List<Cluster> allClusters,
                                                                                      List<? extends Point2DReadOnly> navigableExtrusionPoints)
   {
      // We first go through the extrusions and check if they are actually navigable, i.e. inside the home region and not inside any non-navigable zone.
      boolean[] arePointsActuallyNavigable = new boolean[navigableExtrusionPoints.size()];
      Arrays.fill(arePointsActuallyNavigable, true);

      for (int i = 0; i < navigableExtrusionPoints.size(); i++)
      {
         // Check that the point is actually navigable
         Point2DReadOnly query = navigableExtrusionPoints.get(i);

         boolean isNavigable = PlanarRegionTools.isPointInLocalInsidePlanarRegion(homeRegion, query);

         if (isNavigable)
         {
            isNavigable = isNotInsideANonNavigableZone(query, allClusters);
         }

         arePointsActuallyNavigable[i] = isNavigable;
      }
      return arePointsActuallyNavigable;
   }

   //TODO: Rename.
   public static boolean isPointVisibleForStaticMaps(List<Cluster> clusters, Point2DReadOnly observer, Point2DReadOnly targetPoint)
   {
      for (Cluster cluster : clusters)
      {
         if (cluster.getExtrusionSide() == ExtrusionSide.OUTSIDE)
         {
            BoundingBox2D boundingBox = cluster.getNonNavigableExtrusionsBoundingBox();

            // If either the target or observer or both are in the bounding box, we have to do the thorough check.
            // If both are outside the bounding box, then we can check if the line segment does not intersect.
            // If that is the case, then the point is visible and we can check the next one.
            if (!boundingBox.isInsideInclusive(observer) && !boundingBox.isInsideInclusive(targetPoint))
            {
               if (!boundingBox.doesIntersectWithLineSegment2D(observer, targetPoint))
               {
                  continue;
               }
            }
         }

         boolean closed = cluster.isClosed();
         if (!VisibilityTools.isPointVisible(observer, targetPoint, cluster.getNonNavigableExtrusionsInLocal(), closed))
         {
            return false;
         }
      }

      return true;
   }
}
