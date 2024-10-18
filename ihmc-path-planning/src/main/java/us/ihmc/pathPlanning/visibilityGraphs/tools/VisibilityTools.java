package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.List;

import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryMissingTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.InterRegionConnectionFilter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.*;

public class VisibilityTools
{
   public static boolean isPointVisible(Point2DReadOnly observer, Point2DReadOnly targetPoint, ExtrusionHull points, boolean closed)
   {
      return isPointVisible(observer, targetPoint, points.getPoints(), closed);
   }

   public static boolean isPointVisible(Point2DReadOnly observer, Point2DReadOnly targetPoint, List<? extends Point2DReadOnly> listOfPointsInCluster, boolean closed)
   {
      int size = listOfPointsInCluster.size();
      int endIndex = size - 1;
      if (closed)
         endIndex++;

      for (int i = 0; i < endIndex; i++)
      {
         Point2DReadOnly first = listOfPointsInCluster.get(i);
         Point2DReadOnly second = ListWrappingIndexTools.getNext(i, listOfPointsInCluster);

         if (EuclidGeometryTools.doLineSegment2DsIntersect(first, second, observer, targetPoint))
         {
            return false;
         }
      }
      return true;
   }

   public static boolean isPointVisibleInclusive(Point2DReadOnly observer, Point2DReadOnly targetPoint, List<Point2DReadOnly> listOfPointsInCluster,
                                                 boolean closed)
   {
      int size = listOfPointsInCluster.size();
      int endIndex = size - 1;
      if (closed)
         endIndex++;

      for (int i = 0; i < endIndex; i++)
      {
         Point2DReadOnly first = listOfPointsInCluster.get(i);
         Point2DReadOnly second = ListWrappingIndexTools.getNext(i, listOfPointsInCluster);

         // this will return true if they share a point, which isn't always a good thing
         boolean sharesAPoint = first.epsilonEquals(targetPoint, 1.0e-10);
         sharesAPoint |= first.epsilonEquals(observer, 1.0e-10);
         sharesAPoint |= second.epsilonEquals(targetPoint, 1.0e-10);
         sharesAPoint |= second.epsilonEquals(observer, 1.0e-10);

         if (!sharesAPoint && EuclidGeometryTools.doLineSegment2DsIntersect(first, second, observer, targetPoint))
         {
            return false;
         }
      }
      return true;
   }

   public static double distanceToCluster(Point2DReadOnly firstPointOfLine, Point2DReadOnly secondPointOfLine, ExtrusionHull extrusion,
                                          Point2DBasics closestPointOnLineToPack, Point2DBasics closestPointOnClusterToPack,
                                          Vector2DBasics normalToClusterToPack, boolean closed)
   {
      return distanceToCluster(firstPointOfLine, secondPointOfLine, extrusion.getPoints(), closestPointOnLineToPack, closestPointOnClusterToPack,
                               normalToClusterToPack, closed);
   }

   public static double distanceToCluster(Point2DReadOnly firstPointOfLine, Point2DReadOnly secondPointOfLine, List<Point2DReadOnly> listOfPointsInCluster,
                                          Point2DBasics closestPointOnLineToPack, Point2DBasics closestPointOnClusterToPack,
                                          Vector2DBasics normalToClusterToPack, boolean closed)
   {
      int numberOfVertices = listOfPointsInCluster.size();

      if (numberOfVertices == 0)
      {
         if (closestPointOnLineToPack != null)
            closestPointOnLineToPack.setToNaN();
         if (closestPointOnClusterToPack != null)
            closestPointOnClusterToPack.setToNaN();
         if (normalToClusterToPack != null)
            normalToClusterToPack.setToNaN();
         return Double.NaN;
      }

      if (numberOfVertices == 1)
      {
         Point2DReadOnly clusterPoint = listOfPointsInCluster.get(0);
         orthogonalProjectionOnLineSegment2D(clusterPoint, firstPointOfLine, secondPointOfLine, closestPointOnLineToPack);
         if (normalToClusterToPack != null)
            normalToClusterToPack.setToZero();
         if (closestPointOnClusterToPack != null)
            closestPointOnClusterToPack.set(clusterPoint);
         return clusterPoint.distance(closestPointOnLineToPack);
      }

      if (numberOfVertices == 2)
      {
         if (normalToClusterToPack != null)
         {
            normalToClusterToPack.sub(listOfPointsInCluster.get(1), listOfPointsInCluster.get(0));
            EuclidGeometryTools.perpendicularVector2D(normalToClusterToPack, normalToClusterToPack);
            normalToClusterToPack.normalize();
         }

         return EuclidGeometryMissingTools
               .closestPoint2DsBetweenTwoLineSegment2Ds(firstPointOfLine, secondPointOfLine, listOfPointsInCluster.get(0), listOfPointsInCluster.get(1),
                                                        closestPointOnLineToPack, closestPointOnClusterToPack);
      }

      boolean pointIsVisible = isPointVisible(firstPointOfLine, secondPointOfLine, listOfPointsInCluster, closed);

      double minDistance = Double.POSITIVE_INFINITY;

      Point2DBasics closestPointOnLine = new Point2D();
      Point2DBasics closestPointOnCluster = new Point2D();

      for (int index = 0; index < numberOfVertices; index++)
      {
         Point2DReadOnly edgeStart = listOfPointsInCluster.get(index);
         Point2DReadOnly edgeEnd = listOfPointsInCluster.get(EuclidGeometryPolygonTools.next(index, numberOfVertices));

         double distance = EuclidGeometryMissingTools
               .closestPoint2DsBetweenTwoLineSegment2Ds(firstPointOfLine, secondPointOfLine, edgeStart, edgeEnd, closestPointOnLine,
                                                        closestPointOnCluster);
         if (distance < minDistance)
         {
            minDistance = distance;
            if (closestPointOnLineToPack != null)
               closestPointOnLineToPack.set(closestPointOnLine);
            if (closestPointOnClusterToPack != null)
               closestPointOnClusterToPack.set(closestPointOnCluster);

            if (normalToClusterToPack != null)
            {
               normalToClusterToPack.sub(edgeEnd, edgeStart);
               EuclidGeometryTools.perpendicularVector2D(normalToClusterToPack, normalToClusterToPack);
               normalToClusterToPack.normalize();
            }
         }
      }

      if (!pointIsVisible)
         minDistance = -minDistance;
      return minDistance;
   }

   public static double distanceToCluster(Point2DReadOnly point, ExtrusionHull extrusion, Point2DBasics closestPointInCluster, Vector2DBasics normalToCluster)
   {
      return distanceToCluster(point, extrusion.getPoints(), closestPointInCluster, normalToCluster);
   }

   public static double distanceToCluster(Point2DReadOnly point, List<Point2DReadOnly> listOfPointsInCluster, Point2DBasics closestPointInClusterToPack,
                                          Vector2DBasics normalToClusterToPack)
   {
      int numberOfVertices = listOfPointsInCluster.size();

      if (numberOfVertices == 0)
      {
         closestPointInClusterToPack.setToNaN();
         if (normalToClusterToPack != null)
            normalToClusterToPack.setToNaN();
         return Double.NaN;
      }

      if (numberOfVertices == 1)
      {
         closestPointInClusterToPack.set(listOfPointsInCluster.get(0));
         if (normalToClusterToPack != null)
            normalToClusterToPack.setToZero();
         return distanceBetweenPoint2Ds(point.getX(), point.getY(), listOfPointsInCluster.get(0));
      }

      if (numberOfVertices == 2)
      {
         orthogonalProjectionOnLineSegment2D(point, listOfPointsInCluster.get(0), listOfPointsInCluster.get(1), closestPointInClusterToPack);

         if (normalToClusterToPack != null)
         {
            normalToClusterToPack.sub(listOfPointsInCluster.get(1), listOfPointsInCluster.get(0));
            EuclidGeometryTools.perpendicularVector2D(normalToClusterToPack, normalToClusterToPack);
            normalToClusterToPack.normalize();
         }

         return point.distance(closestPointInClusterToPack);
      }

      Point2DBasics tempPoint = new Point2D();
      boolean isQueryOutsidePolygon = false;
      double minDistance = Double.POSITIVE_INFINITY;

      for (int index = 0; index < numberOfVertices; index++)
      {
         Point2DReadOnly edgeStart = listOfPointsInCluster.get(index);
         Point2DReadOnly edgeEnd = listOfPointsInCluster.get(EuclidGeometryPolygonTools.next(index, numberOfVertices));

         isQueryOutsidePolygon |= isPoint2DOnSideOfLine2D(point.getX(), point.getY(), edgeStart, edgeEnd, true);

         orthogonalProjectionOnLineSegment2D(point, edgeStart, edgeEnd, tempPoint);
         double distance = point.distance(tempPoint);

         if (distance < minDistance)
         {
            minDistance = distance;
            closestPointInClusterToPack.set(tempPoint);

            if (normalToClusterToPack != null)
            {
               normalToClusterToPack.sub(edgeEnd, edgeStart);
               EuclidGeometryTools.perpendicularVector2D(normalToClusterToPack, normalToClusterToPack);
               normalToClusterToPack.normalize();
            }
         }
      }

      if (!isQueryOutsidePolygon)
         minDistance = -minDistance;
      return minDistance;
   }

   public static boolean checkIfPointIsInRegionAndOutsideNonNavigableZone(Point2DReadOnly query, PlanarRegion homeRegion, List<Cluster> allClusters)
   {
      // Check that the point is actually navigable
      boolean isNavigable = PlanarRegionTools.isPointInLocalInsidePlanarRegion(homeRegion, query);

      if (isNavigable)
      {
         isNavigable = allClusters.stream().noneMatch(cluster -> cluster.isInsideNonNavigableZone(query));
      }

      return isNavigable;
   }

   public static boolean isInnerRegionEdgeValid(VisibilityGraphNode sourceNode, VisibilityGraphNode targetNode)
   {
      List<Cluster> allClusters = sourceNode.getVisibilityGraphNavigableRegion().getNavigableRegion().getAllClusters();

      Point2DReadOnly sourceNodeInLocal = sourceNode.getPoint2DInLocal();
      Point2DReadOnly targetNodeInLocal = targetNode.getPoint2DInLocal();

      return isPointVisibleToPointInSameRegion(allClusters, sourceNodeInLocal, targetNodeInLocal);
   }

   public static boolean isPointVisibleToPointInSameRegion(List<Cluster> clusters, Point2DReadOnly observer, Point2DReadOnly targetPoint)
   {
      for (Cluster cluster : clusters)
      {
         boolean closed = cluster.isClosed();

         // if it's an outer extrusion, it's an obstacle. This means that you cannot pass through it.
         boolean isAnOuterExtrusion = cluster.getExtrusionSide() == ExtrusionSide.OUTSIDE;
         if (isAnOuterExtrusion)
         {
            BoundingBox2DReadOnly outerMostBoundingBoxToCheck = cluster.getNonNavigableExtrusionsBoundingBox();

            // If either the target or observer or both are in the bounding box, we have to check the interior bounding box.
            // If both are outside the bounding box, then we can check if the line segment does not intersect.
            // If that is the case, then the point is visible and we can check the next one.
            if (!outerMostBoundingBoxToCheck.isInsideInclusive(observer) && !outerMostBoundingBoxToCheck.isInsideInclusive(targetPoint))
            {
               if (!outerMostBoundingBoxToCheck.doesIntersectWithLineSegment2D(observer, targetPoint))
               {
                  continue;
               }
            }
         }

         if (!VisibilityTools.isPointVisible(observer, targetPoint, cluster.getNonNavigableExtrusionsInLocal(), closed))
            return false;
      }

      return true;
   }

   public static boolean isInterRegionEdgeValid(VisibilityGraphNode sourceNode, VisibilityGraphNode targetNode, List<Cluster> sourceObstacleClusters,
                                                List<PlanarRegion> sourceObstacleRegions, List<Cluster> targetObstacleClusters,
                                                List<PlanarRegion> targetObstacleRegions, InterRegionConnectionFilter filter,
                                                double lengthForLongInterRegionEdgeSquared)
   {

      ConnectionPoint3D sourceInWorld = sourceNode.getPointInWorld();

      ConnectionPoint3D targetInWorld = targetNode.getPointInWorld();
      if (!filter.isConnectionValid(sourceInWorld, targetInWorld))
      {
         return false;
      }

      PlanarRegion sourceHomeRegion = sourceNode.getVisibilityGraphNavigableRegion().getNavigableRegion().getHomePlanarRegion();
      RigidBodyTransformReadOnly transformFromWorldToSource = sourceHomeRegion.getTransformToLocal();
      Point2DReadOnly sourceInSourceLocal = sourceNode.getPoint2DInLocal();

      //TODO: +++++++JerryPratt: xyDistance check is a hack to allow connections through keep out regions enough to make them, but not enough to go through walls...
      double xyDistanceSquared = sourceInWorld.distanceXYSquared(targetInWorld);
      if (xyDistanceSquared < lengthForLongInterRegionEdgeSquared)
      {
         return true;
      }

      // Check if the edge is visible if it is a long one.
      PlanarRegion targetHomeRegion = targetNode.getVisibilityGraphNavigableRegion().getNavigableRegion().getHomePlanarRegion();
      RigidBodyTransformReadOnly transformFromWorldToTarget = targetHomeRegion.getTransformToLocal();

      Point2DReadOnly targetInTargetLocal = targetNode.getPoint2DInLocal();

      Point3D targetProjectedVerticallyOntoSource = PlanarRegionTools.projectInZToPlanarRegion(targetInWorld, sourceHomeRegion);
      Point3D sourceProjectedVerticallyOntoTarget = PlanarRegionTools.projectInZToPlanarRegion(sourceInWorld, targetHomeRegion);

      transformFromWorldToSource.transform(targetProjectedVerticallyOntoSource);
      transformFromWorldToTarget.transform(sourceProjectedVerticallyOntoTarget);

      Point2D targetInSourceLocal = new Point2D(targetProjectedVerticallyOntoSource);
      Point2D sourceInTargetLocal = new Point2D(sourceProjectedVerticallyOntoTarget);

      //TODO: +++JerryPratt: Inter-region connections and obstacles still needs some thought and some good unit tests.
      boolean targetIsVisibleThroughSourceObstacles = VisibilityTools.isPointVisibleToPointInOtherRegion(sourceObstacleClusters, sourceObstacleRegions,
                                                                                                         sourceInSourceLocal, targetHomeRegion,
                                                                                                         targetInSourceLocal);
      boolean sourceIsVisibleThroughTargetObstacles = VisibilityTools.isPointVisibleToPointInOtherRegion(targetObstacleClusters, targetObstacleRegions,
                                                                                                         targetInTargetLocal,  sourceHomeRegion,
                                                                                                         sourceInTargetLocal);


      return targetIsVisibleThroughSourceObstacles && sourceIsVisibleThroughTargetObstacles;
   }

   public static boolean isPointVisibleToPointInOtherRegion(List<Cluster> observerObstacleClusters, List<PlanarRegion> observerObstacleRegions,
                                                            Point2DReadOnly observer, PlanarRegion targetRegion, Point2DReadOnly targetPoint)
   {
      for (int i = 0; i < observerObstacleClusters.size(); i++)
      {
         if (observerObstacleRegions.get(i) == targetRegion)
            continue;

         Cluster cluster = observerObstacleClusters.get(i);

         boolean closed = cluster.isClosed();

         // if it's an outer extrusion, it's an obstacle. This means that you cannot pass through it.
         boolean isAnOuterExtrusion = cluster.getExtrusionSide() == ExtrusionSide.OUTSIDE;
         if (isAnOuterExtrusion)
         {
            BoundingBox2DReadOnly outerMostBoundingBoxToCheck = cluster.getNonNavigableExtrusionsBoundingBox();

            // If either the target or observer or both are in the bounding box, we have to check the interior bounding box.
            // If both are outside the bounding box, then we can check if the line segment does not intersect.
            // If that is the case, then the point is visible and we can check the next one.
            if (!outerMostBoundingBoxToCheck.isInsideInclusive(observer) && !outerMostBoundingBoxToCheck.isInsideInclusive(targetPoint))
            {
               if (!outerMostBoundingBoxToCheck.doesIntersectWithLineSegment2D(observer, targetPoint))
               {
                  continue;
               }
            }
         }

         else if (!VisibilityTools.isPointVisible(observer, targetPoint, cluster.getNonNavigableExtrusionsInLocal(), closed))
            return false;
      }

      return true;
   }
}
