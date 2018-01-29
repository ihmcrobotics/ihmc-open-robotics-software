package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.lists.ListWrappingIndexTools;

public class VisibilityTools
{
   private static final double MAGIC_NUMBER = MathTools.square(0.01);
   /**
    * Filter that reduces a little the computation time. When enabled, the tests are still passing,
    * and the path found still looks good, but the inner-region maps look slightly different. Until
    * this is fixed, I'd rather leave the filter disabled.
    */
   private static final boolean ENABLE_EXPERIMENTAL_QUICK_CHECK = false;

   public static boolean isPointVisible(Point2DReadOnly observer, Point2DReadOnly targetPoint, List<? extends Point2DReadOnly> listOfPointsInCluster)
   {
      for (int i = 0; i < listOfPointsInCluster.size() - 1; i++)
      {
         Point2DReadOnly first = listOfPointsInCluster.get(i);
         Point2DReadOnly second = listOfPointsInCluster.get(i + 1);

         if (EuclidGeometryTools.doLineSegment2DsIntersect(first, second, observer, targetPoint))
         {
            return false;
         }
      }
      return true;
   }

   public static List<Connection> getConnectionsThatAreInsideRegion(Collection<Connection> connections, PlanarRegion region)
   {
      List<Connection> filteredConnections = new ArrayList<>();

      for (Connection connection : connections)
      {

         if (PlanarRegionTools.areBothPointsInsidePlanarRegion(connection.getSourcePoint2D(), connection.getTargetPoint2D(), region))
         {
            filteredConnections.add(connection);
         }
      }

      return filteredConnections;
   }

   public static Set<Connection> createStaticVisibilityMap(List<Cluster> clusters, NavigableRegion navigableRegion)
   {
      int regionId = navigableRegion.getMapId();
      PlanarRegion homeRegion = navigableRegion.getHomeRegion();
      Set<Connection> connections = new HashSet<>();
      List<boolean[]> navigability = new ArrayList<>(clusters.size());

      for (Cluster cluster : clusters)
      {
         navigability.add(addClusterSelfVisibility(cluster, homeRegion, clusters, regionId, connections));
      }

      for (int sourceIndex = 0; sourceIndex < clusters.size(); sourceIndex++)
      {
         Cluster source = clusters.get(sourceIndex);
         boolean[] sourceNavigability = navigability.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < clusters.size(); targetIndex++)
         {
            Cluster target = clusters.get(targetIndex);
            boolean[] targetNavigability = navigability.get(targetIndex);

            addCrossClusterVisibility(source, sourceNavigability, target, targetNavigability, clusters, regionId, connections);
         }
      }

      return connections;
   }

   /**
    * Finds all the possible and valid connections using only the vertices from a single cluster,
    * i.e. {@code clusterToBuildMapOf} while considering all the clusters, including
    * {@code clusterToBuildMapOf}, for the visibility check when creating connections.
    * 
    * @param clusterToBuildMapOf the only cluster used to create new connection using its navigable
    *           extrusions. Not modified.
    * @param homeRegion the region to which the clusters belong to. Not modified.
    * @param allClusters list containing all the clusters to consider for the visibility check
    *           including {@code clusterToBuildMapOf}. Not modified.
    * @param mapId the ID used to create the connections.
    * @param connectionsToPack the collection in which the connections are stored. Modified.
    * @return an array of booleans informing on whether each individual navigable extrusion of
    *         {@code clusterToBuildMapOf} is actually navigable or not.
    */
   private static boolean[] addClusterSelfVisibility(Cluster clusterToBuildMapOf, PlanarRegion homeRegion, List<Cluster> allClusters, int mapId,
                                                     Collection<Connection> connectionsToPack)
   {
      List<Point2D> navigableExtrusions = clusterToBuildMapOf.getNavigableExtrusionsInLocal2D();

      // We first go through the extrusions and check if they are actually navigable, i.e. inside the home region and not inside any non-navigable zone.
      boolean[] areActuallyNavigable = new boolean[navigableExtrusions.size()];
      Arrays.fill(areActuallyNavigable, true);

      for (int i = 0; i < navigableExtrusions.size() - 1; i++) // <= the extrusions are actually closed by replicating the first extrusion at the end
      { // Check that the point is actually navigable
         Point2D query = navigableExtrusions.get(i);

         boolean isNavigable = PlanarRegionTools.isPointInLocalInsidePlanarRegion(homeRegion, query);

         if (isNavigable)
            isNavigable = allClusters.stream().noneMatch(cluster -> cluster.isInsideNonNavigableZone(query));

         areActuallyNavigable[i] = isNavigable;
      }

      for (int i = 0; i < navigableExtrusions.size() - 1; i++)
      { // Adding the edges of the navigable extrusion. As long as both the current and next vertices are navigable, the connection is valid.
         if (areActuallyNavigable[i] && areActuallyNavigable[i + 1])
            connectionsToPack.add(new Connection(navigableExtrusions.get(i), mapId, navigableExtrusions.get(i + 1), mapId));
      }

      Vector2D directionToCheck = new Vector2D();
      Vector2D nextEdge = new Vector2D();
      Vector2D prevEdge = new Vector2D();

      // Going through all the other possible combinations for finding connections
      for (int sourceIndex = 0; sourceIndex < navigableExtrusions.size() - 1; sourceIndex++)
      {
         if (!areActuallyNavigable[sourceIndex])
            continue; // Both source and target have to be navigable for the connection to be valid

         Point2D source = navigableExtrusions.get(sourceIndex);

         // Starting from after the next vertex of the source as we already added all the edges as connections
         for (int targetIndex = sourceIndex + 2; targetIndex < navigableExtrusions.size() - 1; targetIndex++)
         {
            if (!areActuallyNavigable[targetIndex])
               continue; // Both source and target have to be navigable for the connection to be valid

            Point2D target = navigableExtrusions.get(targetIndex);

            if (ENABLE_EXPERIMENTAL_QUICK_CHECK)
            {
               directionToCheck.sub(target, source);

               { // Perform quick check on source
                  prevEdge.sub(source, ListWrappingIndexTools.getPrevious(sourceIndex, navigableExtrusions));
                  nextEdge.sub(ListWrappingIndexTools.getNext(sourceIndex, navigableExtrusions), source);
                  if (!quickFeasibilityCheck(directionToCheck, prevEdge, nextEdge, clusterToBuildMapOf.getExtrusionSide()))
                     continue;
               }

               { // Perform quick check on target
                  directionToCheck.negate();
                  prevEdge.sub(target, ListWrappingIndexTools.getPrevious(targetIndex, navigableExtrusions));
                  nextEdge.sub(ListWrappingIndexTools.getNext(targetIndex, navigableExtrusions), target);
                  if (!quickFeasibilityCheck(directionToCheck, prevEdge, nextEdge, clusterToBuildMapOf.getExtrusionSide()))
                     continue;
               }
            }

            // Finally run the expensive test to verify if the target can be seen from the source.
            if (isPointVisibleForStaticMaps(allClusters, source, target))
               connectionsToPack.add(new Connection(source, mapId, target, mapId));
         }
      }

      return areActuallyNavigable;
   }

   /**
    * Finds all the possible and valid connections going from the navigable extrusions of
    * {@code sourceCluster} to the ones of {@code targetCluster} while considering all the clusters,
    * including {@code sourceCluster} and {@code targetCluster} when performing the visibility check
    * when creating connections.
    * 
    * @param sourceCluster the cluster which the navigable extrusions are used as source points for
    *           the connections. Not modified.
    * @param sourceNavigability the array containing the information of whether or not each
    *           individual navigable extrusion of {@code sourceCluster} is actually navigable. Not
    *           modified.
    * @param targetCluster the cluster which the navigable extrusions are used as target points for
    *           the connections. Not modified.
    * @param targetNavigability the array containing the information of whether or not each
    *           individual navigable extrusion of {@code targetCluster} is actually navigable. Not
    *           modified.
    * @param allClusters list containing all the clusters to consider for the visibility check
    *           including {@code sourceCluster} and {@code targetCluster}. Not modified.
    * @param mapId the ID used to create the connections.
    * @param connectionsToPack the collection in which the connections are stored. Modified.
    */
   private static void addCrossClusterVisibility(Cluster sourceCluster, boolean[] sourceNavigability, Cluster targetCluster, boolean[] targetNavigability,
                                                 List<Cluster> allClusters, int mapId, Collection<Connection> connectionsToPack)
   {
      Vector2D directionToCheck = new Vector2D();
      Vector2D nextEdge = new Vector2D();
      Vector2D prevEdge = new Vector2D();

      List<Point2D> sources = sourceCluster.getNavigableExtrusionsInLocal2D();
      List<Point2D> targets = targetCluster.getNavigableExtrusionsInLocal2D();

      for (int sourceIndex = 0; sourceIndex < sourceNavigability.length - 1; sourceIndex++)
      {
         if (!sourceNavigability[sourceIndex])
            continue;

         Point2D source = sources.get(sourceIndex);

         for (int targetIndex = 0; targetIndex < targetNavigability.length - 1; targetIndex++)
         {
            if (!targetNavigability[targetIndex])
               continue;

            Point2D target = targets.get(targetIndex);

            if (ENABLE_EXPERIMENTAL_QUICK_CHECK)
            {

               directionToCheck.sub(target, source);

               { // Perform quick check on source
                  prevEdge.sub(source, ListWrappingIndexTools.getPrevious(sourceIndex, sources));
                  nextEdge.sub(ListWrappingIndexTools.getNext(sourceIndex, sources), source);
                  if (!quickFeasibilityCheck(directionToCheck, prevEdge, nextEdge, sourceCluster.getExtrusionSide()))
                     continue;
               }

               { // Perform quick check on target
                  directionToCheck.negate();
                  prevEdge.sub(target, ListWrappingIndexTools.getPrevious(targetIndex, targets));
                  nextEdge.sub(ListWrappingIndexTools.getNext(targetIndex, targets), target);
                  if (!quickFeasibilityCheck(directionToCheck, prevEdge, nextEdge, targetCluster.getExtrusionSide()))
                     continue;
               }
            }

            if (isPointVisibleForStaticMaps(allClusters, source, target))
               connectionsToPack.add(new Connection(source, mapId, target, mapId));
         }
      }
   }

   public static Set<Connection> createStaticVisibilityMap(Point3DReadOnly observer, int observerRegionId, List<Cluster> clusters, int clustersRegionId)
   {
      Set<Connection> connections = new HashSet<>();
      List<Point2D> listOfTargetPoints = new ArrayList<>();
      Point2D observer2D = new Point2D(observer);

      // Add all navigable points (including dynamic objects) to a list
      for (Cluster cluster : clusters)
      {
         if (cluster.isInsideNonNavigableZone(observer2D))
            return Collections.emptySet();

         for (Point2D point : cluster.getNavigableExtrusionsInLocal2D())
         {
            listOfTargetPoints.add(point);
         }
      }

      for (int j = 0; j < listOfTargetPoints.size(); j++)
      {
         Point2D target = listOfTargetPoints.get(j);

         if (observer.distanceXYSquared(target) > MAGIC_NUMBER)
         {
            boolean targetIsVisible = isPointVisibleForStaticMaps(clusters, observer2D, target);

            if (targetIsVisible)
            {
               connections.add(new Connection(observer, observerRegionId, new Point3D(target), clustersRegionId));
            }
         }
      }

      return connections;
   }

   /**
    * The main idea of the quick check is to verifying that when attempting to connect a vertex of a
    * clockwise polygon to a target that the direction to that target goes the "proper way" with
    * respect to the next and previous edges of the source.
    * 
    * @param directionToCheck
    * @param previousEdgeDirection
    * @param nexEdgeDirection
    * @param extrusionSide
    * @return
    */
   private static boolean quickFeasibilityCheck(Vector2DReadOnly directionToCheck, Vector2DReadOnly previousEdgeDirection, Vector2DReadOnly nexEdgeDirection,
                                                ExtrusionSide extrusionSide)
   {
      double epsilon = 1.0e-8;
      double prevCrossDirection = previousEdgeDirection.cross(directionToCheck);
      if (MathTools.epsilonEquals(0.0, prevCrossDirection, epsilon))
         return true;
      double nextCrossDirection = nexEdgeDirection.cross(directionToCheck);
      if (MathTools.epsilonEquals(0.0, nextCrossDirection, epsilon))
         return true;

      if (extrusionSide == ExtrusionSide.OUTSIDE)
      {
         boolean isGoingToTheLeftOfPreviousEdge = prevCrossDirection > 0.0;
         boolean isGoingToTheLeftOfNextEdge = nextCrossDirection > 0.0;

         if (previousEdgeDirection.cross(nexEdgeDirection) < 0.0)
         { // Concave at vertex: direction has to go to the left of both previous and next edges
            return isGoingToTheLeftOfPreviousEdge && isGoingToTheLeftOfNextEdge;
         }
         else
         { // Convex at vertex: direction has to go to the left of either previous and next edges
            return isGoingToTheLeftOfPreviousEdge || isGoingToTheLeftOfNextEdge;
         }
      }
      else
      {
         boolean isGoingToTheRightOfPreviousEdge = prevCrossDirection < 0.0;
         boolean isGoingToTheRightOfNextEdge = nextCrossDirection < 0.0;

         if (previousEdgeDirection.cross(nexEdgeDirection) < 0.0)
         { // Concave at vertex: direction has to go to the right of either previous and next edges
            return isGoingToTheRightOfPreviousEdge || isGoingToTheRightOfNextEdge;
         }
         else
         { // Convex at vertex: direction has to go to the right of both previous and next edges
            return isGoingToTheRightOfPreviousEdge && isGoingToTheRightOfNextEdge;
         }
      }
   }

   public static boolean isPointVisibleForStaticMaps(List<Cluster> clusters, Point2DReadOnly observer, Point2DReadOnly targetPoint)
   {
      for (Cluster cluster : clusters)
      {
         if (cluster.getExtrusionSide() == ExtrusionSide.OUTSIDE)
         {
            BoundingBox2D boundingBox = cluster.getNonNavigableExtrusionsBoundingBox();

            // If both the target and observer are in the bounding box, we have to do the thorough check.
            if (!boundingBox.isInsideInclusive(observer) || !boundingBox.isInsideInclusive(targetPoint))
            {
               if (!boundingBox.doesIntersectWithLineSegment2D(observer, targetPoint))
                  continue;
            }
         }

         if (!VisibilityTools.isPointVisible(observer, targetPoint, cluster.getNonNavigableExtrusionsInLocal2D()))
         {
            return false;
         }
      }

      return true;
   }

   public static List<Connection> removeConnectionsFromExtrusionsOutsideRegions(Collection<Connection> connections, PlanarRegion homeRegion)
   {
      return VisibilityTools.getConnectionsThatAreInsideRegion(connections, homeRegion);
   }

   public static List<Connection> removeConnectionsFromExtrusionsInsideNoGoZones(Collection<Connection> connectionsToClean, List<Cluster> clusters)
   {
      List<Connection> validConnections = new ArrayList<>(connectionsToClean);

      for (Cluster cluster : clusters)
      {
         validConnections = getValidConnections(validConnections, cluster);
      }

      return validConnections;
   }

   public static List<Connection> getValidConnections(Collection<Connection> connections, Cluster cluster)
   {
      List<Connection> filteredConnections = new ArrayList<>();

      for (Connection connection : connections)
      {
         Point2D source = connection.getSourcePoint2D();
         Point2D target = connection.getTargetPoint2D();

         if (!cluster.isInsideNonNavigableZone(source) && !cluster.isInsideNonNavigableZone(target))
            filteredConnections.add(connection);
      }

      return filteredConnections;
   }
}
