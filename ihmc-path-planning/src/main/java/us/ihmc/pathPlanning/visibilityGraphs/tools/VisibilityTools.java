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
import us.ihmc.commons.lists.ListWrappingIndexTools;

public class VisibilityTools
{
   private static final double MAGIC_NUMBER = MathTools.square(0.01);
   /**
    * Filter that reduces a little the computation time. When enabled, the tests are still passing,
    * and the path found still looks good, but the inner-region maps look slightly different. Until
    * this is fixed, I'd rather leave the filter disabled.
    */
   private static final boolean ENABLE_EXPERIMENTAL_QUICK_CHECK = false;

   
   // TODO: +++JEP: (Remove) Some hackish booleans to reduce expansion to O(n) by not doing all n*n combinations.
   private static final boolean JUST_DO_AROUND_RING_INSTEAD_OF_ALL_CONNECTIONS = false;
   private static final int INNER_CONNECTIONS_EVERY_N_POINTS = 1;
   private static final boolean RETURN_AFTER_FINDING_A_SINGLE_CONNECTION = false;
   private static final boolean CONTINUE_AFTER_FINDING_A_SINGLE_CONNECTION = false;
   

   public static boolean isPointVisible(Point2DReadOnly observer, Point2DReadOnly targetPoint, List<? extends Point2DReadOnly> listOfPointsInCluster)
   {
      //TODO: +++JEP: Need to check the closing point if it is a polygon!! Also, add test cases for that. Also need to make sure one polygon per cluster...
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

   public static Set<Connection> createStaticVisibilityMap(NavigableRegion navigableRegion)
   {
      int regionId = navigableRegion.getMapId();
      PlanarRegion homeRegion = navigableRegion.getHomePlanarRegion();
      
      Cluster homeRegionCluster = navigableRegion.getHomeRegionCluster();
      List<Cluster> allClusters = navigableRegion.getAllClusters();

      Set<Connection> connections = new HashSet<>();
      List<boolean[]> navigability = new ArrayList<>(allClusters.size());

      for (Cluster cluster : allClusters)
      {
         if (cluster == homeRegionCluster)
         {
            if (JUST_DO_AROUND_RING_INSTEAD_OF_ALL_CONNECTIONS)
               navigability.add(addClusterSelfVisibilityAroundRing(cluster, homeRegion, allClusters, regionId, connections));  
            else
               navigability.add(addClusterSelfVisibility(cluster, homeRegion, allClusters, regionId, connections));
         }
         else
         {
            navigability.add(addClusterSelfVisibilityAroundRing(cluster, homeRegion, allClusters, regionId, connections));            
         }
      }

      for (int sourceIndex = 0; sourceIndex < allClusters.size(); sourceIndex++)
      {
         Cluster source = allClusters.get(sourceIndex);
         boolean[] sourceNavigability = navigability.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < allClusters.size(); targetIndex++)
         {
            Cluster target = allClusters.get(targetIndex);
            boolean[] targetNavigability = navigability.get(targetIndex);

            addCrossClusterVisibility(source, sourceNavigability, target, targetNavigability, allClusters, regionId, connections);
         }
      }

      return connections;
   }

   private static boolean isNotInsideANonNavigableZone(Point2DReadOnly query, List<Cluster> clusters)
   {
      return clusters.stream().noneMatch(cluster -> cluster.isInsideNonNavigableZone(query));
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
   public static boolean[] addClusterSelfVisibility(Cluster clusterToBuildMapOf, PlanarRegion homeRegion, List<Cluster> allClusters, int mapId,
                                                    Collection<Connection> connectionsToPack)
   {
      List<? extends Point2DReadOnly> navigableExtrusionPoints = clusterToBuildMapOf.getNavigableExtrusionsInLocal();

      boolean[] arePointsActuallyNavigable = checkIfPointsInsidePlanarRegionAndOutsideNonavigableZones(homeRegion, allClusters, navigableExtrusionPoints);

      Vector2D directionToCheck = new Vector2D();
      Vector2D nextEdge = new Vector2D();
      Vector2D prevEdge = new Vector2D();

      // Going through all of the possible combinations of two points for finding connections
      for (int sourceIndex = 0; sourceIndex < navigableExtrusionPoints.size(); sourceIndex++)
      {
         if (!arePointsActuallyNavigable[sourceIndex])
            continue; // Both source and target have to be navigable for the connection to be valid

         Point2DReadOnly source = navigableExtrusionPoints.get(sourceIndex);

         // Starting from after the next vertex of the source as we already added all the edges as connections
         for (int targetIndex = sourceIndex + 1; targetIndex < navigableExtrusionPoints.size(); targetIndex=targetIndex + INNER_CONNECTIONS_EVERY_N_POINTS)
         {
            if (!arePointsActuallyNavigable[targetIndex])
               continue; // Both source and target have to be navigable for the connection to be valid

            Point2DReadOnly target = navigableExtrusionPoints.get(targetIndex);

            if (ENABLE_EXPERIMENTAL_QUICK_CHECK)
            {
               directionToCheck.sub(target, source);

               { // Perform quick check on source
                  prevEdge.sub(source, ListWrappingIndexTools.getPrevious(sourceIndex, navigableExtrusionPoints));
                  nextEdge.sub(ListWrappingIndexTools.getNext(sourceIndex, navigableExtrusionPoints), source);
                  if (!quickFeasibilityCheck(directionToCheck, prevEdge, nextEdge, clusterToBuildMapOf.getExtrusionSide()))
                     continue;
               }

               { // Perform quick check on target
                  directionToCheck.negate();
                  prevEdge.sub(target, ListWrappingIndexTools.getPrevious(targetIndex, navigableExtrusionPoints));
                  nextEdge.sub(ListWrappingIndexTools.getNext(targetIndex, navigableExtrusionPoints), target);
                  if (!quickFeasibilityCheck(directionToCheck, prevEdge, nextEdge, clusterToBuildMapOf.getExtrusionSide()))
                     continue;
               }
            }

            // Finally run the expensive test to verify if the target can be seen from the source.
            if (isPointVisibleForStaticMaps(allClusters, source, target))
               connectionsToPack.add(new Connection(source, mapId, target, mapId));
         }
      }

      return arePointsActuallyNavigable;
   }
   
   public static boolean[] addClusterSelfVisibilityAroundRing(Cluster clusterToBuildMapOf, PlanarRegion homeRegion, List<Cluster> allClusters, int mapId,
                                                    Collection<Connection> connectionsToPack)
   {
      List<? extends Point2DReadOnly> navigableExtrusionPoints = clusterToBuildMapOf.getNavigableExtrusionsInLocal();
      boolean[] arePointsActuallyNavigable = checkIfPointsInsidePlanarRegionAndOutsideNonavigableZones(homeRegion, allClusters, navigableExtrusionPoints);

      // Going through all of the possible combinations of two points for finding connections
      for (int sourceIndex = 0; sourceIndex < navigableExtrusionPoints.size(); sourceIndex++)
      {
         if (!arePointsActuallyNavigable[sourceIndex])
            continue; // Both source and target have to be navigable for the connection to be valid

         Point2DReadOnly source = navigableExtrusionPoints.get(sourceIndex);

         // Starting from after the next vertex of the source as we already added all the edges as connections
         int targetIndex = (sourceIndex + 1) % navigableExtrusionPoints.size();
         {
            if (!arePointsActuallyNavigable[targetIndex])
               continue; // Both source and target have to be navigable for the connection to be valid

            Point2DReadOnly target = navigableExtrusionPoints.get(targetIndex);

            // Finally run the expensive test to verify if the target can be seen from the source.
            if (isPointVisibleForStaticMaps(allClusters, source, target))
               connectionsToPack.add(new Connection(source, mapId, target, mapId));
         }
      }

      return arePointsActuallyNavigable;
   }

   private static boolean[] checkIfPointsInsidePlanarRegionAndOutsideNonavigableZones(PlanarRegion homeRegion, List<Cluster> allClusters,
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
   public static void addCrossClusterVisibility(Cluster sourceCluster, boolean[] sourceNavigability, Cluster targetCluster, boolean[] targetNavigability,
                                                List<Cluster> allClusters, int mapId, Collection<Connection> connectionsToPack)
   {
      Vector2D directionToCheck = new Vector2D();
      Vector2D nextEdge = new Vector2D();
      Vector2D prevEdge = new Vector2D();

      List<? extends Point2DReadOnly> sources = sourceCluster.getNavigableExtrusionsInLocal();
      List<? extends Point2DReadOnly> targets = targetCluster.getNavigableExtrusionsInLocal();

      sourceLoop:
      for (int sourceIndex = 0; sourceIndex < sourceNavigability.length; sourceIndex++)
      {
         if (!sourceNavigability[sourceIndex])
            continue;

         Point2DReadOnly source = sources.get(sourceIndex);

         for (int targetIndex = 0; targetIndex < targetNavigability.length; targetIndex++)
         {
            if (!targetNavigability[targetIndex])
               continue;

            Point2DReadOnly target = targets.get(targetIndex);

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
            {
               connectionsToPack.add(new Connection(source, mapId, target, mapId));
               if (RETURN_AFTER_FINDING_A_SINGLE_CONNECTION)
                  return;
               if (CONTINUE_AFTER_FINDING_A_SINGLE_CONNECTION)
                  continue sourceLoop;
            }
         }
      }
   }

   public static Set<Connection> createStaticVisibilityMap(Point3DReadOnly observer, int observerRegionId, List<Cluster> clusters, int clustersRegionId)
   {
      Set<Connection> connections = new HashSet<>();
      List<Point2DReadOnly> listOfTargetPoints = new ArrayList<>();
      Point2D observer2D = new Point2D(observer);

      // Add all navigable points (including dynamic objects) to a list
      for (Cluster cluster : clusters)
      {
         if (cluster.isInsideNonNavigableZone(observer2D))
            return Collections.emptySet();

         for (Point2DReadOnly point : cluster.getNavigableExtrusionsInLocal())
         {
            listOfTargetPoints.add(point);
         }
      }

      for (int j = 0; j < listOfTargetPoints.size(); j++)
      {
         Point2DReadOnly target = listOfTargetPoints.get(j);

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

   //TODO: +++JEP: Get rid of these checks after no longer needed for optimizing. 
   private static int numberIsPointVisibleChecks = 0, edgeChecks = 0, ruledOutByBoundingBox = 0, notVisible = 0, visiblePoints = 0;
   
   public static boolean isPointVisibleForStaticMaps(List<Cluster> clusters, Point2DReadOnly observer, Point2DReadOnly targetPoint)
   {
      numberIsPointVisibleChecks++;
      if (numberIsPointVisibleChecks % 1000000 == 0) printStats();

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
                  ruledOutByBoundingBox++;
                  continue;
               }
            }
         }

         edgeChecks++;
         
         //TODO: +++JEP: Lots of time taken here. 2,379,800 calls for 4.7 sec.
         if (!VisibilityTools.isPointVisible(observer, targetPoint, cluster.getNonNavigableExtrusionsInLocal()))
         {
            notVisible++;
            return false;
         }
      }

      visiblePoints++;
      
      return true;
   }

   private static void printStats()
   {
      System.out.println("numberIsPointVisibleChecks = " + numberIsPointVisibleChecks);
      System.out.println("ruledOutByBoundingBox = " + ruledOutByBoundingBox);
      System.out.println("edgeChecks = " + edgeChecks);
      System.out.println("notVisible = " + notVisible);
      System.out.println("visiblePoints = " + visiblePoints);
      System.out.println("");
   }

   public static List<Connection> removeConnectionsFromExtrusionsOutsideRegions(Collection<Connection> connections, PlanarRegion homeRegion)
   {
      return VisibilityTools.getConnectionsThatAreInsideRegion(connections, homeRegion);
   }

   public static List<Connection> removeConnectionsFromExtrusionsInsideNoGoZones(Collection<Connection> connectionsToClean, List<Cluster> clusters)
   {
      //TODO: +++JEP: This seems buggy to me. It doesn't seem to do the 2D / 3D thing correctly. Should be easy to make a failing test case I think... Or remove if we don't need it.
      List<Connection> validConnections = new ArrayList<>(connectionsToClean);

      for (Cluster cluster : clusters)
      {
         validConnections = getValidConnections(validConnections, cluster);
      }

      return validConnections;
   }

   private static List<Connection> getValidConnections(Collection<Connection> connections, Cluster cluster)
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
