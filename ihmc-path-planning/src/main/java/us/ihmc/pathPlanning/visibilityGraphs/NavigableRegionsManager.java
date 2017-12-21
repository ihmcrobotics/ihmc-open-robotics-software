package us.ihmc.pathPlanning.visibilityGraphs;

import static us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools.isPointVisibleForStaticMaps;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.jgrapht.Graph;
import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.InterRegionConnectionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.NavigableRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.OcclussionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class NavigableRegionsManager
{
   private final static boolean debug = false;
   private final static boolean TRUNCATE_OBSTACLE_REGIONS = true;

   private final static int START_GOAL_ID = 0;

   private List<PlanarRegion> regions;
   private SingleSourceVisibilityMap startMap, goalMap;
   private List<NavigableRegion> navigableRegions;

   private final VisibilityGraphsParameters parameters;

   private List<Connection> interRegionConnections;

   public NavigableRegionsManager()
   {
      this(null, null);
   }

   public NavigableRegionsManager(VisibilityGraphsParameters parameters)
   {
      this(parameters, null);
   }

   public NavigableRegionsManager(List<PlanarRegion> regions)
   {
      this(null, regions);
   }

   public NavigableRegionsManager(VisibilityGraphsParameters parameters, List<PlanarRegion> regions)
   {
      this.parameters = parameters == null ? new DefaultVisibilityGraphParameters() : parameters;
      setPlanarRegions(regions);
   }

   public void setPlanarRegions(List<PlanarRegion> regions)
   {
      if (regions != null)
      {
         regions = PlanarRegionTools.ensureClockwiseOrder(regions);
         regions = PlanarRegionTools.filterPlanarRegionsByArea(parameters.getPlanarRegionMinArea(), regions);
         regions = PlanarRegionTools.filterPlanarRegionsByHullSize(parameters.getPlanarRegionMinSize(), regions);
      }

      this.regions = regions;
   }

   public List<Point3DReadOnly> calculateBodyPath(final Point3DReadOnly start, final Point3DReadOnly goal)
   {
      if (start == null)
      {
         throw new RuntimeException("Start is null!.");
      }

      if (goal == null)
      {
         throw new RuntimeException("Goal is null!.");
      }

      if (debug)
         PrintTools.info("Starting to calculate body path");

      regions = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(start, goal, parameters.getExplorationDistanceFromStartGoal(), regions);

      long startBodyPathComputation = System.currentTimeMillis();
      long startCreatingMaps = System.currentTimeMillis();

      NavigableRegionFilter navigableRegionFilter = parameters.getNavigableRegionFilter();
      navigableRegions = regions.stream().filter(navigableRegionFilter::isPlanarRegionNavigable).map(this::createVisibilityGraphForRegion)
                                .collect(Collectors.toList());

      long endCreationTime = System.currentTimeMillis();

      startMap = createSingleSourceVisibilityMap(start, navigableRegions);
      goalMap = createSingleSourceVisibilityMap(goal, navigableRegions);

      if (startMap.getHostRegion() == goalMap.getHostRegion())
      {
         if (isPointVisibleForStaticMaps(startMap.getHostRegion().getAllClusters(), startMap.getSourceInLocal2D(), goalMap.getSourceInLocal2D()))
         {
            startMap.addConnectionInWorld(new Connection(start, startMap.getMapId(), goal, goalMap.getMapId()));
         }
      }

      long startConnectingTime = System.currentTimeMillis();
      interRegionConnections = computeInterRegionConnections(navigableRegions, parameters.getInterRegionConnectionFilter());
      long endConnectingTime = System.currentTimeMillis();

      List<VisibilityMapHolder> visibilityMapHolders = new ArrayList<>();
      visibilityMapHolders.addAll(navigableRegions);
      visibilityMapHolders.add(startMap);
      visibilityMapHolders.add(goalMap);

      long aStarStartTime = System.currentTimeMillis();

      List<Point3DReadOnly> path = null;
      path = calculatePathOnVisibilityGraph(start, goal, interRegionConnections, visibilityMapHolders);

      if (debug)
      {
         if (path != null)
         {
            PrintTools.info("----Navigable Regions Manager Stats-----");
            PrintTools.info("Map creation completed in " + (endCreationTime - startCreatingMaps) + "ms");
            PrintTools.info("Connection completed in " + (endConnectingTime - startConnectingTime) + "ms");
            PrintTools.info("A* took: " + (System.currentTimeMillis() - aStarStartTime) + "ms");
            PrintTools.info("Total time to find solution was: " + (System.currentTimeMillis() - startBodyPathComputation) + "ms");
         }
         else
         {
            PrintTools.info("NO BODY PATH SOLUTION WAS FOUND!" + (System.currentTimeMillis() - startBodyPathComputation) + "ms");
         }
      }

      return path;
   }

   public List<Point3DReadOnly> calculateBodyPathWithOcclussions(Point3D start, Point3D goal)
   {
      List<Point3DReadOnly> path = calculateBodyPath(start, goal);

      if (path == null)
      {
         if (!OcclussionTools.IsTheGoalIntersectingAnyObstacles(navigableRegions.get(0), start, goal))
         {
            System.out.println("StraightLine available");

            path = new ArrayList<>();
            path.add(new Point3D(start));
            path.add(goal);

            return path;
         }

         NavigableRegion regionContainingPoint = PlanarRegionTools.getNavigableRegionContainingThisPoint(start, navigableRegions);
         List<Cluster> intersectingClusters = OcclussionTools.getListOfIntersectingObstacles(regionContainingPoint.getAllClusters(), start, goal);
         Cluster closestCluster = ClusterTools.getTheClosestCluster(start, intersectingClusters);
         Point3D closestExtrusion = ClusterTools.getTheClosestVisibleExtrusionPoint(1.0, start, goal, closestCluster.getNavigableExtrusionsInWorld3D(),
                                                                                    regionContainingPoint.getHomeRegion());

         path = calculateBodyPath(start, closestExtrusion);
         path.add(goal);

         return path;
      }
      else
      {
         return path;
      }
   }

   private static SingleSourceVisibilityMap createSingleSourceVisibilityMap(Point3DReadOnly source, List<NavigableRegion> navigableRegions)
   {
      NavigableRegion hostRegion = PlanarRegionTools.getNavigableRegionContainingThisPoint(source, navigableRegions);
      Point3D sourceInLocal = new Point3D(source);
      hostRegion.transformFromWorldToLocal(sourceInLocal);
      int mapId = hostRegion.getMapId();

      Set<Connection> connections = VisibilityTools.createStaticVisibilityMap(sourceInLocal, mapId, hostRegion.getAllClusters(), mapId, true);
      return new SingleSourceVisibilityMap(source, connections, hostRegion);
   }

   private static List<Point3DReadOnly> calculatePathOnVisibilityGraph(Point3DReadOnly start, Point3DReadOnly goal, Collection<Connection> interConnections,
                                                                       Collection<VisibilityMapHolder> allVisibilityMapHolders)
   {
      SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graph = createGlobalVisibilityGraph(interConnections, allVisibilityMapHolders);
      List<DefaultWeightedEdge> solution = DijkstraShortestPath.findPathBetween(graph, new ConnectionPoint3D(start, START_GOAL_ID),
                                                                                new ConnectionPoint3D(goal, START_GOAL_ID));
      return convertVisibilityGraphSolutionToPath(solution, start, graph);
   }

   private static SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> createGlobalVisibilityGraph(Collection<Connection> interConnections,
                                                                                                          Collection<VisibilityMapHolder> allVisibilityMapHolders)
   {
      SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

      addConnectionsToGraph(interConnections, globalVisMap);
      allVisibilityMapHolders.stream().map(VisibilityMapHolder::getVisibilityMapInWorld).map(VisibilityMap::getConnections)
                             .forEach(connections -> addConnectionsToGraph(connections, globalVisMap));

      return globalVisMap;
   }

   private static void addConnectionsToGraph(Iterable<Connection> connections, SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graphToUpdate)
   {
      connections.forEach(connection -> addConnectionToGraph(connection, graphToUpdate));
   }

   private static void addConnectionToGraph(Connection connection, SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graphToUpdate)
   {
      ConnectionPoint3D source = connection.getSourcePoint();
      ConnectionPoint3D target = connection.getTargetPoint();

      if (!source.epsilonEquals(target, 1.0e-3))
      {
         graphToUpdate.addVertex(source);
         graphToUpdate.addVertex(target);
         DefaultWeightedEdge edge = new DefaultWeightedEdge();
         graphToUpdate.addEdge(source, target, edge);
         graphToUpdate.setEdgeWeight(edge, source.distance(target));
      }
   }

   private static List<Point3DReadOnly> convertVisibilityGraphSolutionToPath(List<DefaultWeightedEdge> solution, Point3DReadOnly start,
                                                                             Graph<ConnectionPoint3D, DefaultWeightedEdge> graph)
   {
      List<Point3DReadOnly> path = new ArrayList<>();
      path.clear();

      if (solution == null)
      {
         if (debug)
            PrintTools.info("WARNING - Visibility graph found no solution");
      }
      else
      {
         for (DefaultWeightedEdge edge : solution)
         {
            Point3DReadOnly from = graph.getEdgeSource(edge);
            Point3DReadOnly to = graph.getEdgeTarget(edge);

            if (!path.contains(new Point3D(from)))
               path.add(new Point3D(from));
            if (!path.contains(new Point3D(to)))
               path.add(new Point3D(to));
         }

         // FIXME Sylvain: it looks like this is to cover a bug.
         if (!path.get(0).epsilonEquals(start, 1e-5))
         {
            Point3DReadOnly pointOut = path.get(1);
            path.remove(1);
            path.add(0, pointOut);
         }

         if (debug)
            PrintTools.info("Visibility graph successfully found a solution");
      }

      return path;
   }

   private static List<Connection> computeInterRegionConnections(List<NavigableRegion> navigableRegions, InterRegionConnectionFilter filter)
   {
      List<Connection> interRegionConnections = new ArrayList<>();
      if (debug)
      {
         PrintTools.info("Starting connectivity check");
      }
      for (int sourceMapIndex = 0; sourceMapIndex < navigableRegions.size(); sourceMapIndex++)
      {
         VisibilityMap sourceMap = navigableRegions.get(sourceMapIndex).getVisibilityMapInWorld();
         Set<ConnectionPoint3D> sourcePoints = sourceMap.getVertices();

         for (ConnectionPoint3D source : sourcePoints)
         {
            for (int targetMapIndex = sourceMapIndex + 1; targetMapIndex < navigableRegions.size(); targetMapIndex++)
            {
               VisibilityMap targetMap = navigableRegions.get(targetMapIndex).getVisibilityMapInWorld();

               Set<ConnectionPoint3D> targetPoints = targetMap.getVertices();

               for (ConnectionPoint3D target : targetPoints)
               {
                  if (source.getRegionId() == target.getRegionId())
                     continue;

                  if (filter.isConnectionValid(source, target))
                  {
                     interRegionConnections.add(new Connection(source, target));
                  }
               }
            }
         }
      }

      return interRegionConnections;
   }

   private NavigableRegion createVisibilityGraphForRegion(PlanarRegion region)
   {
      if (debug)
      {
         PrintTools.info("Creating a visibility graph for region with ID:" + region.getRegionId());
      }

      NavigableRegion navigableRegion = new NavigableRegion(region);
      processRegion(navigableRegion);
      return navigableRegion;
   }

   private void processRegion(NavigableRegion navigableRegionLocalPlanner)
   {
      List<PlanarRegion> lineObstacleRegions = new ArrayList<>();
      List<PlanarRegion> polygonObstacleRegions = new ArrayList<>();
      List<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();
      List<Cluster> clusters = new ArrayList<>();
      PlanarRegion homeRegion = navigableRegionLocalPlanner.getHomeRegion();

      regionsInsideHomeRegion = PlanarRegionTools.determineWhichRegionsAreInside(homeRegion, regions);
      if (TRUNCATE_OBSTACLE_REGIONS)
      {
         double depthThresholdForConvexDecomposition = 0.05; // TODO Extract me!
         int minTruncatedSize = 0; // TODO Extract me!
         double minTruncatedArea = 0.01; // TODO Extract me!
         regionsInsideHomeRegion = PlanarRegionTools.filterRegionsByTruncatingVerticesBeneathHomeRegion(regionsInsideHomeRegion, homeRegion,
                                                                                                        depthThresholdForConvexDecomposition, minTruncatedSize,
                                                                                                        minTruncatedArea);
      }
      else
      {
         regionsInsideHomeRegion = PlanarRegionTools.keepOnlyRegionsThatAreEntirelyAboveHomeRegion(regionsInsideHomeRegion, homeRegion);
      }

      double normalZThresholdForPolygonObstacles = parameters.getNormalZThresholdForPolygonObstacles();
      RigidBodyTransform transformToWorldFrame = navigableRegionLocalPlanner.getTransformToWorld();
      double extrusionDistance = parameters.getExtrusionDistance();

      ClusterTools.classifyExtrusions(regionsInsideHomeRegion, homeRegion, lineObstacleRegions, polygonObstacleRegions, normalZThresholdForPolygonObstacles);
      ClusterTools.createClustersFromRegions(homeRegion, regionsInsideHomeRegion, lineObstacleRegions, polygonObstacleRegions, clusters, transformToWorldFrame,
                                             parameters);
      ClusterTools.createClusterForHomeRegion(clusters, transformToWorldFrame, homeRegion, extrusionDistance);

      if (debug)
      {
         System.out.println("Extruding obstacles...");
      }

      ClusterTools.performExtrusions(parameters.getExtrusionDistanceCalculator(), clusters);

      for (Cluster cluster : clusters)
      {
         PointCloudTools.doBrakeDownOn2DPoints(cluster.getNavigableExtrusionsInLocal2D(), parameters.getClusterResolution());
      }

      Collection<Connection> connectionsForMap = VisibilityTools.createStaticVisibilityMap(clusters, navigableRegionLocalPlanner);

      connectionsForMap = VisibilityTools.removeConnectionsFromExtrusionsOutsideRegions(connectionsForMap, homeRegion);
      connectionsForMap = VisibilityTools.removeConnectionsFromExtrusionsInsideNoGoZones(connectionsForMap, clusters);

      VisibilityMap visibilityMap = new VisibilityMap();
      visibilityMap.setConnections(connectionsForMap);

      navigableRegionLocalPlanner.setClusters(clusters);
      navigableRegionLocalPlanner.setRegionsInsideHomeRegion(regionsInsideHomeRegion);
      navigableRegionLocalPlanner.setLineObstacleRegions(lineObstacleRegions);
      navigableRegionLocalPlanner.setPolygonObstacleRegions(polygonObstacleRegions);
      navigableRegionLocalPlanner.setVisibilityMapInLocal(visibilityMap);
   }

   public Point3D[][] getNavigableExtrusions()
   {
      Point3D[][] allNavigableExtrusions = new Point3D[navigableRegions.size()][];

      for (int i = 0; i < navigableRegions.size(); i++)
      {
         NavigableRegion localPlanner = navigableRegions.get(i);
         Point3D[] navigableExtrusions = new Point3D[localPlanner.getAllClusters().size()];

         for (Cluster cluster : localPlanner.getAllClusters())
         {
            for (int j = 0; j < cluster.getNumberOfNavigableExtrusions(); j++)
            {
               navigableExtrusions[j] = cluster.getNavigableExtrusionInWorld3D(j);
            }
         }

         allNavigableExtrusions[i] = navigableExtrusions;
      }

      return allNavigableExtrusions;
   }

   public VisibilityMapHolder getStartMap()
   {
      return startMap;
   }

   public VisibilityMapHolder getGoalMap()
   {
      return goalMap;
   }

   public List<NavigableRegion> getNavigableRegions()
   {
      return navigableRegions;
   }

   public List<Connection> getInterRegionConnections()
   {
      return interRegionConnections;
   }
}
