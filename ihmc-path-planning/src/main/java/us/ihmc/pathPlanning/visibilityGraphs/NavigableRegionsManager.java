package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import java.util.Set;

import org.jgrapht.Graph;
import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.OcclussionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class NavigableRegionsManager
{
   private final static boolean debug = false;
   private final static boolean CONNECT_GOAL_TO_CLOSEST_REGION = false;
   private final static boolean TRUNCATE_OBSTACLE_REGIONS = true;

   private final static int START_GOAL_ID = 0;

   private List<PlanarRegion> regions;
   private List<PlanarRegion> accesibleRegions = new ArrayList<>();
   private List<PlanarRegion> obstacleRegions = new ArrayList<>();
   private List<NavigableRegion> navigableRegions = new ArrayList<>();
   private List<VisibilityMap> visMaps = new ArrayList<>();
   private SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

   private final VisibilityGraphsParameters parameters;

   private ArrayList<Connection> connectionPoints = new ArrayList<>();
   private ArrayList<Connection> globalMapPoints = new ArrayList<>();

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

   private void createIndividualVisMapsForRegions()
   {
      navigableRegions.clear();
      visMaps.clear();
      accesibleRegions.clear();

      PlanarRegionTools.classifyRegions(regions, parameters.getNormalZThresholdForAccessibleRegions(), obstacleRegions, accesibleRegions);

      for (PlanarRegion region : accesibleRegions)
      {
         createVisibilityGraphForRegion(region, null, null);
      }

   }

   public List<Point3DReadOnly> calculateBodyPath(final Point3DReadOnly start, final Point3DReadOnly goal)
   {
      connectionPoints.clear();
      globalMapPoints.clear();

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

      createIndividualVisMapsForRegions();

      long endCreationTime = System.currentTimeMillis();

      long startForcingPoints = System.currentTimeMillis();

      if (CONNECT_GOAL_TO_CLOSEST_REGION)
         connectToClosestRegions(goal, START_GOAL_ID);

      long endForcingPoints = System.currentTimeMillis();

      boolean readyToRunBodyPath = createVisMapsForStartAndGoal(start, goal);

      createGlobalMapFromAlltheLocalMaps();
      long startConnectingTime = System.currentTimeMillis();
      connectLocalMaps();
      long endConnectingTime = System.currentTimeMillis();

      if (readyToRunBodyPath)
      {
         long startGlobalMapTime = System.currentTimeMillis();
         createGlobalVisibilityGraph();
         long endGlobalMapTime = System.currentTimeMillis();

         long aStarStartTime = System.currentTimeMillis();

         List<Point3DReadOnly> path = null;
         path = calculatePathOnVisibilityGraph(start, goal, globalVisMap);

         if (debug)
         {
            if (path != null)
            {
               PrintTools.info("----Navigable Regions Manager Stats-----");
               PrintTools.info("Map creation completed in " + (endCreationTime - startCreatingMaps) + "ms");
               PrintTools.info("Connection completed in " + (endConnectingTime - startConnectingTime) + "ms");
               PrintTools.info("Forcing points took: " + (endForcingPoints - startForcingPoints) + "ms");
               PrintTools.info("Global Map creation took " + (endGlobalMapTime - startGlobalMapTime) + "ms");
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

      return null;
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

   private boolean createVisMapsForStartAndGoal(Point3DReadOnly start, Point3DReadOnly goal)
   {
      NavigableRegion startRegion = PlanarRegionTools.getNavigableRegionContainingThisPoint(start, navigableRegions);
      Point3D startInRegionFrame = new Point3D(start);
      startRegion.transformFromWorldToLocal(startInRegionFrame);
      NavigableRegion goalRegion = PlanarRegionTools.getNavigableRegionContainingThisPoint(goal, navigableRegions);
      Point3D goalInRegionFrame = new Point3D(goal);
      goalRegion.transformFromWorldToLocal(goalInRegionFrame);

      VisibilityMap startMap = createVisMapForSinglePointSource(startInRegionFrame, startRegion.getRegionId(), startRegion, true);
      VisibilityMap goalMap = createVisMapForSinglePointSource(goalInRegionFrame, goalRegion.getRegionId(), goalRegion, true);

      if (startRegion == goalRegion)
      {
         Point2D start2D = new Point2D(startInRegionFrame);
         Point2D goal2D = new Point2D(goalInRegionFrame);
         boolean targetIsVisible = VisibilityTools.isPointVisibleForStaticMaps(startRegion.getAllClusters(), start2D, goal2D);
         if (targetIsVisible)
         {
            globalMapPoints.add(new Connection(start, startRegion.getRegionId(), goal, goalRegion.getRegionId()));
         }
      }

      if (startMap.isEmpty() || goalMap.isEmpty())
         return false;

      visMaps.add(startMap);
      visMaps.add(goalMap);
      return true;
   }

   private static List<Point3DReadOnly> calculatePathOnVisibilityGraph(Point3DReadOnly start, Point3DReadOnly goal,
                                                                       Graph<ConnectionPoint3D, DefaultWeightedEdge> graph)
   {
      List<DefaultWeightedEdge> solution = DijkstraShortestPath.findPathBetween(graph, new ConnectionPoint3D(start, START_GOAL_ID),
                                                                                new ConnectionPoint3D(goal, START_GOAL_ID));
      return convertVisibilityGraphSolutionToPath(solution, start, graph);
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

   private void createGlobalVisibilityGraph()
   {
      globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

      for (Connection pair : globalMapPoints)
      {
         ConnectionPoint3D pt1 = pair.getSourcePoint();
         ConnectionPoint3D pt2 = pair.getTargetPoint();

         if (!pt1.geometricallyEquals(pt2, 1.0e-3))
         {
            globalVisMap.addVertex(pt1);
            globalVisMap.addVertex(pt2);
            DefaultWeightedEdge edge = new DefaultWeightedEdge();
            globalVisMap.addEdge(pt1, pt2, edge);
            globalVisMap.setEdgeWeight(edge, pt1.distance(pt2));
         }
      }
   }

   private static VisibilityMap createVisMapForSinglePointSource(Point3DReadOnly point, int pointRegionId, NavigableRegion navigableRegion,
                                                                 boolean ensureConnection)
   {
      Set<Connection> connections = VisibilityTools.createStaticVisibilityMap(point, pointRegionId, navigableRegion.getAllClusters(),
                                                                              navigableRegion.getRegionId(), ensureConnection);

      VisibilityMap mapForSingleObserver = new VisibilityMap();
      mapForSingleObserver.setConnections(connections);
      navigableRegion.transformFromLocalToWorld(mapForSingleObserver);
      mapForSingleObserver.computeVertices();
      return mapForSingleObserver;
   }

   private void createGlobalMapFromAlltheLocalMaps()
   {
      for (VisibilityMap map : visMaps)
      {
         for (Connection connection : map.getConnections())
         {
            globalMapPoints.add(new Connection(connection.getSourcePoint(), connection.getTargetPoint()));
         }
      }
   }

   private void connectToClosestRegions(Point3DReadOnly position, int positionId)
   {
      ArrayList<PlanarRegionDistance> planarRegionsDistance = new ArrayList<>();

      for (NavigableRegion planner : navigableRegions)
      {
         double minDistance = Double.MAX_VALUE;
         PlanarRegion homeRegion = planner.getHomeRegion();
         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         homeRegion.getTransformToWorld(transformToWorld);

         for (int i = 0; i < homeRegion.getConcaveHull().length; i++)
         {
            Point3D pointInWorld = new Point3D(homeRegion.getConcaveHull()[i]);
            pointInWorld.applyTransform(transformToWorld);

            double currentDistance = position.distanceSquared(pointInWorld);

            if (currentDistance < minDistance)
            {
               minDistance = currentDistance;
            }
         }

         planarRegionsDistance.add(new PlanarRegionDistance(homeRegion, minDistance));
      }

      planarRegionsDistance.sort(new PlanarRegionDistanceComparator());

      PlanarRegion closestRegion = planarRegionsDistance.get(0).getRegion();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      closestRegion.getTransformToWorld(transformToWorld);

      for (int i = 0; i < closestRegion.getConcaveHull().length; i++)
      {
         Point3D pointInWorld = new Point3D(closestRegion.getConcaveHull()[i]);
         pointInWorld.applyTransform(transformToWorld);

         connectionPoints.add(new Connection(position, positionId, pointInWorld, closestRegion.getRegionId()));
      }

      if (debug)
         System.out.println("Sorted: " + planarRegionsDistance.size() + " planar regions");
   }

   private void connectLocalMaps()
   {
      if (debug)
      {
         PrintTools.info("Starting connectivity check");
      }

      if (navigableRegions.size() > 1)
      {
         for (VisibilityMap sourceMap : visMaps)
         {
            Set<ConnectionPoint3D> sourcePoints = sourceMap.getVertices();

            for (ConnectionPoint3D sourcePt : sourcePoints)
            {
               for (VisibilityMap targetMap : visMaps)
               {
                  if (sourceMap != targetMap)
                  {
                     Set<ConnectionPoint3D> targetPoints = targetMap.getVertices();

                     for (ConnectionPoint3D targetPt : targetPoints)
                     {
                        if (sourcePt.getRegionId() == targetPt.getRegionId())
                           continue;

                        boolean distanceOk = sourcePt.distance(targetPt) < parameters.getMinimumConnectionDistanceForRegions();
                        boolean heightOk = Math.abs(sourcePt.getZ() - targetPt.getZ()) < parameters.getTooHighToStepDistance();

                        if (distanceOk && heightOk)
                        {
                           connectionPoints.add(new Connection(sourcePt, targetPt));
                           globalMapPoints.add(new Connection(sourcePt, targetPt));
                        }
                     }
                  }
               }
            }
         }
      }
   }

   private void createVisibilityGraphForRegion(PlanarRegion region, Point3D startPos, Point3D goalPos)
   {
      if (debug)
      {
         PrintTools.info("Creating a visibility graph for region with ID:" + region.getRegionId());
      }

      NavigableRegion navigableRegion = new NavigableRegion(region);
      processRegion(navigableRegion);
      navigableRegions.add(navigableRegion);
      visMaps.add(navigableRegion.getVisibilityGraphInWorld());
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

      // TODO The use of Double.MAX_VALUE for the observer seems rather risky. I'm actually surprised that it works.
      //      clusterMgr.performExtrusions(new Point2D(Double.MAX_VALUE, Double.MAX_VALUE), parameters.getExtrusionDistance());
      ClusterTools.performExtrusions(new Point2D(Double.MAX_VALUE, Double.MAX_VALUE), parameters.getExtrusionDistanceCalculator(), clusters);

      for (Cluster cluster : clusters)
      {
         PointCloudTools.doBrakeDownOn2DPoints(cluster.getNavigableExtrusionsInLocal2D(), parameters.getClusterResolution());
      }

      Collection<Connection> connectionsForMap = VisibilityTools.createStaticVisibilityMap(null, null, clusters, navigableRegionLocalPlanner.getRegionId());

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

   public List<NavigableRegion> getListOfLocalPlanners()
   {
      return navigableRegions;
   }

   public List<PlanarRegion> getListOfAccesibleRegions()
   {
      return accesibleRegions;
   }

   public List<PlanarRegion> getListOfObstacleRegions()
   {
      return obstacleRegions;
   }

   public ArrayList<Connection> getGlobalMapPoints()
   {
      return globalMapPoints;
   }

   public ArrayList<Connection> getConnectionPoints()
   {
      return connectionPoints;
   }

   private class PlanarRegionDistance implements Comparable<PlanarRegionDistance>
   {
      PlanarRegion region;
      double distance;

      public PlanarRegionDistance(PlanarRegion region, double distance)
      {
         this.region = region;
         this.distance = distance;
      }

      public PlanarRegion getRegion()
      {
         return region;
      }

      @Override
      public int compareTo(PlanarRegionDistance region)
      {
         if (distance > region.distance)
         {
            return 1;
         }
         else if (distance < region.distance)
         {
            return -1;
         }
         else
         {
            return 0;
         }
      }
   }

   private class PlanarRegionDistanceComparator implements Comparator<PlanarRegionDistance>
   {
      @Override
      public int compare(PlanarRegionDistance region1, PlanarRegionDistance region2)
      {
         return region1.compareTo(region2);
      }
   }
}
