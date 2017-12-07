package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;

import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class NavigableRegionsManager
{
   private final static boolean debug = true;

   private List<PlanarRegion> regions;
   private List<PlanarRegion> accesibleRegions = new ArrayList<>();
   private List<PlanarRegion> obstacleRegions = new ArrayList<>();
   private List<NavigableRegion> listOfLocalPlanners = new ArrayList<>();
   private List<VisibilityMap> visMaps = new ArrayList<>();
   private SimpleWeightedGraph<Point3D, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

   private JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;
   private double pathLength = 0.0;
   private final VisibilityGraphsParameters parameters;

   private ArrayList<Connection> connectionPoints = new ArrayList<>();
   private ArrayList<Connection> globalMapPoints = new ArrayList<>();
   private ArrayList<DistancePoint> distancePoints = new ArrayList<>();

   public NavigableRegionsManager(VisibilityGraphsParameters parameters)
   {
      this(parameters, null, null);
   }

   public NavigableRegionsManager(List<PlanarRegion> regions)
   {
      this(regions, null);
   }

   public NavigableRegionsManager(VisibilityGraphsParameters parameters, List<PlanarRegion> regions)
   {
      this(parameters, regions, null);
   }

   public NavigableRegionsManager(JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this(null, null, javaFXMultiColorMeshBuilder);
   }

   public NavigableRegionsManager(VisibilityGraphsParameters parameters, JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this(parameters, null, javaFXMultiColorMeshBuilder);
   }

   public NavigableRegionsManager(List<PlanarRegion> regions, JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this(null, regions, javaFXMultiColorMeshBuilder);
   }

   public NavigableRegionsManager(VisibilityGraphsParameters parameters, List<PlanarRegion> regions, JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      this.regions = regions;
      this.javaFXMultiColorMeshBuilder = javaFXMultiColorMeshBuilder;
      this.parameters = parameters == null ? new DefaultVisibilityGraphParameters() : parameters;
   }

   public void setPlanarRegions(List<PlanarRegion> regions)
   {
      //      ArrayList<PlanarRegion> regions1 = new ArrayList<>();
      //      regions1.add(regions.get(0));
      //      regions1.add(regions.get(1));
      //      regions1.add(regions.get(2));
      //      regions1.add(regions.get(3));
      //      regions1.add(regions.get(4));
      //      regions1.add(regions.get(5));
      //      regions1.add(regions.get(6));
      //    regions1.add(regions.get(20)); //slanted
      //    regions1.add(regions.get(21)); //slanted 2
      //    regions1.add(regions.get(24)); //slanted 3

      regions = PlanarRegionTools.filterPlanarRegionsByArea(parameters.getPlanarRegionMinArea(), regions);
      regions = PlanarRegionTools.filterPlanarRegionsByHullSize(parameters.getPlanarRegionMinSize(), regions);

      this.regions = regions;
   }

   public List<Point3D> calculateBodyPath(Point3D start, Point3D goal)
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

      // Projecting start and goal onto the closest region if inside and close
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(regions);
      Point3D startProjected = PlanarRegionTools.projectPointToPlanesVertically(start, planarRegionsList);
      Point3D goalProjected = PlanarRegionTools.projectPointToPlanesVertically(goal, planarRegionsList);

      if (startProjected != null && start.distance(startProjected) < parameters.getMaxDistanceToProjectStartGoalToClosestRegion())
         start = startProjected;
      if (goalProjected != null && goal.distance(goalProjected) < parameters.getMaxDistanceToProjectStartGoalToClosestRegion())
         goal = goalProjected;

      regions = PlanarRegionTools.filterPlanarRegionsWithBoundingCapsule(start, goal, parameters.getExplorationDistanceFromStartGoal(), regions);

      long startBodyPathComputation = System.currentTimeMillis();

      long startCreatingMaps = System.currentTimeMillis();
      listOfLocalPlanners.clear();
      visMaps.clear();
      accesibleRegions.clear();

      PlanarRegionTools.classifyRegions(regions, parameters.getNormalZThresholdForAccessibleRegions(), obstacleRegions, accesibleRegions);

      for (PlanarRegion region : accesibleRegions)
      {
         createVisibilityGraphForRegion(region, start, goal);
      }

      long endCreationTime = System.currentTimeMillis();

      connectionPoints.clear();
      globalMapPoints.clear();

      VisibilityMap startMap = createVisMapForSinglePointSource(startProjected);
      VisibilityMap goalMap = createVisMapForSinglePointSource(goalProjected);
      
//      boolean targetIsVisible = isPointVisibleForStaticMaps(clusters, observer, target);


      if (startMap != null && goalMap != null)
      {
         visMaps.add(startMap);
         visMaps.add(goalMap);

         createGlobalMapFromAlltheLocalMaps();

         long startConnectingTime = System.currentTimeMillis();
         connectLocalMaps();
         long endConnectingTime = System.currentTimeMillis();

         long startForcingPoints = System.currentTimeMillis();
         start = forceConnectionOrSnapPoint(start);

         if (debug)
         {
            if (start == null)
               PrintTools.error("Visibility graph unable to snap the start point to the closest point in the graph");
         }

         if (start == null)
         {
            throw new RuntimeException("Visibility graph unable to snap the start point to the closest point in the graph");
         }

         goal = forceConnectionOrSnapPoint(goal);
         connectToClosestRegions(goal);

         if (debug)
         {
            if (goal == null)
               PrintTools.error("Visibility graph unable to snap the goal point to the closest point in the graph");
         }

         if (goal == null)
         {
            throw new RuntimeException("Visibility graph unable to snap the goal point to the closest point in the graph");
         }

         long endForcingPoints = System.currentTimeMillis();

         long startGlobalMapTime = System.currentTimeMillis();
         createGlobalVisibilityGraph();
         long endGlobalMapTime = System.currentTimeMillis();

         long startSnappingTime = System.currentTimeMillis();
         Point3D snappedGoalPosition = getSnappedPointFromVisibilityGraph(goal);
         if (debug && snappedGoalPosition == null)
         {
            PrintTools.error("Snapping of goal returned null.");
         }
         Point3D snappedStartPosition = getSnappedPointFromVisibilityGraph(start);
         if (debug && snappedStartPosition == null)
         {
            PrintTools.error("Snapping of start returned null.");
         }
         if (snappedGoalPosition == null || snappedStartPosition == null)
         {
            throw new RuntimeException("Snapping start/goal from visibility graph has failed.");
         }
         long endSnappingTime = System.currentTimeMillis();

         long aStarStartTime = System.currentTimeMillis();

         List<Point3D> path = null;
         if (snappedGoalPosition != null && snappedStartPosition != null)
         {
            path = calculatePathOnVisibilityGraph(snappedStartPosition, snappedGoalPosition);
         }
         else
         {
            if (debug)
               PrintTools.error("Start or goal pose is null, visibilty graph unable to compute a path!");
         }

         if (debug)
         {
            if (path != null)
            {
               PrintTools.info("----Navigable Regions Manager Stats-----");
               PrintTools.info("Map creation completed in " + (endCreationTime - startCreatingMaps) + "ms");
               PrintTools.info("Connection completed in " + (endConnectingTime - startConnectingTime) + "ms");
               PrintTools.info("Forcing points took: " + (endForcingPoints - startForcingPoints) + "ms");
               PrintTools.info("Global Map creation took " + (endGlobalMapTime - startGlobalMapTime) + "ms");
               PrintTools.info("Snapping points took: " + (endSnappingTime - startSnappingTime) + "ms");
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

   private static PlanarRegion getLargestAreaRegion(List<PlanarRegion> planarRegions)
   {
      double largestArea = Double.NEGATIVE_INFINITY;
      int index = -1;

      for (int i = 0; i < planarRegions.size(); i++)
      {
         double area = 0.0;
         PlanarRegion region = planarRegions.get(i);

         for (int j = 0; j < planarRegions.get(i).getNumberOfConvexPolygons(); j++)
         {
            area += region.getConvexPolygon(j).getArea();
         }

         if (area > largestArea)
         {
            largestArea = area;
            index = i;
         }
      }

      return planarRegions.get(index);
   }

   private List<Point3D> calculatePathOnVisibilityGraph(Point3D start, Point3D goal)
   {
      ArrayList<DefaultWeightedEdge> solution = (ArrayList<DefaultWeightedEdge>) DijkstraShortestPath.findPathBetween(globalVisMap, start, goal);
      return convertVisibilityGraphSolutionToPath(solution, start);
   }

   private List<Point3D> convertVisibilityGraphSolutionToPath(ArrayList<DefaultWeightedEdge> solution, Point3D start)
   {
      List<Point3D> path = new ArrayList<>();
      pathLength = 0.0;
      path.clear();

      if (solution != null)
      {
         for (DefaultWeightedEdge edge : solution)
         {
            Point3D from = globalVisMap.getEdgeSource(edge);
            Point3D to = globalVisMap.getEdgeTarget(edge);
            pathLength = pathLength + from.distance(to);

            if (!path.contains(new Point3D(from)))
               path.add(from);
            if (!path.contains(new Point3D(to)))
               path.add(to);
         }

         if (!path.get(0).epsilonEquals(start, 1e-5))
         {
            Point3D pointOut = path.get(1);
            path.remove(1);
            path.add(0, pointOut);
         }

         if (debug)
            PrintTools.info("Visibility graph successfully found a solution");
      }
      else
      {
         if (debug)
            PrintTools.info("WARNING - Visibility graph found no solution");
      }

      return path;
   }

   private Point3D forceConnectionOrSnapPoint(Point3D pointToCheck)
   {
      if (PlanarRegionTools.isPointInsideAnyRegion(accesibleRegions, pointToCheck))
      {
         if (isPointInsideNoGoZone(pointToCheck))
         {
            if (debug)
            {
               PrintTools.info("Point " + pointToCheck + " is in a NO-GO zone. Snapping the point to the closest navigable point.");
            }
            pointToCheck = snapDesiredPointToClosestPoint(pointToCheck);
         }
      }
      else
      {
         if (debug)
         {
            PrintTools.info("Point " + pointToCheck + " is outside a planar region. Forcing connections to the closest navigable regions");
         }
         forceConnectionToPoint(pointToCheck);
      }
      return pointToCheck;
   }

   private void createGlobalVisibilityGraph()
   {
      globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

      int actualConnectionsAdded = 0;
      for (Connection pair : globalMapPoints)
      {
         Point3D pt1 = pair.getSourcePoint();
         Point3D pt2 = pair.getTargetPoint();
         if (!pt1.epsilonEquals(pt2, 1e-5))
         {
            globalVisMap.addVertex(pt1);
            globalVisMap.addVertex(pt2);
            DefaultWeightedEdge edge = new DefaultWeightedEdge();
            globalVisMap.addEdge(pt1, pt2, edge);
            globalVisMap.setEdgeWeight(edge, pt1.distance(pt2));
            actualConnectionsAdded++;
         }
      }
      //      System.out.println("Actual connections added: " + actualConnectionsAdded);
   }

   public VisibilityMap createVisMapForSinglePointSource(Point3D point)
   {

      for (NavigableRegion planner : listOfLocalPlanners)
      {
         FramePoint3D pointFpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), point);
         pointFpt.changeFrame(planner.getLocalReferenceFrame());

         Point2D point2D = null;

         if (PlanarRegionTools.isPointInLocalInsideARegion(planner.getHomeRegion(), pointFpt.getPoint()))
         {
            point2D = new Point2D(pointFpt.getPoint().getX(), pointFpt.getPoint().getY());
         }

         if (point2D != null)
         {
            HashSet<Connection> connections = VisibilityTools.createStaticVisibilityMap(point2D, planner.getClusters());

            VisibilityMap mapForSingleObserverLocal = new VisibilityMap();
            mapForSingleObserverLocal.setConnections(connections);

            VisibilityMap mapForSingleObserverWorld = new VisibilityMap();

            for (Connection connection : mapForSingleObserverLocal.getConnections())
            {
               Point2D edgeSource = new Point2D(connection.getSourcePoint().getX(), connection.getSourcePoint().getY());
               Point2D edgeTarget = new Point2D(connection.getTargetPoint().getX(), connection.getTargetPoint().getY());

               FramePoint3D pt1 = new FramePoint3D(planner.getLocalReferenceFrame(), new Point3D(edgeSource.getX(), edgeSource.getY(), 0));
               pt1.changeFrame(ReferenceFrame.getWorldFrame());
               FramePoint3D pt2 = new FramePoint3D(planner.getLocalReferenceFrame(), new Point3D(edgeTarget.getX(), edgeTarget.getY(), 0));
               pt2.changeFrame(ReferenceFrame.getWorldFrame());

               mapForSingleObserverWorld.addConnection(new Connection(pt1.getPoint(), pt2.getPoint()));
            }

            mapForSingleObserverWorld.computeVertices();
            return mapForSingleObserverWorld;
         }
      }
      return null;
   }

   public Point3D getSnappedPointFromVisibilityGraph(Point3D position)
   {
      for (DefaultWeightedEdge edge : globalVisMap.edgeSet())
      {
         Point3D source = globalVisMap.getEdgeSource(edge);

         if (Math.abs(source.getX() - position.getX()) < 0.001)
         {
            if (Math.abs(source.getY() - position.getY()) < 0.001)
            {
               if (Math.abs(source.getZ() - position.getZ()) < 0.001)
               {
                  return source;
               }
            }
         }

         Point3D target = globalVisMap.getEdgeTarget(edge);

         if (Math.abs(target.getX() - position.getX()) < 0.001)
         {
            if (Math.abs(target.getY() - position.getY()) < 0.001)
            {
               if (Math.abs(target.getZ() - position.getZ()) < 0.001)
               {
                  return target;
               }
            }
         }
      }

      return null;
   }

   private void createGlobalMapFromAlltheLocalMaps()
   {
      int connectionsAdded = 0;
      for (VisibilityMap map : visMaps)
      {
         for (Connection connection : map.getConnections())
         {
            globalMapPoints.add(new Connection(connection.getSourcePoint(), connection.getTargetPoint()));
            connectionsAdded++;
         }
      }

      //      System.out.println("Creating global map added " + connectionsAdded + " connections");
   }

   private Point3D snapDesiredPointToClosestPoint(Point3D desiredPointToSnap)
   {
      int connectionsAdded = 0;
      if (debug)
      {
         PrintTools.info("------>>>>  Point: " + desiredPointToSnap + " is inside a planar region and a No-Go-Zone - snapping connection");
      }

      distancePoints.clear();

      int index1 = 0;
      for (Connection pair : globalMapPoints)
      {
         DistancePoint point1 = new DistancePoint(pair.getSourcePoint(), pair.getSourcePoint().distance(desiredPointToSnap));
         DistancePoint point2 = new DistancePoint(pair.getTargetPoint(), pair.getTargetPoint().distance(desiredPointToSnap));

         distancePoints.add(point1);
         distancePoints.add(point2);
         index1++;
      }

      //      System.out.println("Added raw: " + index1);

      distancePoints.sort(new DistancePointComparator());

      ArrayList<Point3D> filteredList = new ArrayList<>();

      for (DistancePoint point : distancePoints)
      {
         //         System.out.println("Adding connection "+ point.point + "  to " + position);

         filteredList.add(point.point);
      }

      //      System.out.println("After filtering: " + filteredList.size());

      Iterator<Point3D> it = filteredList.iterator();

      Point3D pointToSnapTo = null;
      while (it.hasNext())
      {
         Point3D source = it.next();
         Point3D target = it.next();

         //Cannot add an edge where the source is equal to the target!
         if (!source.epsilonEquals(desiredPointToSnap, 1e-5))
         {
            pointToSnapTo = target;
            //            System.out.println("-----> new snapped point: " + pointToSnapTo);
            connectionPoints.add(new Connection(pointToSnapTo, desiredPointToSnap));
            break;
         }
      }

      return pointToSnapTo;
   }

   private void forceConnectionToPoint(Point3D position)
   {
      int connectionsAdded = 0;
      if (debug)
      {
         PrintTools.info("------>>>>  Point: " + position + " is not inside a planar region - forcing connection to closest points");
      }

      distancePoints.clear();

      for (Connection pair : globalMapPoints)
      {
         DistancePoint point1 = new DistancePoint(pair.getSourcePoint(), pair.getSourcePoint().distance(position));
         DistancePoint point2 = new DistancePoint(pair.getTargetPoint(), pair.getTargetPoint().distance(position));

         distancePoints.add(point1);
         distancePoints.add(point2);
      }

      distancePoints.sort(new DistancePointComparator());

      ArrayList<Point3D> listOfOrderedPoints = new ArrayList<>();

      for (DistancePoint point : distancePoints)
      {
         listOfOrderedPoints.add(point.point);
      }

      Iterator<Point3D> it = listOfOrderedPoints.iterator();

      int index = 0;
      while (it.hasNext() && index < parameters.getNumberOfForcedConnections())
      {
         Point3D closestPoint = it.next();

         //Cannot add an edge where the source is equal to the target!
         if (!closestPoint.epsilonEquals(position, 1e-5))
         {
            globalMapPoints.add(new Connection(closestPoint, position));
            connectionPoints.add(new Connection(closestPoint, position));
            connectionsAdded++;
            index++;
         }
      }
   }

   private void connectToClosestRegions(Point3D position)
   {
      ArrayList<PlanarRegionDistance> planarRegsDistance = new ArrayList<>();

      for (NavigableRegion planner : listOfLocalPlanners)
      {
         double minDist = Double.MAX_VALUE;
         for (int i = 0; i < planner.getHomeRegion().getConcaveHull().length; i++)
         {
            Point2D point2D = (Point2D) planner.getHomeRegion().getConcaveHull()[i];
            Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
            FramePoint3D pointInWorld = new FramePoint3D();
            pointInWorld.set(point3D);
            RigidBodyTransform transToWorld = new RigidBodyTransform();
            planner.getHomeRegion().getTransformToWorld(transToWorld);
            pointInWorld.applyTransform(transToWorld);

            if (position.distance(pointInWorld) < minDist)
            {
               minDist = position.distance(pointInWorld);
            }
         }
         planarRegsDistance.add(new PlanarRegionDistance(planner.getHomeRegion(), minDist));
      }

      planarRegsDistance.sort(new PlanarRegionDistanceComparator());

      for (int i = 0; i < planarRegsDistance.get(0).getRegion().getConcaveHull().length; i++)
      {
         Point2D point2D = (Point2D) planarRegsDistance.get(0).getRegion().getConcaveHull()[i];
         Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);
         FramePoint3D pointInWorld = new FramePoint3D();
         pointInWorld.set(point3D);
         RigidBodyTransform transToWorld = new RigidBodyTransform();
         planarRegsDistance.get(0).getRegion().getTransformToWorld(transToWorld);
         pointInWorld.applyTransform(transToWorld);

         //         connectionPoints.add(new Connection(position, pointInWorld.getPoint()));
      }

      System.out.println("Sorted: " + planarRegsDistance.size() + " planar regions");
   }

   private void connectLocalMaps()
   {
      int connectionsAdded = 0;
      if (debug)
      {
         PrintTools.info("Starting connectivity check");
      }

      if (listOfLocalPlanners.size() > 1)
      {
         for (VisibilityMap sourceMap : visMaps)
         {
            HashSet<Point3D> sourcePoints = sourceMap.getVertices();

            for (Point3D sourcePt : sourcePoints)
            {
               for (VisibilityMap targetMap : visMaps)
               {
                  if (sourceMap != targetMap)
                  {
                     HashSet<Point3D> targetPoints = targetMap.getVertices();

                     for (Point3D targetPt : targetPoints)
                     {
                        boolean distanceOk = sourcePt.distance(targetPt) < parameters.getMinimumConnectionDistanceForRegions();
                        boolean heightOk = Math.abs(sourcePt.getZ() - targetPt.getZ()) < parameters.getTooHighToStepDistance();
                        if (distanceOk && heightOk)
                        {
                           connectionPoints.add(new Connection(sourcePt, targetPt));
                           globalMapPoints.add(new Connection(sourcePt, targetPt));
                           connectionsAdded++;
                        }
                     }
                  }
               }
            }
         }
      }

      //      System.out.println("Connecting maps added " + connectionsAdded + " connections");

   }

   private void createVisibilityGraphForRegion(PlanarRegion region, Point3D startPos, Point3D goalPos)
   {
      if (debug)
      {
         PrintTools.info("Creating a visibility graph for region with ID:" + region.getRegionId());
      }

      NavigableRegion navigableRegionLocalPlanner = new NavigableRegion(region);
      processRegion(navigableRegionLocalPlanner);
      listOfLocalPlanners.add(navigableRegionLocalPlanner);

      VisibilityMap localVisibilityMapInWorld = new VisibilityMap();
      visMaps.add(localVisibilityMapInWorld);

      for (Connection connection : navigableRegionLocalPlanner.getLocalVisibilityGraph().getConnections())
      {
         Point2D edgeSource = new Point2D(connection.getSourcePoint().getX(), connection.getSourcePoint().getY());
         Point2D edgeTarget = new Point2D(connection.getTargetPoint().getX(), connection.getTargetPoint().getY());

         FramePoint3D pt1 = new FramePoint3D(navigableRegionLocalPlanner.getLocalReferenceFrame(), new Point3D(edgeSource.getX(), edgeSource.getY(), 0));
         pt1.changeFrame(ReferenceFrame.getWorldFrame());
         FramePoint3D pt2 = new FramePoint3D(navigableRegionLocalPlanner.getLocalReferenceFrame(), new Point3D(edgeTarget.getX(), edgeTarget.getY(), 0));
         pt2.changeFrame(ReferenceFrame.getWorldFrame());

         localVisibilityMapInWorld.addConnection(new Connection(pt1.getPoint(), pt2.getPoint()));
      }

      localVisibilityMapInWorld.computeVertices();
   }

   public void processRegion(NavigableRegion navigableRegionLocalPlanner)
   {
      List<PlanarRegion> lineObstacleRegions = new ArrayList<>();
      List<PlanarRegion> polygonObstacleRegions = new ArrayList<>();
      List<PlanarRegion> regionsInsideHomeRegion = new ArrayList<>();
      List<Cluster> clusters = new ArrayList<>();

      regionsInsideHomeRegion = PlanarRegionTools.determineWhichRegionsAreInside(navigableRegionLocalPlanner.getHomeRegion(), regions);

      if (debug)
      {
         System.out.println(regionsInsideHomeRegion.size());
      }

      ClusterTools.classifyExtrusions(regionsInsideHomeRegion, navigableRegionLocalPlanner.getHomeRegion(), lineObstacleRegions, polygonObstacleRegions,
                                      parameters.getNormalZThresholdForPolygonObstacles());
      regionsInsideHomeRegion = PlanarRegionTools.filterRegionsThatAreAboveHomeRegion(regionsInsideHomeRegion, navigableRegionLocalPlanner.getHomeRegion());

      ClusterTools.createClustersFromRegions(navigableRegionLocalPlanner.getHomeRegion(), regionsInsideHomeRegion, lineObstacleRegions, polygonObstacleRegions,
                                             clusters, navigableRegionLocalPlanner.getLocalReferenceFrame().getTransformToWorldFrame(), parameters);
      ClusterTools.createClusterForHomeRegion(clusters, navigableRegionLocalPlanner.getLocalReferenceFrame().getTransformToWorldFrame(),
                                              navigableRegionLocalPlanner.getHomeRegion(), parameters.getExtrusionDistance());

      if (debug)
      {
         System.out.println("Extruding obstacles...");
      }

      // TODO The use of Double.MAX_VALUE for the observer seems rather risky. I'm actually surprised that it works.
      //      clusterMgr.performExtrusions(new Point2D(Double.MAX_VALUE, Double.MAX_VALUE), parameters.getExtrusionDistance());
      ClusterTools.performExtrusions(new Point2D(Double.MAX_VALUE, Double.MAX_VALUE), parameters.getExtrusionDistance(), clusters);

      for (Cluster cluster : clusters)
      {
         PointCloudTools.doBrakeDownOn2DPoints(cluster.getNavigableExtrusionsInLocal(), parameters.getClusterResolution());
      }

      VisibilityMap visibilityMap = new VisibilityMap();
      HashSet<Connection> connectionsForMap = VisibilityTools.createStaticVisibilityMap(null, null, clusters);
      visibilityMap.setConnections(connectionsForMap);

      ArrayList<Connection> connections = new ArrayList<>();

      Iterator it = visibilityMap.getConnections().iterator();

      while (it.hasNext())
      {
         Connection connection = (Connection) it.next();
         connections.add(connection);
      }

      ArrayList<Connection> filteredConnections1 = VisibilityTools.removeConnectionsFromExtrusionsOutsideRegions(connections,
                                                                                                                 navigableRegionLocalPlanner.getHomeRegion());
      ArrayList<Connection> filteredConnections2 = VisibilityTools.removeConnectionsFromExtrusionsInsideNoGoZones(filteredConnections1, clusters);

      HashSet<Connection> sets = new HashSet<>();

      for (Connection connection : filteredConnections2)
      {
         sets.add(connection);
      }

      visibilityMap.setConnections(sets);

      navigableRegionLocalPlanner.setClusters(clusters);
      navigableRegionLocalPlanner.setRegionsInsideHomeRegion(regionsInsideHomeRegion);
      navigableRegionLocalPlanner.setLineObstacleRegions(lineObstacleRegions);
      navigableRegionLocalPlanner.setPolygonObstacleRegions(polygonObstacleRegions);
      navigableRegionLocalPlanner.setVisibilityMap(visibilityMap);
   }

   public boolean isPointInsideNoGoZone(Point3DReadOnly pointToCheck)
   {
      int index = 0;
      for (NavigableRegion localPlanner : listOfLocalPlanners)
      {
         for (Cluster cluster : localPlanner.getClusters())
         {
            if (cluster.getNonNavigableExtrusionsInWorld().size() == 0)
            {
               continue;
            }

            Point2D[] homePointsArr = new Point2D[cluster.getNonNavigableExtrusionsInWorld().size()];
            ArrayList<Point2D> points = new ArrayList<>();
            for (int i = 0; i < cluster.getNonNavigableExtrusionsInWorld().size(); i++)
            {
               Point3D extrusion = cluster.getNonNavigableExtrusionsInWorld().get(i);
               Point2D point2D = new Point2D(extrusion.getX(), extrusion.getY());
               homePointsArr[i] = point2D;
               points.add(point2D);
            }

            Point2D centroid = EuclidGeometryTools.averagePoint2Ds(points);

            Vector2D directionToCentroid = new Vector2D(centroid.getX() - pointToCheck.getX(), centroid.getY() - pointToCheck.getY());
            directionToCentroid.normalize();
            directionToCentroid.scale(10);

            Point2D endPoint = new Point2D(pointToCheck.getX() + directionToCentroid.getX(), pointToCheck.getY() + directionToCentroid.getY());

            if (PlanarRegionTools.isPointInsidePolygon(homePointsArr, new Point2D(pointToCheck.getX(), pointToCheck.getY()), endPoint))
            {
               index++;

               if (index > 1)
               {
                  return true;
               }
            }
         }
      }

      return false;
   }

   public Point3D[][] getNavigableExtrusions()
   {
      Point3D[][] allNavigableExtrusions = new Point3D[listOfLocalPlanners.size()][];

      for (int i = 0; i < listOfLocalPlanners.size(); i++)
      {
         NavigableRegion localPlanner = listOfLocalPlanners.get(i);
         Point3D[] navigableExtrusions = new Point3D[localPlanner.getClusters().size()];

         for (Cluster cluster : localPlanner.getClusters())
         {
            for (int j = 0; j < cluster.getNumberOfNavigableExtrusions(); j++)
            {
               navigableExtrusions[j] = cluster.getNavigableExtrusionInWorld(j);
            }
         }

         allNavigableExtrusions[i] = navigableExtrusions;
      }

      return allNavigableExtrusions;
   }

   public List<NavigableRegion> getListOfLocalPlanners()
   {
      return listOfLocalPlanners;
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

   private class DistancePoint implements Comparable<DistancePoint>
   {
      Point3D point;
      double distance;

      public DistancePoint(Point3D point, double distance)
      {
         this.point = point;
         this.distance = distance;
      }

      @Override
      public int compareTo(DistancePoint point1)
      {
         if (distance > point1.distance)
         {
            return 1;
         }
         else if (distance < point1.distance)
         {
            return -1;
         }
         else
         {
            return 0;
         }
      }
   }

   private class DistancePointComparator implements Comparator<DistancePoint>
   {
      @Override
      public int compare(DistancePoint point1, DistancePoint point2)
      {
         return point1.compareTo(point2);
      }
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
