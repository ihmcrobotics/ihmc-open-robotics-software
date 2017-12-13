package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.OcclussionTools;
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
   private SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);

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
      regions = PlanarRegionTools.filterPlanarRegionsByArea(parameters.getPlanarRegionMinArea(), regions);
      regions = PlanarRegionTools.filterPlanarRegionsByHullSize(parameters.getPlanarRegionMinSize(), regions);

      this.regions = regions;
   }

   public void createIndividualVisMapsForRegions()
   {
      listOfLocalPlanners.clear();
      visMaps.clear();
      accesibleRegions.clear();

      PlanarRegionTools.classifyRegions(regions, parameters.getNormalZThresholdForAccessibleRegions(), obstacleRegions, accesibleRegions);

      for (PlanarRegion region : accesibleRegions)
      {
         createVisibilityGraphForRegion(region, null, null);
      }

   }

   public List<Point3DReadOnly> calculateBodyPath(Point3D start, Point3D goal)
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

      createIndividualVisMapsForRegions();

      long endCreationTime = System.currentTimeMillis();

      boolean readyToRunBodyPath = false;
      //      if (!OcclussionTools.IsTheGoalIntersectingAnyObstacles(listOfLocalPlanners.get(0), start, goal))
      //      {
      //         readyToRunBodyPath = true;
      //         globalMapPoints.add(new Connection(start, goal));
      //      }
      //      else
      //      {
      if (startProjected != null && goalProjected != null)
      {
         readyToRunBodyPath = createVisMapsForStartAndGoal(startProjected, goalProjected);
      }
      //      }
      
      createGlobalMapFromAlltheLocalMaps();
      long startConnectingTime = System.currentTimeMillis();
      connectLocalMaps();
      long endConnectingTime = System.currentTimeMillis();

      if (readyToRunBodyPath)
      {
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
         Point3DReadOnly snappedGoalPosition = getSnappedPointFromVisibilityGraph(goal);
         if (debug && snappedGoalPosition == null)
         {
            PrintTools.error("Snapping of goal returned null.");
         }
         Point3DReadOnly snappedStartPosition = getSnappedPointFromVisibilityGraph(start);
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

         List<Point3DReadOnly> path = null;
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

   public List<Point3DReadOnly> calculateBodyPathWithOcclussions(Point3D start, Point3D goal)
   {
      List<Point3DReadOnly> path = calculateBodyPath(start, goal);

      if (path == null)
      {
         if (!OcclussionTools.IsTheGoalIntersectingAnyObstacles(listOfLocalPlanners.get(0), start, goal))
         {
            System.out.println("StraightLine available");

            path = new ArrayList<>();
            path.add(new Point3D(start));
            path.add(goal);

            return path;
         }

         NavigableRegion regionContainingPoint = PlanarRegionTools.getNavigableRegionContainingThisPoint(start, listOfLocalPlanners);
         List<Cluster> intersectingClusters = OcclussionTools.getListOfIntersectingObstacles(regionContainingPoint.getClusters(), start, goal);
         Cluster closestCluster = ClusterTools.getTheClosestCluster(start, intersectingClusters);
         Point3D closestExtrusion = ClusterTools.getTheClosestVisibleExtrusionPoint(1.0, start, goal, closestCluster.getNavigableExtrusionsInWorld(),
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

   public boolean createVisMapsForStartAndGoal(Point3DReadOnly startProjected, Point3DReadOnly goalProjected)
   {
      NavigableRegion startRegion = null;
      Point2D startInRegionFrame = null;

      for (NavigableRegion navigableRegion : listOfLocalPlanners)
      {
         FramePoint3D frameStart = new FramePoint3D(ReferenceFrame.getWorldFrame(), startProjected);
         frameStart.changeFrame(navigableRegion.getLocalReferenceFrame());

         if (PlanarRegionTools.isPointInLocalInsideARegion(navigableRegion.getHomeRegion(), frameStart))
         {
            startInRegionFrame = new Point2D(frameStart);
            startRegion = navigableRegion;
            break;
         }
      }

      NavigableRegion goalRegion = null;
      Point2D goalInRegionFrame = null;

      for (NavigableRegion navigableRegion : listOfLocalPlanners)
      {
         FramePoint3D frameGoal = new FramePoint3D(ReferenceFrame.getWorldFrame(), goalProjected);
         frameGoal.changeFrame(navigableRegion.getLocalReferenceFrame());

         if (PlanarRegionTools.isPointInLocalInsideARegion(navigableRegion.getHomeRegion(), frameGoal))
         {
            goalInRegionFrame = new Point2D(frameGoal);
            goalRegion = navigableRegion;
            break;
         }
      }

      VisibilityMap startMap = createVisMapForSinglePointSource(startInRegionFrame, startRegion);
      VisibilityMap goalMap = createVisMapForSinglePointSource(goalInRegionFrame, goalRegion);

      if (startRegion == goalRegion)
      {
         boolean targetIsVisible = VisibilityTools.isPointVisibleForStaticMaps(startRegion.getClusters(), startInRegionFrame, goalInRegionFrame);
         if (targetIsVisible)
         {
            globalMapPoints.add(new Connection(startProjected, goalProjected));
         }
      }

      if (startMap.isEmpty() || goalMap.isEmpty())
         return false;

      visMaps.add(startMap);
      visMaps.add(goalMap);
      return true;
   }

   private List<Point3DReadOnly> calculatePathOnVisibilityGraph(Point3DReadOnly start, Point3DReadOnly goal)
   {
      List<DefaultWeightedEdge> solution = DijkstraShortestPath.findPathBetween(globalVisMap, new ConnectionPoint3D(start), new ConnectionPoint3D(goal));
      return convertVisibilityGraphSolutionToPath(solution, start);
   }

   private List<Point3DReadOnly> convertVisibilityGraphSolutionToPath(List<DefaultWeightedEdge> solution, Point3DReadOnly start)
   {
      List<Point3DReadOnly> path = new ArrayList<>();
      pathLength = 0.0;
      path.clear();

      if (solution != null)
      {
         for (DefaultWeightedEdge edge : solution)
         {
            Point3DReadOnly from = globalVisMap.getEdgeSource(edge);
            Point3DReadOnly to = globalVisMap.getEdgeTarget(edge);
            pathLength = pathLength + from.distance(to);

            if (!path.contains(new Point3D(from)))
               path.add(from);
            if (!path.contains(new Point3D(to)))
               path.add(to);
         }

         if (!path.get(0).epsilonEquals(start, 1e-5))
         {
            Point3DReadOnly pointOut = path.get(1);
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
         ConnectionPoint3D pt1 = pair.getSourcePoint();
         ConnectionPoint3D pt2 = pair.getTargetPoint();

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

   public static VisibilityMap createVisMapForSinglePointSource(Point2DReadOnly point, NavigableRegion navigableRegion)
   {
      Set<Connection> connections = VisibilityTools.createStaticVisibilityMap(point, navigableRegion.getClusters());

      RigidBodyTransform transformToWorld = navigableRegion.getLocalReferenceFrame().getTransformToWorldFrame();

      VisibilityMap mapForSingleObserver = new VisibilityMap();
      mapForSingleObserver.setConnections(connections);
      mapForSingleObserver.applyTransform(transformToWorld);
      mapForSingleObserver.computeVertices();
      return mapForSingleObserver;
   }

   public NavigableRegion getTheRegionContainingThePoint(Point3D point, Point2D pointToPack)
   {
      for (NavigableRegion navigableRegion : listOfLocalPlanners)
      {
         FramePoint3D pointFpt = new FramePoint3D(ReferenceFrame.getWorldFrame(), point);
         pointFpt.changeFrame(navigableRegion.getLocalReferenceFrame());

         if (PlanarRegionTools.isPointInLocalInsideARegion(navigableRegion.getHomeRegion(), pointFpt.getPoint()))
         {
            pointToPack = new Point2D(pointFpt.getPoint().getX(), pointFpt.getPoint().getY());
            System.out.println(pointToPack);
            return navigableRegion;
         }
      }
      return null;
   }

   public Point3DReadOnly getSnappedPointFromVisibilityGraph(Point3D position)
   {
      for (DefaultWeightedEdge edge : globalVisMap.edgeSet())
      {
         Point3DReadOnly source = globalVisMap.getEdgeSource(edge);

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

         Point3DReadOnly target = globalVisMap.getEdgeTarget(edge);

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

      ArrayList<Point3DReadOnly> filteredList = new ArrayList<>();

      for (DistancePoint point : distancePoints)
      {
         //         System.out.println("Adding connection "+ point.point + "  to " + position);

         filteredList.add(point.point);
      }

      //      System.out.println("After filtering: " + filteredList.size());

      Iterator<Point3DReadOnly> it = filteredList.iterator();

      Point3DReadOnly pointToSnapTo = null;
      while (it.hasNext())
      {
         Point3DReadOnly source = it.next();
         Point3DReadOnly target = it.next();

         //Cannot add an edge where the source is equal to the target!
         if (!source.epsilonEquals(desiredPointToSnap, 1e-5))
         {
            pointToSnapTo = target;
            //            System.out.println("-----> new snapped point: " + pointToSnapTo);
            connectionPoints.add(new Connection(pointToSnapTo, desiredPointToSnap));
            break;
         }
      }

      return new Point3D(pointToSnapTo);
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

      ArrayList<Point3DReadOnly> listOfOrderedPoints = new ArrayList<>();

      for (DistancePoint point : distancePoints)
      {
         listOfOrderedPoints.add(point.point);
      }

      Iterator<Point3DReadOnly> it = listOfOrderedPoints.iterator();

      int index = 0;
      while (it.hasNext() && index < parameters.getNumberOfForcedConnections())
      {
         Point3DReadOnly closestPoint = it.next();

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
            Set<Point3DReadOnly> sourcePoints = sourceMap.getVertices();

            for (Point3DReadOnly sourcePt : sourcePoints)
            {
               for (VisibilityMap targetMap : visMaps)
               {
                  if (sourceMap != targetMap)
                  {
                     Set<Point3DReadOnly> targetPoints = targetMap.getVertices();

                     for (Point3DReadOnly targetPt : targetPoints)
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

      Collection<Connection> connectionsForMap = VisibilityTools.createStaticVisibilityMap(null, null, clusters);

      connectionsForMap = VisibilityTools.removeConnectionsFromExtrusionsOutsideRegions(connectionsForMap, navigableRegionLocalPlanner.getHomeRegion());
      connectionsForMap = VisibilityTools.removeConnectionsFromExtrusionsInsideNoGoZones(connectionsForMap, clusters);

      VisibilityMap visibilityMap = new VisibilityMap();
      visibilityMap.setConnections(connectionsForMap);

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
            cluster.getNonNavigableExtrusionsInWorld().toArray(homePointsArr);

            if (PlanarRegionTools.isPointInsidePolygon(homePointsArr, new Point2D(pointToCheck)))
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
      Point3DReadOnly point;
      double distance;

      public DistancePoint(Point3DReadOnly point, double distance)
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
