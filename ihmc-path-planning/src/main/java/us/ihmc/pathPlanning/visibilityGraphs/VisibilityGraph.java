package us.ihmc.pathPlanning.visibilityGraphs;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.InterRegionVisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphNode;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapSolution;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.InterRegionConnectionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityGraph
{
   private static final double minimumDistanceForStartOrGoalEdgeSquared = 0.01 * 0.01;

   private ArrayList<VisibilityGraphNavigableRegion> visibilityGraphNavigableRegions = new ArrayList<>();
   private final NavigableRegions navigableRegions;
   private final ArrayList<VisibilityGraphEdge> crossRegionEdges = new ArrayList<VisibilityGraphEdge>();

   private VisibilityGraphNode startNode, goalNode;
   //   private final ArrayList<VisibilityGraphEdge> startEdges = new ArrayList<>();
   //   private final ArrayList<VisibilityGraphEdge> goalEdges = new ArrayList<>();

   public VisibilityGraph(NavigableRegions navigableRegions, InterRegionConnectionFilter filter)
   {
      this.navigableRegions = navigableRegions;

      List<NavigableRegion> naviableRegionsList = navigableRegions.getNaviableRegionsList();

      for (NavigableRegion navigableRegion : naviableRegionsList)
      {
         VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = new VisibilityGraphNavigableRegion(navigableRegion);
         visibilityGraphNavigableRegion.createGraphAroundClusterRings();
         visibilityGraphNavigableRegion.createGraphBetweenInnerClusterRings();

         visibilityGraphNavigableRegions.add(visibilityGraphNavigableRegion);
      }

      for (int sourceIndex = 0; sourceIndex < visibilityGraphNavigableRegions.size(); sourceIndex++)
      {
         VisibilityGraphNavigableRegion sourceNavigableRegion = visibilityGraphNavigableRegions.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < visibilityGraphNavigableRegions.size(); targetIndex++)
         {
            VisibilityGraphNavigableRegion targetNavigableRegion = visibilityGraphNavigableRegions.get(targetIndex);
            createInterRegionVisibilityConnections(sourceNavigableRegion, targetNavigableRegion, filter, crossRegionEdges);
         }
      }
   }

   public VisibilityGraphNode getStartNode()
   {
      return startNode;
   }

   public VisibilityGraphNode getGoalNode()
   {
      return goalNode;
   }

   public List<VisibilityGraphEdge> getStartEdges()
   {
      if (startNode == null)
         return null;
      return startNode.getEdges();
   }

   public List<VisibilityGraphEdge> getGoalEdges()
   {
      if (goalNode == null)
         return null;
      return goalNode.getEdges();
   }

   public boolean setStart(Point3DReadOnly sourceLocationInWorld, double searchHostEpsilon)
   {
      boolean success = true;

      VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = getVisibilityGraphNavigableRegionContainingThisPoint(sourceLocationInWorld,
                                                                                                                           searchHostEpsilon);
      if (visibilityGraphNavigableRegion == null)
         return false;

      startNode = createStaticVisibilityMap(sourceLocationInWorld, visibilityGraphNavigableRegion, goalNode);
      success = !startNode.getEdges().isEmpty();

      return success;

      //TODO: Need a fallback map...
      //    return connectSourceToHostOrFallbackMap(source, potentialFallbackMap, hostRegion);
   }

   public boolean setGoal(Point3DReadOnly sourceLocationInWorld, double searchHostEpsilon)
   {
      boolean success = true;

      VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = getVisibilityGraphNavigableRegionContainingThisPoint(sourceLocationInWorld,
                                                                                                                           searchHostEpsilon);
      if (visibilityGraphNavigableRegion == null)
         return false;

      goalNode = createStaticVisibilityMap(sourceLocationInWorld, visibilityGraphNavigableRegion, startNode);
      success = !goalNode.getEdges().isEmpty();

      return success;

      //TODO: Need a fallback map...
      //    return connectSourceToHostOrFallbackMap(source, potentialFallbackMap, hostRegion);
   }

   private VisibilityGraphNavigableRegion getVisibilityGraphNavigableRegionContainingThisPoint(Point3DReadOnly sourceLocationInWorld, double searchHostEpsilon)
   {
      NavigableRegion hostNavigableRegion = PlanarRegionTools.getNavigableRegionContainingThisPoint(sourceLocationInWorld, navigableRegions, searchHostEpsilon);
      VisibilityGraphNavigableRegion visibilityGraphNavigableRegion = getVisibilityGraphNavigableRegion(hostNavigableRegion);
      return visibilityGraphNavigableRegion;
   }

   private VisibilityGraphNavigableRegion getVisibilityGraphNavigableRegion(NavigableRegion navigableRegion)
   {
      if (navigableRegion == null)
         return null;

      for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {
         if (visibilityGraphNavigableRegion.getNavigableRegion() == navigableRegion)
            return visibilityGraphNavigableRegion;
      }

      return null;
   }

   public ArrayList<VisibilityGraphNavigableRegion> getVisibilityGraphNavigableRegions()
   {
      return visibilityGraphNavigableRegions;
   }

   public ArrayList<VisibilityGraphEdge> getCrossRegionEdges()
   {
      return crossRegionEdges;
   }

   public static VisibilityGraphNode createStaticVisibilityMap(Point3DReadOnly sourceInWorld, VisibilityGraphNavigableRegion visibilityGraphNavigableRegion,
                                                               VisibilityGraphNode nodeToAttachToIfInSameRegion)
   {
      NavigableRegion navigableRegion = visibilityGraphNavigableRegion.getNavigableRegion();
      Point3D sourceInLocal3D = new Point3D(sourceInWorld);
      navigableRegion.transformFromWorldToLocal(sourceInLocal3D);
      Point2D sourceInLocal = new Point2D(sourceInLocal3D);

      Point3D projectedSourceInWorld = new Point3D(sourceInLocal);
      navigableRegion.transformFromLocalToWorld(projectedSourceInWorld);

      ArrayList<VisibilityGraphNode> homeRegionNodes = visibilityGraphNavigableRegion.getHomeRegionNodes();

      List<Cluster> allClusters = navigableRegion.getAllClusters();

      VisibilityGraphNode sourceNode = new VisibilityGraphNode(projectedSourceInWorld, sourceInLocal, visibilityGraphNavigableRegion.getMapId());

      for (VisibilityGraphNode targetNode : homeRegionNodes)
      {
         Point2DReadOnly targetInLocal = targetNode.getPoint2DInLocal();
         if (sourceInLocal.distanceSquared(targetInLocal) > minimumDistanceForStartOrGoalEdgeSquared)
         {
            boolean targetIsVisible = VisibilityTools.isPointVisibleForStaticMaps(allClusters, sourceInLocal, targetInLocal);

            if (targetIsVisible)
            {
               VisibilityGraphEdge newEdge = new VisibilityGraphEdge(sourceNode, targetNode);
               sourceNode.addEdge(newEdge);
               targetNode.addEdge(newEdge);
            }
         }
      }

      if (nodeToAttachToIfInSameRegion != null)
      {
         if (sourceNode.getRegionId() == nodeToAttachToIfInSameRegion.getRegionId())
         {
            VisibilityGraphEdge edgeFromSourceToAdditionalTarget = new VisibilityGraphEdge(sourceNode, nodeToAttachToIfInSameRegion);
            sourceNode.addEdge(edgeFromSourceToAdditionalTarget);
            nodeToAttachToIfInSameRegion.addEdge(edgeFromSourceToAdditionalTarget);
         }
      }

      return sourceNode;
   }

   //TODO: +++JEP: Get rid of these stats after optimized.
   private static int numberPlanarRegionBoundingBoxesTooFar = 0, totalSourceTargetChecks = 0, numberPassValidFilter = 0, numberNotInsideSourceRegion = 0,
         numberNotInsideTargetRegion = 0, numberSourcesInNoGoZones = 0, numberTargetsInNoGoZones = 0, numberValidConnections = 0;

   public static void createInterRegionVisibilityConnections(VisibilityGraphNavigableRegion sourceNavigableRegion,
                                                             VisibilityGraphNavigableRegion targetNavigableRegion, InterRegionConnectionFilter filter,
                                                             ArrayList<VisibilityGraphEdge> edgesToPack)
   {
      int sourceId = sourceNavigableRegion.getMapId();
      int targetId = targetNavigableRegion.getMapId();

      if (sourceId == targetId)
         return;

      PlanarRegion sourceHomePlanarRegion = sourceNavigableRegion.getNavigableRegion().getHomePlanarRegion();
      PlanarRegion targetHomePlanarRegion = targetNavigableRegion.getNavigableRegion().getHomePlanarRegion();

      BoundingBox3D sourceHomeRegionBoundingBox = sourceHomePlanarRegion.getBoundingBox3dInWorld();
      BoundingBox3D targetHomeRegionBoundingBox = targetHomePlanarRegion.getBoundingBox3dInWorld();

      // If the source and target regions are simply too far apart, then do not check their individual points.
      if (!sourceHomeRegionBoundingBox.intersectsEpsilon(targetHomeRegionBoundingBox, filter.getMaximumInterRegionConnetionDistance()))
      {
         numberPlanarRegionBoundingBoxesTooFar++;
         return;
      }

      List<Cluster> sourceObstacleClusters = sourceNavigableRegion.getNavigableRegion().getObstacleClusters();
      List<Cluster> targetObstacleClusters = targetNavigableRegion.getNavigableRegion().getObstacleClusters();

      ArrayList<VisibilityGraphNode> sourceHomeRegionNodes = sourceNavigableRegion.getHomeRegionNodes();
      ArrayList<VisibilityGraphNode> targetHomeRegionNodes = targetNavigableRegion.getHomeRegionNodes();

      //TODO: +++JEP: Also connect to the obstacle navigable regions when appropriate!!!
      ArrayList<ArrayList<VisibilityGraphNode>> sourceObstacleNodesLists = sourceNavigableRegion.getObstacleNavigableNodes();

      ArrayList<ArrayList<VisibilityGraphNode>> targetObstacleNodesLists = targetNavigableRegion.getObstacleNavigableNodes();

      createInterRegionVisibilityConnections(sourceHomeRegionNodes, targetHomeRegionNodes, sourceObstacleClusters, targetObstacleClusters, filter, edgesToPack);

      //      printStats();

   }

   public static void createInterRegionVisibilityConnections(ArrayList<VisibilityGraphNode> sourceNodeList, ArrayList<VisibilityGraphNode> targetNodeList,
                                                             List<Cluster> sourceObstacleClusters, List<Cluster> targetObstacleClusters,
                                                             InterRegionConnectionFilter filter, ArrayList<VisibilityGraphEdge> edgesToPack)
   {
      for (VisibilityGraphNode sourceNode : sourceNodeList)
      {
         for (VisibilityGraphNode targetNode : targetNodeList)
         {
            totalSourceTargetChecks++;
            if (totalSourceTargetChecks % 10000000 == 0)
            {
               printStats();
            }

            //                  ConnectionPoint3D source = new ConnectionPoint3D(sourcePoint3DInWorld, sourceId);
            //                  ConnectionPoint3D target = new ConnectionPoint3D(targetPoint3D, targetId);

            if (filter.isConnectionValid(sourceNode.getPointInWorld(), targetNode.getPointInWorld()))
            {
               numberPassValidFilter++;

               //                     Point2D sourcePoint2DInLocal = VisibilityGraphsFactory.getPoint2DInLocal(sourceNavigableRegion, sourcePoint3DInWorld);
               Point2DReadOnly sourcePoint2DInLocal = sourceNode.getPoint2DInLocal();

               //                     Point2D targetPoint2DInLocal = VisibilityGraphsFactory.getPoint2DInLocal(targetNavigableRegion, targetPoint3D);
               Point2DReadOnly targetPoint2DInLocal = targetNode.getPoint2DInLocal();

               boolean sourceIsInsideNoGoZone = VisibilityGraphsFactory.isInsideANonNavigableZone(sourcePoint2DInLocal, sourceObstacleClusters);
               if (sourceIsInsideNoGoZone)
               {
                  numberSourcesInNoGoZones++;
                  continue;
               }

               boolean targetIsInsideNoGoZone = VisibilityGraphsFactory.isInsideANonNavigableZone(targetPoint2DInLocal, targetObstacleClusters);
               if (targetIsInsideNoGoZone)
               {
                  numberTargetsInNoGoZones++;
                  continue;
               }

               numberValidConnections++;

               VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
               sourceNode.addEdge(edge);
               targetNode.addEdge(edge);

               edgesToPack.add(edge);
            }
         }
      }
   }

   private static void printStats()
   {
      System.out.println("numberPlanarRegionBoundingBoxesTooFar = " + numberPlanarRegionBoundingBoxesTooFar);
      System.out.println("totalSourceTargetChecks = " + totalSourceTargetChecks);
      System.out.println("numberPassValidFilter = " + numberPassValidFilter);
      System.out.println("numberNotInsideSourceRegion = " + numberNotInsideSourceRegion);
      System.out.println("numberNotInsideTargetRegion = " + numberNotInsideTargetRegion);
      System.out.println("numberSourcesInNoGoZones = " + numberSourcesInNoGoZones);
      System.out.println("numberTargetsInNoGoZones = " + numberTargetsInNoGoZones);
      System.out.println("numberValidConnections = " + numberValidConnections);

      System.out.println();
   }

   public VisibilityMapSolution createVisibilityMapSolution()
   {
      VisibilityMapSolution solution = new VisibilityMapSolution();

      solution.setNavigableRegions(navigableRegions);

      ArrayList<VisibilityMapWithNavigableRegion> visibilityMapsWithNavigableRegions = new ArrayList<>();

      for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : visibilityGraphNavigableRegions)
      {

         NavigableRegion navigableRegion = visibilityGraphNavigableRegion.getNavigableRegion();
         ArrayList<VisibilityGraphNode> homeRegionNodes = visibilityGraphNavigableRegion.getHomeRegionNodes();
         ArrayList<ArrayList<VisibilityGraphNode>> obstacleNavigableNodes = visibilityGraphNavigableRegion.getObstacleNavigableNodes();
         ArrayList<VisibilityGraphEdge> allEdges = visibilityGraphNavigableRegion.getAllEdges();

         VisibilityMapWithNavigableRegion visibilityMapWithNavigableRegion = new VisibilityMapWithNavigableRegion(navigableRegion);

         Collection<Connection> connections = new ArrayList<Connection>();

         for (VisibilityGraphEdge edge : allEdges)
         {
            ConnectionPoint3D sourcePoint = edge.getSourcePoint();
            ConnectionPoint3D targetPoint = edge.getTargetPoint();
            connections.add(new Connection(sourcePoint, targetPoint));
         }

         VisibilityMap visibilityMapInWorld = new VisibilityMap(connections);
         visibilityMapWithNavigableRegion.setVisibilityMapInWorld(visibilityMapInWorld);
         visibilityMapsWithNavigableRegions.add(visibilityMapWithNavigableRegion);
      }

      solution.setVisibilityMapsWithNavigableRegions(visibilityMapsWithNavigableRegions);

      InterRegionVisibilityMap interRegionVisibilityMap = new InterRegionVisibilityMap();

      for (VisibilityGraphEdge crossRegionEdge : crossRegionEdges)
      {
         Connection connection = new Connection(crossRegionEdge.getSourcePoint(), crossRegionEdge.getTargetPoint());
         interRegionVisibilityMap.addConnection(connection);
      }

      solution.setInterRegionVisibilityMap(interRegionVisibilityMap);
      return solution;
   }

}
