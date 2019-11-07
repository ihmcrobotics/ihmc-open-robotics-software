package us.ihmc.pathPlanning.visibilityGraphs;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.*;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.NavigableRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.OcclusionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.*;
import java.util.stream.Collectors;

public class NavigableRegionsManager
{
   private final static boolean debug = false;
   private final static boolean fullyExpandVisibilityGraph = false;

   private final VisibilityGraphsParametersReadOnly parameters;

   private final BodyPathPostProcessor postProcessor;

   private final VisibilityMapSolution visibilityMapSolution = new VisibilityMapSolution();

   private VisibilityGraph visibilityGraph;
   private VisibilityGraphNode startNode;
   private VisibilityGraphNode goalNode;
   private VisibilityGraphNode endNode;
   private Point3DReadOnly goalInWorld;
   private PriorityQueue<VisibilityGraphNode> stack;
   private HashSet<VisibilityGraphNode> expandedNodes;

   public NavigableRegionsManager()
   {
      this(null, null, null);
   }

   public NavigableRegionsManager(VisibilityGraphsParametersReadOnly parameters)
   {
      this(parameters, null, null);
   }

   public NavigableRegionsManager(List<PlanarRegion> regions)
   {
      this(null, regions, null);
   }

   public NavigableRegionsManager(VisibilityGraphsParametersReadOnly parameters, List<PlanarRegion> regions)
   {
      this(parameters, regions, null);
   }

   public NavigableRegionsManager(VisibilityGraphsParametersReadOnly parameters, List<PlanarRegion> regions, BodyPathPostProcessor postProcessor)
   {
      visibilityMapSolution.setNavigableRegions(new NavigableRegions(parameters, regions));
      this.parameters = parameters == null ? new DefaultVisibilityGraphParameters() : parameters;
      this.postProcessor = postProcessor;
   }

   private static ArrayList<VisibilityMapWithNavigableRegion> createListOfVisibilityMapsWithNavigableRegions(NavigableRegions navigableRegions)
   {
      ArrayList<VisibilityMapWithNavigableRegion> list = new ArrayList<>();

      List<NavigableRegion> naviableRegionsList = navigableRegions.getNaviableRegionsList();

      for (NavigableRegion navigableRegion : naviableRegionsList)
      {
         VisibilityMapWithNavigableRegion visibilityMapWithNavigableRegion = new VisibilityMapWithNavigableRegion(navigableRegion);
         list.add(visibilityMapWithNavigableRegion);
      }

      return list;
   }

   public List<VisibilityMapWithNavigableRegion> getNavigableRegionsList()
   {
      return visibilityMapSolution.getVisibilityMapsWithNavigableRegions();
   }

   public void setPlanarRegions(List<PlanarRegion> planarRegions)
   {
      visibilityMapSolution.getNavigableRegions().setPlanarRegions(planarRegions);
   }

   public List<Point3DReadOnly> calculateBodyPath(final Point3DReadOnly start, final Point3DReadOnly goal)
   {
      return calculateBodyPath(start, goal, fullyExpandVisibilityGraph, false);
   }

   public List<Point3DReadOnly> calculateBodyPath(final Point3DReadOnly start,
                                                  final Point3DReadOnly goal,
                                                  boolean fullyExpandVisibilityGraph,
                                                  boolean accommodateOcclusions)
   {
      return calculateVisibilityMapWhileFindingPath(start, goal, fullyExpandVisibilityGraph, accommodateOcclusions);
   }

   private List<Point3DReadOnly> calculateVisibilityMapWhileFindingPath(Point3DReadOnly startInWorld, Point3DReadOnly finalGoalInWorld,
                                                                        boolean fullyExpandVisibilityGraph, boolean accommodateOcclusions)
   {
      if (!initialize(startInWorld, finalGoalInWorld, fullyExpandVisibilityGraph, accommodateOcclusions))
         return null;

      // here, we modify the goal if there is clearly no path to the goal i.e. goal or path to goal is occluded
      if (accommodateOcclusions)
      {
         /**
          * try to plan to final goal once. If does not reach the goal:
          * plan to the closest free/escape node to the goal
          * here, a free/escape node is defined as one on the edge of the navigable area but not next to an obstacle
          * Other options:
          * - Keep track of where we've travelled
          * - employ actual maze solving
          */

         List<Point3DReadOnly> plan = bestEffortPlanToGoal(startInWorld, finalGoalInWorld);

         if (plan.isEmpty() || !plan.get(plan.size() - 1).geometricallyEquals(finalGoalInWorld, 1e-6))
         {
            if (!initialize(startInWorld, finalGoalInWorld, fullyExpandVisibilityGraph, accommodateOcclusions))
               return null;

            // plan again, this time only to closest free edge
            ArrayList<VisibilityGraphNode> homeRegionNodes = new ArrayList<>();
            for (VisibilityGraphNavigableRegion visibilityGraphNavigableRegion : visibilityGraph.getVisibilityGraphNavigableRegions())
            {
               for (VisibilityGraphNode homeRegionNode : visibilityGraphNavigableRegion.getHomeRegionNodes())
               {
                  homeRegionNodes.add(homeRegionNode);
               }
            }

            if (homeRegionNodes.isEmpty())
            {
               return null; // trapped!
            }

            VisibilityGraphNode closestHomeRegionNodeToGoal = homeRegionNodes.get(0);
            for (VisibilityGraphNode visibilityGraphNode : homeRegionNodes)
            {
               if (visibilityGraphNode.getPointInWorld().distance(finalGoalInWorld) < closestHomeRegionNodeToGoal.getPointInWorld().distance(finalGoalInWorld))
               {
                  closestHomeRegionNodeToGoal = visibilityGraphNode;
               }
            }

            plan = bestEffortPlanToGoal(startInWorld, closestHomeRegionNodeToGoal.getPointInWorld());
            return plan;
         }
         else
         {
            return plan;
         }
      }
      else
      {
         return bestEffortPlanToGoal(startInWorld, finalGoalInWorld);
      }
   }

   private List<Point3DReadOnly> bestEffortPlanToGoal(Point3DReadOnly startInWorld, Point3DReadOnly finalGoalInWorld)
   {
      if (!resetStackForStartAndGoal(startInWorld, finalGoalInWorld))
         return null;

      return planInternal();
   }

   private boolean initialize(Point3DReadOnly startInWorld, Point3DReadOnly finalGoalInWorld, boolean fullyExpandVisibilityGraph, boolean accommodateOcclusions)
   {
      if (!checkIfStartAndGoalAreValid(startInWorld, finalGoalInWorld))
         return false;

      NavigableRegions navigableRegions = visibilityMapSolution.getNavigableRegions();

      // filterPlanarRegionsWithBoundingCapsule is a chicken or the egg problem with occlusions
      // maybe bound it instead by surrounding area or different shape?
      if (!accommodateOcclusions)
      {
         navigableRegions.filterPlanarRegionsWithBoundingCapsule(startInWorld, goalInWorld, parameters.getExplorationDistanceFromStartGoal());
      }

      navigableRegions.createNavigableRegions(); // big deal; does a lot of computation and finds obstacles

      visibilityGraph = new VisibilityGraph(navigableRegions, parameters.getInterRegionConnectionFilter(),
                                            parameters.getPreferredToPreferredInterRegionConnectionFilter(),
                                            parameters.getPreferredToNonPreferredInterRegionConnectionFilter(),
                                            parameters);

      if (fullyExpandVisibilityGraph)
         visibilityGraph.fullyExpandVisibilityGraph();

      return true;
   }

   private boolean resetStackForStartAndGoal(Point3DReadOnly startInWorld, Point3DReadOnly goalInWorld)
   {
      double searchHostEpsilon = parameters.getSearchHostRegionEpsilon();
      startNode = visibilityGraph.setStart(startInWorld, parameters.getCanDuckUnderHeight(), searchHostEpsilon);
      goalNode = visibilityGraph.setGoal(goalInWorld, parameters.getCanDuckUnderHeight(), searchHostEpsilon);
      endNode = null;
      this.goalInWorld = goalInWorld;

      if ((startNode == null) || (goalNode == null))
      {
         LogTools.info("startNode or goalNode are null. startNode = {}. goalNode = {}", startNode, goalNode);
         return false;
      }

      PathNodeComparator comparator = new PathNodeComparator();
      stack = new PriorityQueue<>(comparator);

      startNode.setEdgesHaveBeenDetermined(true);
      startNode.setCostFromStart(0.0, null);
      startNode.setEstimatedCostToGoal(startInWorld.distanceXY(goalInWorld));
      stack.add(startNode);
      expandedNodes = new HashSet<>();
      return true;
   }

   /**
    * @return plan as 3D waypoints, never null
    */
   private List<Point3DReadOnly> planInternal()
   {
      long startBodyPathComputation = System.currentTimeMillis();
      long expandedNodesCount = 0;
      long iterations = 0;

      while (!stack.isEmpty())
      {
         iterations++;

         VisibilityGraphNode nodeToExpand = stack.poll();
         if (expandedNodes.contains(nodeToExpand))
            continue;
         expandedNodes.add(nodeToExpand);

         if (checkAndHandleNodeAtGoal(nodeToExpand))
            break;

         checkAndHandleBestEffortNode(nodeToExpand);

         List<VisibilityGraphEdge> neighboringEdges = expandNode(visibilityGraph, nodeToExpand);
         expandedNodesCount += neighboringEdges.size();

         // A* using XY distance heuristic
         for (VisibilityGraphEdge neighboringEdge : neighboringEdges)
         {
            VisibilityGraphNode neighbor = getNeighborNode(nodeToExpand, neighboringEdge);

            double connectionCost = computeEdgeCost(nodeToExpand, neighbor, neighboringEdge.getEdgeWeight(), neighboringEdge.getStaticEdgeCost());
            double newCostFromStart = nodeToExpand.getCostFromStart() + connectionCost;

            double currentCostFromStart = neighbor.getCostFromStart();

            if (Double.isNaN(currentCostFromStart) || (newCostFromStart < currentCostFromStart))
            {
               neighbor.setCostFromStart(newCostFromStart, nodeToExpand);

               double heuristicCost = parameters.getHeuristicWeight() * neighbor.getPointInWorld().distanceXY(goalInWorld);
               neighbor.setEstimatedCostToGoal(heuristicCost);

               // FIXME is this check necessary?
               stack.remove(neighbor);
               stack.add(neighbor);
            }
         }
      }

      VisibilityMapSolution visibilityMapSolutionFromNewVisibilityGraph = visibilityGraph.createVisibilityMapSolution();

      visibilityMapSolution.setVisibilityMapsWithNavigableRegions(visibilityMapSolutionFromNewVisibilityGraph.getVisibilityMapsWithNavigableRegions());
      visibilityMapSolution.setInterRegionVisibilityMap(visibilityMapSolutionFromNewVisibilityGraph.getInterRegionVisibilityMap());

      visibilityMapSolution.setStartMap(visibilityMapSolutionFromNewVisibilityGraph.getStartMap());
      visibilityMapSolution.setGoalMap(visibilityMapSolutionFromNewVisibilityGraph.getGoalMap());

      List<VisibilityGraphNode> nodePath = new ArrayList<>();
      VisibilityGraphNode nodeWalkingBack = endNode;

      while (nodeWalkingBack != null)
      {
         nodePath.add(nodeWalkingBack);
         nodeWalkingBack = nodeWalkingBack.getBestParentNode();
      }
      Collections.reverse(nodePath);

      List<Point3DReadOnly> path;
      if (postProcessor != null)
      {
         path = postProcessor.computePathFromNodes(nodePath, visibilityMapSolution); // modifies and/or adds waypoints
      }
      else
      {
         path = nodePath.stream().map(node -> new Point3D(node.getPointInWorld())).collect(Collectors.toList());
      }

      printResults(startBodyPathComputation, expandedNodesCount, iterations, path);
      return path;
   }

   /**
    * This computes edge costs.
    */
   List<VisibilityGraphEdge> expandNode(VisibilityGraph visibilityGraph, VisibilityGraphNode nodeToExpand)
   {
      if (nodeToExpand.getHasBeenExpanded())
      {
         throw new RuntimeException("Node has already been expanded!!");
      }

      if (!nodeToExpand.getEdgesHaveBeenDetermined()) // compute edge costs
      {
         visibilityGraph.computeInnerAndInterEdges(nodeToExpand);
      }

      nodeToExpand.setHasBeenExpanded(true);

      return nodeToExpand.getEdges();
   }

   private double computeEdgeCost(VisibilityGraphNode nodeToExpandInWorld, VisibilityGraphNode nextNodeInWorld, double edgeWeight, double staticEdgeCost)
   {
      Point3DReadOnly originPointInWorld = nodeToExpandInWorld.getPointInWorld();
      Point3DReadOnly nextPointInWorld = nextNodeInWorld.getPointInWorld();

      double horizontalDistance = originPointInWorld.distanceXY(nextPointInWorld);
      if (horizontalDistance <= 0.0)
         return 0.0;

      double verticalDistance = Math.abs(nextPointInWorld.getZ() - originPointInWorld.getZ());

      double angle = Math.atan(verticalDistance / horizontalDistance);

      double distanceCost = parameters.getDistanceWeight() * horizontalDistance;
      // 2/pi to scale error from {0:90} degrees or {0:pi/2} radians degrees to {0:1}
      double elevationCost = parameters.getElevationWeight() * angle * (2.0 / Math.PI);

      return edgeWeight * distanceCost + elevationCost + staticEdgeCost;
   }

   private boolean checkIfStartAndGoalAreValid(Point3DReadOnly start, Point3DReadOnly goal)
   {
      boolean areStartAndGoalValid = true;
      if (start == null)
      {
         LogTools.error("Start is null!");
         areStartAndGoalValid = false;
      }

      if (goal == null)
      {
         LogTools.error("Goal is null!");
         areStartAndGoalValid = false;
      }

      if (debug)
         LogTools.info("Starting to calculate body path");

      return areStartAndGoalValid;
   }

   private boolean checkAndHandleNodeAtGoal(VisibilityGraphNode nodeToExpand)
   {
      if (nodeToExpand == goalNode)
      {
         endNode = nodeToExpand;
         return true;
      }
      return false;
   }

   private void checkAndHandleBestEffortNode(VisibilityGraphNode nodeToExpand)
   {
      if (!parameters.returnBestEffortSolution())
         return;

      if (nodeToExpand == startNode)
         return;

      double expandedTotalCost = nodeToExpand.getCostFromStart() + nodeToExpand.getEstimatedCostToGoal();
      if (endNode == null || expandedTotalCost < endNode.getCostFromStart() + endNode.getEstimatedCostToGoal())
      {
         endNode = nodeToExpand;
      }
   }

   private VisibilityGraphNode getNeighborNode(VisibilityGraphNode nodeToExpand, VisibilityGraphEdge neighboringEdge)
   {
      VisibilityGraphNode nextNode = null;

      VisibilityGraphNode sourceNode = neighboringEdge.getSourceNode();
      VisibilityGraphNode targetNode = neighboringEdge.getTargetNode();

      if (nodeToExpand == sourceNode)
      {
         nextNode = targetNode;
      }
      else if (nodeToExpand == targetNode)
      {
         nextNode = sourceNode;
      }

      return nextNode;
   }

   private void printResults(long startBodyPathComputation, long expandedNodesCount, long iterationCount, List<? extends Point3DReadOnly> path)
   {
      if (debug)
      {
         if (path != null)
         {
            LogTools.info("Total time to find solution was: " + (System.currentTimeMillis() - startBodyPathComputation) + "ms");
         }
         else
         {
            LogTools.info("NO BODY PATH SOLUTION WAS FOUND!" + (System.currentTimeMillis() - startBodyPathComputation) + "ms");
         }
         LogTools.info("Number of iterations: " + iterationCount);
         LogTools.info("Number of nodes expanded: " + expandedNodesCount);
      }
   }

   @Deprecated
   public List<Point3DReadOnly> calculateBodyPathWithOcclusions(Point3DReadOnly start, Point3DReadOnly goal)
   {
      List<Point3DReadOnly> path = calculateBodyPath(start, goal, fullyExpandVisibilityGraph, false);

      if (path == null)
      {
         NavigableRegions navigableRegions = visibilityMapSolution.getNavigableRegions();
         ArrayList<VisibilityMapWithNavigableRegion> visibilityMapsWithNavigableRegions = createListOfVisibilityMapsWithNavigableRegions(navigableRegions);

         if (!OcclusionTools.isTheGoalIntersectingAnyObstacles(visibilityMapsWithNavigableRegions.get(0), start, goal))
         {
            if (debug)
            {
               LogTools.info("StraightLine available");
            }

            path = new ArrayList<>();
            path.add(start);
            path.add(goal);

            return path;
         }

         NavigableRegion regionContainingPoint = NavigableRegionTools.getNavigableRegionContainingThisPoint(start, navigableRegions, parameters.getCanDuckUnderHeight());
         List<Cluster> intersectingClusters = OcclusionTools.getListOfIntersectingObstacles(regionContainingPoint.getObstacleClusters(), start, goal);
         Cluster closestCluster = ClusterTools.getTheClosestCluster(start, intersectingClusters);

         Point3D closestExtrusion = ClusterTools.getTheClosestVisibleExtrusionPoint(1.0,
                                                                                    start,
                                                                                    goal,
                                                                                    closestCluster.getNavigableExtrusionsInWorld(),
                                                                                    regionContainingPoint.getHomePlanarRegion());

         path = calculateBodyPath(start, closestExtrusion, fullyExpandVisibilityGraph, false);
         path.add(goal);

         return path;
      }
      else
      {
         return path;
      }
   }

   public VisibilityGraph getVisibilityGraph()
   {
      return visibilityGraph;
   }

   public VisibilityMapSolution getVisibilityMapSolution()
   {
      return visibilityMapSolution;
   }

   public VisibilityMapHolder getStartMap()
   {
      return visibilityMapSolution.getStartMap();
   }

   public VisibilityMapHolder getGoalMap()
   {
      return visibilityMapSolution.getGoalMap();
   }

   public InterRegionVisibilityMap getInterRegionConnections()
   {
      return visibilityMapSolution.getInterRegionVisibilityMap();
   }

}
