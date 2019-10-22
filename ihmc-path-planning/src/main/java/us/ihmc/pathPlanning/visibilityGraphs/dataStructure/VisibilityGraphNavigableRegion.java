package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class VisibilityGraphNavigableRegion
{
   private final NavigableRegion navigableRegion;

   private final ArrayList<VisibilityGraphNode> preferredHomeRegionNodes = new ArrayList<>();
   private final ArrayList<VisibilityGraphNode> homeRegionNodes = new ArrayList<>();
   private final ArrayList<List<VisibilityGraphNode>> obstacleNavigableNodes = new ArrayList<>();
   private final ArrayList<List<VisibilityGraphNode>> obstaclePreferredNavigableNodes = new ArrayList<>();
   private final ArrayList<VisibilityGraphNode> intermediateObstacleNodes = new ArrayList<>();

   private final ArrayList<VisibilityGraphEdge> innerRegionEdges = new ArrayList<>();

   public VisibilityGraphNavigableRegion(NavigableRegion navigableRegion)
   {
      this.navigableRegion = navigableRegion;
   }

   public NavigableRegion getNavigableRegion()
   {
      return navigableRegion;
   }

   public int getMapId()
   {
      return navigableRegion.getMapId();
   }

   public List<VisibilityGraphNode> getHomeRegionNodes()
   {
      return homeRegionNodes;
   }

   public List<VisibilityGraphNode> getAllNavigableNodes()
   {
      //TODO: Store them all in one list instead of several to not require this???
      ArrayList<VisibilityGraphNode> allNavigableNodes = new ArrayList<>();
      allNavigableNodes.addAll(homeRegionNodes);

      for (List<VisibilityGraphNode> obstacleNodes : obstacleNavigableNodes)
      {
         allNavigableNodes.addAll(obstacleNodes);
      }

      return allNavigableNodes;
   }

   public List<VisibilityGraphNode> getAllPreferredNavigableNodes()
   {
      //TODO: Store them all in one list instead of several to not require this???
      ArrayList<VisibilityGraphNode> allNavigableNodes = new ArrayList<>();
      allNavigableNodes.addAll(preferredHomeRegionNodes);

      for (List<VisibilityGraphNode> obstacleNodes : obstaclePreferredNavigableNodes)
      {
         allNavigableNodes.addAll(obstacleNodes);
      }

      return allNavigableNodes;
   }

   public void addIntermediateObstacleNode(VisibilityGraphNode node)
   {
      intermediateObstacleNodes.add(node);
   }

   public List<List<VisibilityGraphNode>> getObstaclePreferredNavigableNodes()
   {
      return obstaclePreferredNavigableNodes;
   }

   public List<List<VisibilityGraphNode>> getObstacleNavigableNodes()
   {
      return obstacleNavigableNodes;
   }

   public List<VisibilityGraphEdge> getAllEdges()
   {
      return innerRegionEdges;
   }

   public void addInnerRegionEdge(VisibilityGraphNode sourceNode, VisibilityGraphNode targetNode)
   {
      addInnerRegionEdge(sourceNode, targetNode, 1.0, 0.0);
   }

   public void addInnerRegionEdge(VisibilityGraphNode sourceNode, VisibilityGraphNode targetNode, double weight, double staticCost)
   {
      VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
      edge.setEdgeWeight(weight);
      edge.setStaticEdgeCost(staticCost);
      sourceNode.addEdge(edge);
      targetNode.addEdge(edge);
      innerRegionEdges.add(edge);
   }

   public void createNavigableRegionNodes(boolean createEdgesAroundClusterRing)
   {
      PlanarRegion homePlanarRegion = navigableRegion.getHomePlanarRegion();
      Cluster homeRegionCluster = navigableRegion.getHomeRegionCluster();
      List<Cluster> allClusters = navigableRegion.getAllClusters();
      List<Cluster> obstacleClusters = navigableRegion.getObstacleClusters();
      int mapId = navigableRegion.getMapId();

      createNavigableRegionNodes(this, homeRegionCluster, homePlanarRegion, allClusters, mapId, preferredHomeRegionNodes, homeRegionNodes,
                                 innerRegionEdges, createEdgesAroundClusterRing);

      obstaclePreferredNavigableNodes.clear();
      obstacleNavigableNodes.clear();
      for (int i = 0; i < obstacleClusters.size(); i++)
      {
         obstaclePreferredNavigableNodes.add(new ArrayList<>());
         obstacleNavigableNodes.add(new ArrayList<>());
      }

      for (int i = 0; i < obstacleClusters.size(); i++)
      {
         Cluster obstacleCluster = obstacleClusters.get(i);
         List<VisibilityGraphNode> obstacleNodes = obstacleNavigableNodes.get(i);
         List<VisibilityGraphNode> obstaclePreferredNodes = obstaclePreferredNavigableNodes.get(i);
         createNavigableRegionNodes(this, obstacleCluster, homePlanarRegion, allClusters, mapId, obstaclePreferredNodes, obstacleNodes,
                                    innerRegionEdges, createEdgesAroundClusterRing);
      }
   }

   public void createGraphBetweenInnerClusterRings(double nonPreferredWeight, double nonPreferredStaticCost)
   {
      List<Cluster> allClusters = navigableRegion.getAllClusters();

      addClusterSelfVisibility(allClusters, preferredHomeRegionNodes, homeRegionNodes, innerRegionEdges, nonPreferredWeight, nonPreferredStaticCost);

      for (int targetIndex = 0; targetIndex < obstacleNavigableNodes.size(); targetIndex++)
      {
         List<VisibilityGraphNode> targetNodes = obstacleNavigableNodes.get(targetIndex);

         addCrossClusterVisibility(preferredHomeRegionNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, nonPreferredStaticCost, false);
         addCrossClusterVisibility(homeRegionNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, nonPreferredStaticCost, false);
      }

      for (int targetIndex = 0; targetIndex < obstaclePreferredNavigableNodes.size(); targetIndex++)
      {
         List<VisibilityGraphNode> targetNodes = obstaclePreferredNavigableNodes.get(targetIndex);

         addCrossClusterVisibility(preferredHomeRegionNodes, targetNodes, allClusters, innerRegionEdges, 1.0, 0.0, true);
         addCrossClusterVisibility(homeRegionNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, nonPreferredStaticCost, false);
      }

      for (int sourceIndex = 0; sourceIndex < obstacleNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> obstacleNodes = obstacleNavigableNodes.get(sourceIndex);
         List<VisibilityGraphNode> obstaclePreferredNodes = obstaclePreferredNavigableNodes.get(sourceIndex);
         addClusterSelfVisibility(allClusters, obstaclePreferredNodes, obstacleNodes, innerRegionEdges, nonPreferredWeight, nonPreferredStaticCost);
      }

      // between all the obstacle non-preferred navigable nodes.
      for (int sourceIndex = 0; sourceIndex < obstacleNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> sourceNodes = obstacleNavigableNodes.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < obstacleNavigableNodes.size(); targetIndex++)
         {
            List<VisibilityGraphNode> targetNodes = obstacleNavigableNodes.get(targetIndex);

            addCrossClusterVisibility(sourceNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, nonPreferredStaticCost, false);
         }
      }

      // between all the obstacle preferred navigable nodes.
      for (int sourceIndex = 0; sourceIndex < obstaclePreferredNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> sourceNodes = obstaclePreferredNavigableNodes.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < obstaclePreferredNavigableNodes.size(); targetIndex++)
         {
            List<VisibilityGraphNode> targetNodes = obstaclePreferredNavigableNodes.get(targetIndex);

            addCrossClusterVisibility(sourceNodes, targetNodes, allClusters, innerRegionEdges, 1.0, 0.0, true);
         }
      }

      // between all the obstacle preferred and non-preferred navigable nodes.
      for (int sourceIndex = 0; sourceIndex < obstaclePreferredNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> sourceNodes = obstaclePreferredNavigableNodes.get(sourceIndex);

         for (int targetIndex = 0; targetIndex < obstacleNavigableNodes.size(); targetIndex++)
         {
            List<VisibilityGraphNode> targetNodes = obstacleNavigableNodes.get(targetIndex);

            addCrossClusterVisibility(sourceNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, nonPreferredStaticCost, false);
         }
      }

      // between all the obstacle non-preferred and preferred navigable nodes.
      for (int sourceIndex = 0; sourceIndex < obstacleNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> sourceNodes = obstacleNavigableNodes.get(sourceIndex);

         for (int targetIndex = 0; targetIndex < obstaclePreferredNavigableNodes.size(); targetIndex++)
         {
            List<VisibilityGraphNode> targetNodes = obstaclePreferredNavigableNodes.get(targetIndex);

            addCrossClusterVisibility(sourceNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, nonPreferredStaticCost, false);
         }
      }
   }

   // FIXME check the combinatorics
   public static void createNavigableRegionNodes(VisibilityGraphNavigableRegion visibilityGraphNavigableRegion, Cluster clusterToBuildMapOf,
                                                 PlanarRegion homeRegion, List<Cluster> allClusters, int mapId, List<VisibilityGraphNode> preferredNodesToPack,
                                                 List<VisibilityGraphNode> nodesToPack,
                                                 ArrayList<VisibilityGraphEdge> edgesToPack, boolean createEdgesAroundClusterRing)
   {
      List<? extends Point2DReadOnly> preferredNavigableExtrusionPoints = clusterToBuildMapOf.getPreferredNavigableExtrusionsInLocal();
      List<? extends Point2DReadOnly> navigableExtrusionPoints = clusterToBuildMapOf.getNavigableExtrusionsInLocal();
      boolean[] arePreferredPointsActuallyNavigable = VisibilityTools.checkIfPointsInsidePlanarRegionAndOutsideNonNavigableZones(homeRegion, allClusters,
                                                                                                                                 preferredNavigableExtrusionPoints);
      boolean[] arePointsActuallyNavigable = VisibilityTools.checkIfPointsInsidePlanarRegionAndOutsideNonNavigableZones(homeRegion, allClusters,
                                                                                                                        navigableExtrusionPoints);

      ArrayList<VisibilityGraphNode> newPreferredNodes = new ArrayList<>();
      ArrayList<VisibilityGraphNode> newNodes = new ArrayList<>();

      // Create all the nodes that are valid first:
      for (int nodeIndex = 0; nodeIndex < preferredNavigableExtrusionPoints.size(); nodeIndex++)
      {
         if (arePreferredPointsActuallyNavigable[nodeIndex])
         {
            Point2DReadOnly nodePointInLocal = preferredNavigableExtrusionPoints.get(nodeIndex);

            Point3D sourcePointInWorld = new Point3D(nodePointInLocal);
            homeRegion.transformFromLocalToWorld(sourcePointInWorld);
            VisibilityGraphNode node = new VisibilityGraphNode(sourcePointInWorld, nodePointInLocal, visibilityGraphNavigableRegion, true);

            newPreferredNodes.add(node);
            preferredNodesToPack.add(node);
         }
      }

      for (int nodeIndex = 0; nodeIndex < navigableExtrusionPoints.size(); nodeIndex++)
      {
         if (arePointsActuallyNavigable[nodeIndex])
         {
            Point2DReadOnly nodePointInLocal = navigableExtrusionPoints.get(nodeIndex);

            Point3D sourcePointInWorld = new Point3D(nodePointInLocal);
            homeRegion.transformFromLocalToWorld(sourcePointInWorld);
            VisibilityGraphNode node = new VisibilityGraphNode(sourcePointInWorld, nodePointInLocal, visibilityGraphNavigableRegion, false);

            newNodes.add(node);
            nodesToPack.add(node);
         }
      }

      if (createEdgesAroundClusterRing)
      {
         createEdgesAroundClusterRing(newPreferredNodes, allClusters, edgesToPack, true);
         createEdgesAroundClusterRing(newNodes, allClusters, edgesToPack, false);
      }
   }

   private static void createEdgesAroundClusterRing(ArrayList<VisibilityGraphNode> newNodes, List<Cluster> allClusters,
                                                    ArrayList<VisibilityGraphEdge> edgesToPack, boolean checkForPreferredVisibility)
   {
      // Go around the ring looking for connections.
      for (int sourceIndex = 0; sourceIndex < newNodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = newNodes.get(sourceIndex);
         Point2DReadOnly sourcePointInLocal = sourceNode.getPoint2DInLocal();

         // Starting from after the next vertex of the source as we already added all the edges as connections
         int targetIndex = (sourceIndex + 1) % newNodes.size();
         {
            VisibilityGraphNode targetNode = newNodes.get(targetIndex);
            Point2DReadOnly targetPointInLocal = targetNode.getPoint2DInLocal();

            // Finally run the expensive test to verify if the target can be seen from the source.
            if (VisibilityTools.isPointVisibleForStaticMaps(allClusters, sourcePointInLocal, targetPointInLocal, checkForPreferredVisibility))
            {
               VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);

               sourceNode.addEdge(edge);
               targetNode.addEdge(edge);

               edgesToPack.add(edge);
            }
         }
      }
   }

   public static void addClusterSelfVisibility(List<Cluster> allClusters, List<VisibilityGraphNode> preferredNodes, List<VisibilityGraphNode> nodes,
                                               List<VisibilityGraphEdge> edgesToPack, double nonPreferredEdgeWeight, double nonPreferredStaticCost)
   {
      // Going through all of the possible combinations of two points for finding connections going from the preferred to the preferred
      for (int sourceIndex = 0; sourceIndex < preferredNodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = preferredNodes.get(sourceIndex);
         addClusterVisibility(allClusters, sourceNode, preferredNodes, sourceIndex + 1, edgesToPack, 1.0, 0.0, true);
      }

      // Going through all of the possible combinations of two points for finding connections going from the non-preferred to the preferred
      for (int sourceIndex = 0; sourceIndex < nodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = nodes.get(sourceIndex);
         addClusterVisibility(allClusters, sourceNode, preferredNodes, 0, edgesToPack, nonPreferredEdgeWeight, nonPreferredStaticCost, false);
      }

      // Going through all of the possible combinations of two points for finding connections going from the preferred to the non-preferred
      for (int sourceIndex = 0; sourceIndex < preferredNodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = preferredNodes.get(sourceIndex);
         addClusterVisibility(allClusters, sourceNode, nodes, 0, edgesToPack, nonPreferredEdgeWeight, nonPreferredStaticCost, false);
      }

      // Going through all of the possible combinations of two points for finding connections going from the non-preferred to the non-preferred
      for (int sourceIndex = 0; sourceIndex < nodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = nodes.get(sourceIndex);
         addClusterVisibility(allClusters, sourceNode, nodes, sourceIndex + 1, edgesToPack, nonPreferredEdgeWeight, nonPreferredStaticCost, false);
      }
   }

   public static void addCrossClusterVisibility(List<VisibilityGraphNode> sourceNodes, List<VisibilityGraphNode> targetNodes, List<Cluster> allClusters,
                                                ArrayList<VisibilityGraphEdge> edgesToPack, double edgeWeight, double edgeStaticCost,
                                                boolean connectsToPreferredMap)
   {
      for (int sourceIndex = 0; sourceIndex < sourceNodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = sourceNodes.get(sourceIndex);
         addClusterVisibility(allClusters, sourceNode, targetNodes, 0, edgesToPack, edgeWeight, edgeStaticCost, connectsToPreferredMap);
      }
   }

   public static void addClusterVisibility(List<Cluster> allClusters, VisibilityGraphNode sourceNode, List<VisibilityGraphNode> targetNodes, int targetStartIndex,
                                           List<VisibilityGraphEdge> edgesToPack, double edgeWeight, double staticCost, boolean connectsAllOnPreferredMap)
   {
      for (int targetIndex = targetStartIndex; targetIndex < targetNodes.size(); targetIndex++)
      {
         VisibilityGraphNode targetNode = targetNodes.get(targetIndex);

         // Finally run the expensive test to verify if the target can be seen from the source.
         if (VisibilityTools.isPointVisibleForStaticMaps(allClusters, sourceNode.getPoint2DInLocal(), targetNode.getPoint2DInLocal(), connectsAllOnPreferredMap))
         {
            VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
            edge.setEdgeWeight(edgeWeight);
            edge.setStaticEdgeCost(staticCost);
            edge.registerEdgeWithNodes();

            edgesToPack.add(edge);
         }
      }
   }


   public void addInnerRegionEdgesFromSourceNode(VisibilityGraphNode sourceNode, double nonPreferredWeight, double nonPreferredStaticCost)
   {
      List<VisibilityGraphNode> allNavigableNodes = getAllNavigableNodes();
      List<VisibilityGraphNode> allPreferredNavigableNodes = getAllPreferredNavigableNodes();

      for (VisibilityGraphNode targetNode : allNavigableNodes)
         addInnerEdgeFromSourceToTargetNodeIfVisible(sourceNode, targetNode, nonPreferredWeight, nonPreferredStaticCost, false);

      double weight;
      double cost;
      if (sourceNode.isPreferredNode())
      {
         weight = 1.0;
         cost = 0.0;
      }
      else
      {
         weight = nonPreferredWeight;
         cost = nonPreferredStaticCost;
      }

      for (VisibilityGraphNode targetNode : allPreferredNavigableNodes)
         addInnerEdgeFromSourceToTargetNodeIfVisible(sourceNode, targetNode, weight, cost, sourceNode.isPreferredNode());
   }

   public void addInnerEdgeFromSourceToTargetNodeIfVisible(VisibilityGraphNode sourceNode, VisibilityGraphNode targetNode, double weight, double staticCost,
                                                           boolean bothEndsArePreferred)
   {
      checkNavigableRegionConsistency(sourceNode, targetNode);

      List<Cluster> allClusters = navigableRegion.getAllClusters();

      Point2DReadOnly sourceNodeInLocal = sourceNode.getPoint2DInLocal();
      Point2DReadOnly targetNodeInLocal = targetNode.getPoint2DInLocal();

      boolean targetIsVisible = VisibilityTools.isPointVisibleForStaticMaps(allClusters, sourceNodeInLocal, targetNodeInLocal, bothEndsArePreferred);

      if (targetIsVisible)
      {
         addInnerRegionEdge(sourceNode, targetNode, weight, staticCost);
      }
   }

   private void checkNavigableRegionConsistency(VisibilityGraphNode sourceNode, VisibilityGraphNode targetNode)
   {
      if ((sourceNode.getVisibilityGraphNavigableRegion() != this) || (targetNode.getVisibilityGraphNavigableRegion() != this))
      {
         throw new RuntimeException("(sourceNode.getVisibilityGraphNavigableRegion() != this) || (targetNode.getVisibilityGraphNavigableRegion() != this)");
      }
   }

}
