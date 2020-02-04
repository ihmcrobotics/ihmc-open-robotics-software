package us.ihmc.pathPlanning.visibilityGraphs.dataStructure;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.stream.Stream;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * Navigable region with visibility graph nodes
 */
public class VisibilityGraphNavigableRegion
{
   /**
    * This enables a potential 2X speed boost by replacing for loops
    * with a parallel streamed forEach. It breaks a few tests and there
    * is a null edge in the resulting visibility map.
    * See more: https://bitbucket.ihmc.us/projects/LIBS/repos/ihmc-open-robotics-software/pull-requests/1393
    */
   public static final boolean ENABLE_EXPERIMENTAL_SPEEDUP = false;

   private final NavigableRegion navigableRegion;

   private final ArrayList<VisibilityGraphNode> preferredHomeRegionNodes = new ArrayList<>();
   private final ArrayList<VisibilityGraphNode> homeRegionNodes = new ArrayList<>();
   private final ArrayList<List<VisibilityGraphNode>> obstacleNavigableNodes = new ArrayList<>();
   private final ArrayList<List<VisibilityGraphNode>> obstaclePreferredNavigableNodes = new ArrayList<>();
   private final List<VisibilityGraphNode> allNavigableNodes = new ArrayList<>();
   private final List<VisibilityGraphNode> allPreferredNavigableNodes = new ArrayList<>();

   private final HashSet<VisibilityGraphEdge> innerRegionEdges = new HashSet<>();

   private final boolean createEdgesAroundClusterRing;
   private boolean haveNodesBeenCreated = false;

   public VisibilityGraphNavigableRegion(NavigableRegion navigableRegion, boolean createEdgesAroundClusterRing)
   {
      this.navigableRegion = navigableRegion;
      this.createEdgesAroundClusterRing = createEdgesAroundClusterRing;
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
      if (!haveNodesBeenCreated)
         createNavigableRegionNodes();

      return homeRegionNodes;
   }

   public List<VisibilityGraphNode> getAllNavigableNodes()
   {
      if (!haveNodesBeenCreated)
         createNavigableRegionNodes();

      return allNavigableNodes;
   }

   public List<VisibilityGraphNode> getAllPreferredNavigableNodes()
   {
      if (!haveNodesBeenCreated)
         createNavigableRegionNodes();

      return allPreferredNavigableNodes;
   }

   public List<List<VisibilityGraphNode>> getObstaclePreferredNavigableNodes()
   {
      if (!haveNodesBeenCreated)
         createNavigableRegionNodes();

      return obstaclePreferredNavigableNodes;
   }

   public List<List<VisibilityGraphNode>> getObstacleNavigableNodes()
   {
      if (!haveNodesBeenCreated)
         createNavigableRegionNodes();
      return obstacleNavigableNodes;
   }

   public HashSet<VisibilityGraphEdge> getAllEdges()
   {
      if (!haveNodesBeenCreated)
         createNavigableRegionNodes();
      return innerRegionEdges;
   }

   private void addInnerRegionEdge(VisibilityGraphNode sourceNode, VisibilityGraphNode targetNode, double weight)
   {
      VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
      edge.setEdgeWeight(weight);
      edge.registerEnds();
      innerRegionEdges.add(edge);
   }

   public void createNavigableRegionNodes()
   {
      if (haveNodesBeenCreated)
         return;

      PlanarRegion homePlanarRegion = navigableRegion.getHomePlanarRegion();
      Cluster homeRegionCluster = navigableRegion.getHomeRegionCluster();
      List<Cluster> allClusters = navigableRegion.getAllClusters();
      List<Cluster> obstacleClusters = navigableRegion.getObstacleClusters();

      createNavigableRegionNodes(this, homeRegionCluster, homePlanarRegion, allClusters, preferredHomeRegionNodes, homeRegionNodes,
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
         createNavigableRegionNodes(this, obstacleCluster, homePlanarRegion, allClusters, obstaclePreferredNodes, obstacleNodes,
                                    innerRegionEdges, createEdgesAroundClusterRing);
      }

      allNavigableNodes.addAll(homeRegionNodes);
      obstacleNavigableNodes.forEach(allNavigableNodes::addAll);

      allPreferredNavigableNodes.addAll(preferredHomeRegionNodes);
      obstaclePreferredNavigableNodes.forEach(allPreferredNavigableNodes::addAll);

      haveNodesBeenCreated = true;
   }

   /**
    * Create visibility graph for this region.
    */
   public void createGraphBetweenInnerClusterRings(double nonPreferredWeight)
   {
      List<Cluster> allClusters = navigableRegion.getAllClusters();
      createNavigableRegionNodes();

      addClusterSelfVisibility(allClusters, preferredHomeRegionNodes, homeRegionNodes, innerRegionEdges, nonPreferredWeight);

      for (int targetIndex = 0; targetIndex < obstacleNavigableNodes.size(); targetIndex++)
      {
         List<VisibilityGraphNode> targetNodes = obstacleNavigableNodes.get(targetIndex);

         addCrossClusterVisibility(preferredHomeRegionNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, false);
         addCrossClusterVisibility(homeRegionNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, false);
      }

      for (int targetIndex = 0; targetIndex < obstaclePreferredNavigableNodes.size(); targetIndex++)
      {
         List<VisibilityGraphNode> targetNodes = obstaclePreferredNavigableNodes.get(targetIndex);

         addCrossClusterVisibility(preferredHomeRegionNodes, targetNodes, allClusters, innerRegionEdges, 1.0, true);
         addCrossClusterVisibility(homeRegionNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, false);
      }

      for (int sourceIndex = 0; sourceIndex < obstacleNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> obstacleNodes = obstacleNavigableNodes.get(sourceIndex);
         List<VisibilityGraphNode> obstaclePreferredNodes = obstaclePreferredNavigableNodes.get(sourceIndex);
         addClusterSelfVisibility(allClusters, obstaclePreferredNodes, obstacleNodes, innerRegionEdges, nonPreferredWeight);
      }

      // between all the obstacle non-preferred navigable nodes.
      for (int sourceIndex = 0; sourceIndex < obstacleNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> sourceNodes = obstacleNavigableNodes.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < obstacleNavigableNodes.size(); targetIndex++)
         {
            List<VisibilityGraphNode> targetNodes = obstacleNavigableNodes.get(targetIndex);

            addCrossClusterVisibility(sourceNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, false);
         }
      }

      // between all the obstacle preferred navigable nodes.
      for (int sourceIndex = 0; sourceIndex < obstaclePreferredNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> sourceNodes = obstaclePreferredNavigableNodes.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < obstaclePreferredNavigableNodes.size(); targetIndex++)
         {
            List<VisibilityGraphNode> targetNodes = obstaclePreferredNavigableNodes.get(targetIndex);

            addCrossClusterVisibility(sourceNodes, targetNodes, allClusters, innerRegionEdges, 1.0, true);
         }
      }

      // between all the obstacle preferred and non-preferred navigable nodes.
      for (int sourceIndex = 0; sourceIndex < obstaclePreferredNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> sourceNodes = obstaclePreferredNavigableNodes.get(sourceIndex);

         for (int targetIndex = 0; targetIndex < obstacleNavigableNodes.size(); targetIndex++)
         {
            List<VisibilityGraphNode> targetNodes = obstacleNavigableNodes.get(targetIndex);

            addCrossClusterVisibility(sourceNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, false);
         }
      }

      // between all the obstacle non-preferred and preferred navigable nodes.
      for (int sourceIndex = 0; sourceIndex < obstacleNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> sourceNodes = obstacleNavigableNodes.get(sourceIndex);

         for (int targetIndex = 0; targetIndex < obstaclePreferredNavigableNodes.size(); targetIndex++)
         {
            List<VisibilityGraphNode> targetNodes = obstaclePreferredNavigableNodes.get(targetIndex);

            addCrossClusterVisibility(sourceNodes, targetNodes, allClusters, innerRegionEdges, nonPreferredWeight, false);
         }
      }
   }

   private static void createNavigableRegionNodes(VisibilityGraphNavigableRegion visibilityGraphNavigableRegion, Cluster clusterToBuildMapOf,
                                                 PlanarRegion homeRegion, List<Cluster> allClusters, List<VisibilityGraphNode> preferredNodesToPack,
                                                 List<VisibilityGraphNode> nodesToPack, HashSet<VisibilityGraphEdge> edgesToPack,
                                                 boolean createEdgesAroundClusterRing)
   {
      List<ExtrusionHull> preferredNavigableExtrusionPoints = clusterToBuildMapOf.getPreferredNavigableExtrusionsInLocal();
      ExtrusionHull navigableExtrusionPoints = clusterToBuildMapOf.getNavigableExtrusionsInLocal();

      ArrayList<VisibilityGraphNode> newPreferredNodes = new ArrayList<>();
      ArrayList<VisibilityGraphNode> newNodes = new ArrayList<>();

      // create preferred nodes
      for (int clusterIndex = 0; clusterIndex < preferredNavigableExtrusionPoints.size(); clusterIndex++)
      {
         for (int nodeIndex = 0; nodeIndex < preferredNavigableExtrusionPoints.get(clusterIndex).size(); nodeIndex++)
         {
            Point2DReadOnly nodePointInLocal = preferredNavigableExtrusionPoints.get(clusterIndex).get(nodeIndex);
            if (VisibilityTools.checkIfPointIsInRegionAndOutsidePreferredNonNavigableZone(nodePointInLocal, homeRegion, allClusters))
            {
               Point3D sourcePointInWorld = new Point3D(nodePointInLocal);
               homeRegion.transformFromLocalToWorld(sourcePointInWorld);
               VisibilityGraphNode node = new VisibilityGraphNode(sourcePointInWorld, nodePointInLocal, visibilityGraphNavigableRegion, true);

               newPreferredNodes.add(node);
               preferredNodesToPack.add(node);
            }
         }
      }

      // create nodes
      for (int nodeIndex = 0; nodeIndex < navigableExtrusionPoints.size(); nodeIndex++)
      {
         Point2DReadOnly nodePointInLocal = navigableExtrusionPoints.get(nodeIndex);

         if (VisibilityTools.checkIfPointIsInRegionAndOutsideNonNavigableZone(nodePointInLocal, homeRegion, allClusters))
         {
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
                                                    HashSet<VisibilityGraphEdge> edgesToPack, boolean checkForPreferredVisibility)
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
            if (VisibilityTools.isPointVisibleToPointInSameRegion(allClusters, sourcePointInLocal, targetPointInLocal, checkForPreferredVisibility))
            {
               VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
               edge.registerEnds();

               edgesToPack.add(edge);
            }
         }
      }
   }

   private static void addClusterSelfVisibility(List<Cluster> allClusters, List<VisibilityGraphNode> preferredNodes, List<VisibilityGraphNode> nodes,
                                                HashSet<VisibilityGraphEdge> edgesToPack, double nonPreferredEdgeWeight)
   {
      // Going through all of the possible combinations of two points for finding connections going from the preferred to the preferred
      for (int sourceIndex = 0; sourceIndex < preferredNodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = preferredNodes.get(sourceIndex);
         addClusterVisibility(allClusters, sourceNode, preferredNodes, sourceIndex + 1, edgesToPack, 1.0, true);
      }

      // Going through all of the possible combinations of two points for finding connections going from the non-preferred to the preferred
      for (int sourceIndex = 0; sourceIndex < nodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = nodes.get(sourceIndex);
         addClusterVisibility(allClusters, sourceNode, preferredNodes, 0, edgesToPack, nonPreferredEdgeWeight, false);
      }

      // Going through all of the possible combinations of two points for finding connections going from the preferred to the non-preferred
      for (int sourceIndex = 0; sourceIndex < preferredNodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = preferredNodes.get(sourceIndex);
         addClusterVisibility(allClusters, sourceNode, nodes, 0, edgesToPack, nonPreferredEdgeWeight, false);
      }

      // Going through all of the possible combinations of two points for finding connections going from the non-preferred to the non-preferred
      for (int sourceIndex = 0; sourceIndex < nodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = nodes.get(sourceIndex);
         addClusterVisibility(allClusters, sourceNode, nodes, sourceIndex + 1, edgesToPack, nonPreferredEdgeWeight, false);
      }
   }

   private static void addCrossClusterVisibility(List<VisibilityGraphNode> sourceNodes, List<VisibilityGraphNode> targetNodes, List<Cluster> allClusters,
                                                 HashSet<VisibilityGraphEdge> edgesToPack, double edgeWeight, boolean connectsToPreferredMap)
   {
      for (int sourceIndex = 0; sourceIndex < sourceNodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = sourceNodes.get(sourceIndex);
         addClusterVisibility(allClusters, sourceNode, targetNodes, 0, edgesToPack, edgeWeight, connectsToPreferredMap);
      }
   }

   private static void addClusterVisibility(List<Cluster> allClusters, VisibilityGraphNode sourceNode, List<VisibilityGraphNode> targetNodes, int targetStartIndex,
                                            HashSet<VisibilityGraphEdge> edgesToPack, double edgeWeight, boolean connectsAllOnPreferredMap)
   {
      for (int targetIndex = targetStartIndex; targetIndex < targetNodes.size(); targetIndex++)
      {
         VisibilityGraphNode targetNode = targetNodes.get(targetIndex);

         // Finally run the expensive test to verify if the target can be seen from the source.
         if (VisibilityTools.isPointVisibleToPointInSameRegion(allClusters, sourceNode.getPoint2DInLocal(), targetNode.getPoint2DInLocal(), connectsAllOnPreferredMap))
         {
            VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
            edge.setEdgeWeight(edgeWeight);
            edge.registerEnds();

            edgesToPack.add(edge);
         }
      }
   }


   public void addInnerRegionEdgesFromSourceNode(VisibilityGraphNode sourceNode, double nonPreferredWeight)
   {
      List<VisibilityGraphNode> allNavigableNodes = getAllNavigableNodes();
      List<VisibilityGraphNode> allPreferredNavigableNodes = getAllPreferredNavigableNodes();

      Stream<VisibilityGraphNode> navigableNodeStream;
      Stream<VisibilityGraphNode> preferredNavigableNodeStream;
      if (ENABLE_EXPERIMENTAL_SPEEDUP)
      {
         navigableNodeStream = allNavigableNodes.parallelStream();
         preferredNavigableNodeStream = allPreferredNavigableNodes.parallelStream();
      }
      else
      {
         navigableNodeStream = allNavigableNodes.stream();
         preferredNavigableNodeStream = allPreferredNavigableNodes.stream();
      }

      navigableNodeStream
            .forEach(targetNode -> addInnerEdgeFromSourceToTargetNodeIfVisible(sourceNode, targetNode, nonPreferredWeight));

      double weight = sourceNode.isPreferredNode() ? 1.0 : nonPreferredWeight;

      preferredNavigableNodeStream
            .forEach(targetNode -> addInnerEdgeFromSourceToTargetNodeIfVisible(sourceNode, targetNode, weight));
   }

   public synchronized void addInnerEdgeFromSourceToTargetNodeIfVisible(VisibilityGraphNode sourceNode, VisibilityGraphNode targetNode, double weight)
   {
      checkNavigableRegionConsistency(sourceNode, targetNode);

      if (VisibilityTools.isInnerRegionEdgeValid(sourceNode, targetNode))
      {
         addInnerRegionEdge(sourceNode, targetNode, weight);
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
