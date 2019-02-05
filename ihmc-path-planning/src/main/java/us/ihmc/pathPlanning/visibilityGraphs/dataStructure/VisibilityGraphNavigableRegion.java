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

   private final ArrayList<VisibilityGraphNode> homeRegionNodes = new ArrayList<VisibilityGraphNode>();
   private final ArrayList<List<VisibilityGraphNode>> obstacleNavigableNodes = new ArrayList<>();
   private final ArrayList<VisibilityGraphNode> intermediateObstacleNodes = new ArrayList<>();

   private final ArrayList<VisibilityGraphEdge> innerRegionEdges = new ArrayList<VisibilityGraphEdge>();

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

   public void addIntermediateObstacleNode(VisibilityGraphNode node)
   {
      intermediateObstacleNodes.add(node);
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
      VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
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

      createNavigableRegionNodes(this, homeRegionCluster, homePlanarRegion, allClusters, mapId, homeRegionNodes, innerRegionEdges,
                                 createEdgesAroundClusterRing);

      obstacleNavigableNodes.clear();
      for (int i = 0; i < obstacleClusters.size(); i++)
      {
         obstacleNavigableNodes.add(new ArrayList<>());
      }

      for (int i = 0; i < obstacleClusters.size(); i++)
      {
         Cluster obstacleCluster = obstacleClusters.get(i);
         List<VisibilityGraphNode> obstacleNodes = obstacleNavigableNodes.get(i);
         createNavigableRegionNodes(this, obstacleCluster, homePlanarRegion, allClusters, mapId, obstacleNodes, innerRegionEdges, createEdgesAroundClusterRing);
      }
   }

   public void createGraphBetweenInnerClusterRings()
   {
      int regionId = navigableRegion.getMapId();
      List<Cluster> allClusters = navigableRegion.getAllClusters();

      addClusterSelfVisibility(allClusters, regionId, homeRegionNodes, innerRegionEdges);

      for (int targetIndex = 0; targetIndex < obstacleNavigableNodes.size(); targetIndex++)
      {
         List<VisibilityGraphNode> targetNodes = obstacleNavigableNodes.get(targetIndex);

         addCrossClusterVisibility(homeRegionNodes, targetNodes, allClusters, regionId, innerRegionEdges);
      }

      for (int sourceIndex = 0; sourceIndex < obstacleNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> obstacleNodes = obstacleNavigableNodes.get(sourceIndex);
         addClusterSelfVisibility(allClusters, regionId, obstacleNodes, innerRegionEdges);
      }

      for (int sourceIndex = 0; sourceIndex < obstacleNavigableNodes.size(); sourceIndex++)
      {
         List<VisibilityGraphNode> sourceNodes = obstacleNavigableNodes.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < obstacleNavigableNodes.size(); targetIndex++)
         {
            List<VisibilityGraphNode> targetNodes = obstacleNavigableNodes.get(targetIndex);

            addCrossClusterVisibility(sourceNodes, targetNodes, allClusters, regionId, innerRegionEdges);
         }
      }
   }

   public static void createNavigableRegionNodes(VisibilityGraphNavigableRegion visibilityGraphNavigableRegion, Cluster clusterToBuildMapOf,
                                                 PlanarRegion homeRegion, List<Cluster> allClusters, int mapId, List<VisibilityGraphNode> nodesToPack,
                                                 ArrayList<VisibilityGraphEdge> edgesToPack, boolean createEdgesAroundClusterRing)
   {
      List<? extends Point2DReadOnly> navigableExtrusionPoints = clusterToBuildMapOf.getNavigableExtrusionsInLocal();
      boolean[] arePointsActuallyNavigable = VisibilityTools.checkIfPointsInsidePlanarRegionAndOutsideNonNavigableZones(homeRegion, allClusters,
                                                                                                                        navigableExtrusionPoints);

      ArrayList<VisibilityGraphNode> newNodes = new ArrayList<>();

      // Create all the nodes that are valid first:
      for (int nodeIndex = 0; nodeIndex < navigableExtrusionPoints.size(); nodeIndex++)
      {
         if (arePointsActuallyNavigable[nodeIndex])
         {
            Point2DReadOnly nodePointInLocal = navigableExtrusionPoints.get(nodeIndex);

            Point3D sourcePointInWorld = new Point3D(nodePointInLocal);
            homeRegion.transformFromLocalToWorld(sourcePointInWorld);
            VisibilityGraphNode node = new VisibilityGraphNode(sourcePointInWorld, nodePointInLocal, visibilityGraphNavigableRegion);

            newNodes.add(node);
            nodesToPack.add(node);
         }
      }

      if (createEdgesAroundClusterRing)
      {
         createEdgesAroundClusterRing(newNodes, allClusters, edgesToPack);
      }
   }

   private static void createEdgesAroundClusterRing(ArrayList<VisibilityGraphNode> newNodes, List<Cluster> allClusters,
                                                    ArrayList<VisibilityGraphEdge> edgesToPack)
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
            if (VisibilityTools.isPointVisibleForStaticMaps(allClusters, sourcePointInLocal, targetPointInLocal))
            {
               VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);

               sourceNode.addEdge(edge);
               targetNode.addEdge(edge);

               edgesToPack.add(edge);
            }
         }
      }
   }

   public static void addClusterSelfVisibility(List<Cluster> allClusters, int mapId, List<VisibilityGraphNode> nodes,
                                               List<VisibilityGraphEdge> edgesToPack)
   {
      // Going through all of the possible combinations of two points for finding connections
      for (int sourceIndex = 0; sourceIndex < nodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = nodes.get(sourceIndex);

         for (int targetIndex = sourceIndex + 1; targetIndex < nodes.size(); targetIndex++)
         {
            VisibilityGraphNode targetNode = nodes.get(targetIndex);

            // Finally run the expensive test to verify if the target can be seen from the source.
            if (VisibilityTools.isPointVisibleForStaticMaps(allClusters, sourceNode.getPoint2DInLocal(), targetNode.getPoint2DInLocal()))
            {
               VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
               edgesToPack.add(edge);

               sourceNode.addEdge(edge);
               targetNode.addEdge(edge);
            }
         }
      }
   }

   public static void addCrossClusterVisibility(List<VisibilityGraphNode> sourceNodes, List<VisibilityGraphNode> targetNodes, List<Cluster> allClusters,
                                                int mapId, ArrayList<VisibilityGraphEdge> edgesToPack)
   {
      for (int sourceIndex = 0; sourceIndex < sourceNodes.size(); sourceIndex++)
      {
         VisibilityGraphNode sourceNode = sourceNodes.get(sourceIndex);

         for (int targetIndex = 0; targetIndex < targetNodes.size(); targetIndex++)
         {
            VisibilityGraphNode targetNode = targetNodes.get(targetIndex);

            if (VisibilityTools.isPointVisibleForStaticMaps(allClusters, sourceNode.getPoint2DInLocal(), targetNode.getPoint2DInLocal()))
            {

               VisibilityGraphEdge edge = new VisibilityGraphEdge(sourceNode, targetNode);
               edgesToPack.add(edge);

               sourceNode.addEdge(edge);
               targetNode.addEdge(edge);
            }
         }
      }
   }

   public void addInnerRegionEdgesFromSourceNode(VisibilityGraphNode sourceNode)
   {
      List<VisibilityGraphNode> allNavigableNodes = getAllNavigableNodes();

      for (VisibilityGraphNode targetNode : allNavigableNodes)
      {
         addInnerEdgeFromSourceToTargetNodeIfVisible(sourceNode, targetNode);
      }
   }

   public void addInnerEdgeFromSourceToTargetNodeIfVisible(VisibilityGraphNode sourceNode, VisibilityGraphNode targetNode)
   {
      checkNavigableRegionConsistency(sourceNode, targetNode);

      List<Cluster> allClusters = navigableRegion.getAllClusters();

      Point2DReadOnly sourceNodeInLocal = sourceNode.getPoint2DInLocal();
      Point2DReadOnly targetNodeInLocal = targetNode.getPoint2DInLocal();

      boolean targetIsVisible = VisibilityTools.isPointVisibleForStaticMaps(allClusters, sourceNodeInLocal, targetNodeInLocal);

      if (targetIsVisible)
      {
         addInnerRegionEdge(sourceNode, targetNode);
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
