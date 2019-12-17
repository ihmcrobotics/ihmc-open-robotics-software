package us.ihmc.pathPlanning.graph.structure;

import java.util.*;

/**
 * Class that maintains a directed graph.
 *
 * The graph has to be connected and is grown by adding edges to known or new nodes. These
 * edges must start at known nodes. The class is initialized with a start node and maintains
 * shortest paths and costs to reach all nodes in the graph efficiently.
 *
 * @author Georg
 */
public class DirectedGraph<N>
{
   private final HashMap<GraphEdge, EdgeCost> edgeCostMap = new HashMap<>();
   private final HashMap<N, NodeCost> nodeCostMap = new HashMap<>();

   private final HashMap<N, HashSet<GraphEdge<N>>> outgoingEdges = new HashMap<>();
   private final HashMap<N, GraphEdge<N>> incomingBestEdge = new HashMap<>();

   /**
    * Removes all nodes and edges stored in the graph and
    * starts a new graph from the specified start node.
    *
    * @param startNode start node of the new graph
    */
   public void initialize(N startNode)
   {
      edgeCostMap.clear();
      nodeCostMap.clear();
      outgoingEdges.clear();
      incomingBestEdge.clear();

      nodeCostMap.put(startNode, new NodeCost(0.0));
      GraphEdge<N> edgeToStart = new GraphEdge<>(null, startNode);
      incomingBestEdge.put(startNode, edgeToStart);
   }

   public HashMap<N, HashSet<GraphEdge<N>>> getOutgoingEdges()
   {
      return outgoingEdges;
   }

   /**
    * Adds an edge to the graph and updates all path and node costs affected. The edge must
    * originate at a known node and the cost associated to moving along the edge must be given.
    *
    * @param startNode
    * @param endNode
    * @param transitionCost
    */
   public void checkAndSetEdge(N startNode, N endNode, double transitionCost)
   {
      checkNodeExists(startNode);

      GraphEdge<N> edge = new GraphEdge<>(startNode, endNode);
      if (edgeCostMap.containsKey(edge))
         throw new RuntimeException("Edge exists already.");

      if (!outgoingEdges.containsKey(startNode))
         outgoingEdges.put(startNode, new HashSet<>());
      EdgeCost cost = new EdgeCost(transitionCost);
      edgeCostMap.put(edge, cost);
      outgoingEdges.get(startNode).add(edge);

      double newNodeCost = nodeCostMap.get(startNode).getNodeCost() + transitionCost;
      if (nodeCostMap.containsKey(endNode))
      {
         double oldNodeCost = nodeCostMap.get(endNode).getNodeCost();
         if (newNodeCost >= oldNodeCost)
            return;

         nodeCostMap.put(endNode, new NodeCost(newNodeCost));
         incomingBestEdge.put(endNode, edge);
         updateChildCostsRecursively(endNode);
      }
      else
      {
         nodeCostMap.put(endNode, new NodeCost(newNodeCost));
         incomingBestEdge.put(endNode, edge);
      }
   }

   /**
    * Gets the cost associated to traveling from the start node to the given node.
    */
   public double getCostFromStart(N node)
   {
      checkNodeExists(node);

      return nodeCostMap.get(node).getNodeCost();
   }

   /**
    * Returns all nodes required to travel from the start node to the given node.
    * The nodes returned include start and end node of the path.
    */
   public List<N> getPathFromStart(N node)
   {
      checkNodeExists(node);

      ArrayList<N> path = new ArrayList<>();
      path.add(node);

      GraphEdge<N> edgeFromParent = incomingBestEdge.get(node);
      while (edgeFromParent.getStartNode() != null)
      {
         N parentNode = edgeFromParent.getStartNode();
         path.add(parentNode);
         edgeFromParent = incomingBestEdge.get(parentNode);
      }

      Collections.reverse(path);
      return path;
   }

   /**
    * Will check if a node exists in the graph.
    * @param node
    */
   public boolean doesNodeExist(N node)
   {
      return nodeCostMap.containsKey(node);
   }

   private void updateChildCostsRecursively(N node)
   {
      if (!outgoingEdges.containsKey(node))
         return;

      double parentNodeCost = nodeCostMap.get(node).getNodeCost();
      for (GraphEdge<N> outgoingEdge : outgoingEdges.get(node))
      {
         double newCost = parentNodeCost + edgeCostMap.get(outgoingEdge).getEdgeCost();
         N childNode = outgoingEdge.getEndNode();

         double oldCost = nodeCostMap.get(childNode).getNodeCost();
         if (oldCost <= newCost)
            continue;

         nodeCostMap.put(childNode, new NodeCost(newCost));
         incomingBestEdge.put(childNode, outgoingEdge);
         updateChildCostsRecursively(childNode);
      }
   }

   private void checkNodeExists(N node)
   {
      if (!nodeCostMap.containsKey(node))
         throw new RuntimeException("Node has not been added to graph yet.");
   }

   public N getParentNode(N node)
   {
      return incomingBestEdge.get(node).getStartNode();
   }
}
