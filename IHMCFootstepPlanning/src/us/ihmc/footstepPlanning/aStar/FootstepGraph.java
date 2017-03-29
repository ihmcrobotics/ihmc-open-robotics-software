package us.ihmc.footstepPlanning.aStar;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

/**
 * Class that maintains a directed graph of FootstepNodes.
 *
 * The graph has to be connected and is grown by adding edges to known or new nodes. These
 * edges must start at known nodes. The class is initialized with a start node and maintains
 * shortest paths and costs to reach all nodes in the graph efficiently.
 *
 * @author Georg
 */
public class FootstepGraph
{
   private final HashMap<FootstepEdge, EdgeCost> edgeCostMap = new HashMap<>();
   private final HashMap<FootstepNode, NodeCost> nodeCostMap = new HashMap<>();

   private final HashMap<FootstepNode, HashSet<FootstepEdge>> outgoingEdges = new HashMap<>();
   private final HashMap<FootstepNode, FootstepEdge> incomingBestEdge = new HashMap<>();

   public FootstepGraph(FootstepNode startNode)
   {
      nodeCostMap.put(startNode, new NodeCost(0.0));
      FootstepEdge edgeToStart = new FootstepEdge(null, startNode);
      incomingBestEdge.put(startNode, edgeToStart);
   }

   /**
    * Adds an edge to the graph and updates all path and node costs affected. The edge must
    * originate at a known node and the cost associated to moving along the edge must be given.
    *
    * @param startNode
    * @param endNode
    * @param transitionCost
    */
   public void checkAndSetEdge(FootstepNode startNode, FootstepNode endNode, double transitionCost)
   {
      checkNodeExists(startNode);

      FootstepEdge edge = new FootstepEdge(startNode, endNode);
      if (edgeCostMap.containsKey(edge))
         throw new RuntimeException("Edge exists already.");

      if (!outgoingEdges.containsKey(startNode))
         outgoingEdges.put(startNode, new HashSet<FootstepEdge>());
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
   public double getCostFromStart(FootstepNode node)
   {
      checkNodeExists(node);

      return nodeCostMap.get(node).getNodeCost();
   }

   /**
    * Returns all nodes required to travel from the start node to the given node.
    * The nodes returned include start and end node of the path.
    */
   public List<FootstepNode> getPathFromStart(FootstepNode node)
   {
      checkNodeExists(node);

      ArrayList<FootstepNode> path = new ArrayList<>();
      path.add(node);

      FootstepEdge edgeFromParent = incomingBestEdge.get(node);
      while (edgeFromParent.getStartNode() != null)
      {
         FootstepNode parentNode = edgeFromParent.getStartNode();
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
   public boolean doesNodeExist(FootstepNode node)
   {
      return nodeCostMap.containsKey(node);
   }

   private void updateChildCostsRecursively(FootstepNode node)
   {
      if (!outgoingEdges.containsKey(node))
         return;

      double parentNodeCost = nodeCostMap.get(node).getNodeCost();
      for (FootstepEdge outgoingEdge : outgoingEdges.get(node))
      {
         double newCost = parentNodeCost + edgeCostMap.get(outgoingEdge).getEdgeCost();
         FootstepNode childNode = outgoingEdge.getEndNode();

         double oldCost = nodeCostMap.get(childNode).getNodeCost();
         if (oldCost <= newCost)
            continue;

         nodeCostMap.put(childNode, new NodeCost(newCost));
         incomingBestEdge.put(childNode, outgoingEdge);
         updateChildCostsRecursively(childNode);
      }
   }

   private void checkNodeExists(FootstepNode node)
   {
      if (!nodeCostMap.containsKey(node))
         throw new RuntimeException("Node has not been added to graph yet.");
   }
}
