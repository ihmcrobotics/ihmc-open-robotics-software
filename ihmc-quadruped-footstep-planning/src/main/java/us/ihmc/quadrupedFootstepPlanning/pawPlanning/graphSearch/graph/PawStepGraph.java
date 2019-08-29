package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph;

import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.*;

/**
 * Class that maintains a directed graph of {@link PawNode}.
 *
 * The graph has to be connected and is grown by adding edges to known or new nodes. These
 * edges must start at known nodes. The class is initialized with a start node and maintains
 * shortest paths and costs to reach all nodes in the graph efficiently.
 *
 * @author Robert
 */
public class PawStepGraph
{
   private final HashMap<PawStepEdge, PawStepEdgeCost> edgeCostMap = new HashMap<>();
   private final HashMap<PawNode, PawNodeCost> nodeCostMap = new HashMap<>();

   private final HashMap<PawNode, HashSet<PawStepEdge>> outgoingEdges = new HashMap<>();
   private final HashMap<PawNode, PawStepEdge> incomingBestEdge = new HashMap<>();

   /**
    * Removes all nodes and edges stored in the graph and
    * starts a new graph from the specified start node.
    *
    * @param startNode start node of the new graph
    */
   public void initialize(PawNode startNode)
   {
      edgeCostMap.clear();
      nodeCostMap.clear();
      outgoingEdges.clear();
      incomingBestEdge.clear();

      nodeCostMap.put(startNode, new PawNodeCost(0.0));
      PawStepEdge edgeToStart = new PawStepEdge(null, startNode);
      incomingBestEdge.put(startNode, edgeToStart);
   }

   /**
    * Adds an edge to the graph and updates all path and node costs affected. The edge must
    * originate at a known node and the cost associated to moving along the edge must be given.
    */
   public void checkAndSetEdge(PawNode startNode, PawNode endNode, double transitionCost)
   {
      checkNodeExists(startNode);

      PawStepEdge edge = new PawStepEdge(startNode, endNode);
      if (edgeCostMap.containsKey(edge))
         throw new RuntimeException("Edge exists already.");

      if (!outgoingEdges.containsKey(startNode))
         outgoingEdges.put(startNode, new HashSet<>());
      PawStepEdgeCost cost = new PawStepEdgeCost(transitionCost);
      edgeCostMap.put(edge, cost);
      outgoingEdges.get(startNode).add(edge);

      double newNodeCost = nodeCostMap.get(startNode).getNodeCost() + transitionCost;
      if (nodeCostMap.containsKey(endNode))
      {
         double oldNodeCost = nodeCostMap.get(endNode).getNodeCost();
         if (newNodeCost >= oldNodeCost)
            return;

         nodeCostMap.put(endNode, new PawNodeCost(newNodeCost));
         incomingBestEdge.put(endNode, edge);
         updateChildCostsRecursively(endNode);
      }
      else
      {
         nodeCostMap.put(endNode, new PawNodeCost(newNodeCost));
         incomingBestEdge.put(endNode, edge);
      }
   }

   /**
    * Gets the cost associated to traveling from the start node to the given node.
    */
   public double getCostFromStart(PawNode node)
   {
      checkNodeExists(node);

      return nodeCostMap.get(node).getNodeCost();
   }

   /**
    * Returns all nodes required to travel from the start node to the given node.
    * The nodes returned include start and end node of the path.
    */
   public List<PawNode> getPathFromStart(PawNode endNode)
   {
      checkNodeExists(endNode);

      ArrayList<PawNode> path = new ArrayList<>();
      path.add(endNode);

      PawStepEdge edgeFromParent = incomingBestEdge.get(endNode);
      while (edgeFromParent.getStartNode() != null)
      {
         PawNode parentNode = edgeFromParent.getStartNode();
         PawNode childNode = edgeFromParent.getEndNode();

         if (!edgeFromParent.isValidEdge())
            throw new RuntimeException("Edge moves more than one paw, making it invalid.");

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (childNode.getMovingQuadrant() == robotQuadrant)
               continue;

            if (!edgeFromParent.getEndNode().quadrantGeometricallyEquals(robotQuadrant, edgeFromParent.getStartNode()))
               throw new RuntimeException("Edge moves more than one paw, and I missed it somehow.");

         }

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
   public boolean doesNodeExist(PawNode node)
   {
      return nodeCostMap.containsKey(node);
   }

   private void updateChildCostsRecursively(PawNode node)
   {
      if (!outgoingEdges.containsKey(node))
         return;

      double parentNodeCost = nodeCostMap.get(node).getNodeCost();
      for (PawStepEdge outgoingEdge : outgoingEdges.get(node))
      {
         double newCost = parentNodeCost + edgeCostMap.get(outgoingEdge).getEdgeCost();
         PawNode childNode = outgoingEdge.getEndNode();

         double oldCost = nodeCostMap.get(childNode).getNodeCost();
         if (oldCost <= newCost)
            continue;

         nodeCostMap.put(childNode, new PawNodeCost(newCost));
         incomingBestEdge.put(childNode, outgoingEdge);
         updateChildCostsRecursively(childNode);
      }
   }

   private void checkNodeExists(PawNode node)
   {
      if (!doesNodeExist(node))
         throw new RuntimeException("Node has not been added to graph yet.");
   }
}
