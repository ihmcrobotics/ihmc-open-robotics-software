package us.ihmc.humanoidBehaviors.exploreArea;

import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.function.ToDoubleFunction;

public class ExploreAreaLatticePlanner
{
   private class LatticeCell
   {
      int x;
      int y;
   }

   private final HashSet<LatticeCell> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<LatticeCell> graph = new DirectedGraph<>();
   private final AStarIterationData<LatticeCell> iterationData = new AStarIterationData<>();
   private final List<FootstepGraphNode> neighbors = new ArrayList<>();
   private final PriorityQueue<LatticeCell> stack;

   public ExploreAreaLatticePlanner(ToDoubleFunction<LatticeCell> heuristicsCalculator)
   {
      NodeComparator<LatticeCell> nodeComparator = new NodeComparator<>(graph, heuristicsCalculator);
      this.stack = new PriorityQueue<>(nodeComparator);
   }

   /**
    * Clears the stack and search graph and adds the given node
    * @param startNode
    */
   public void initialize(LatticeCell startNode)
   {
      stack.clear();
      stack.add(startNode);
      graph.initialize(startNode);

      expandedNodeSet.clear();
   }

   public AStarIterationData<LatticeCell> doPlanningIteration(LatticeCell nodeToExpand, boolean performIterativeExpansion)
   {
//      iterationData.clear();
//      iterationData.setParentNode(nodeToExpand);
//
//      boolean partialExpansion;
//      if (performIterativeExpansion)
//      {
//         partialExpansion = nodeExpansion.doIterativeExpansion(nodeToExpand, neighbors);
//      }
//      else
//      {
//         nodeExpansion.doFullExpansion(nodeToExpand, neighbors);
//         partialExpansion = false;
//      }
//
//      for(FootstepGraphNode neighbor : neighbors)
//      {
//         if(edgeChecker.isStepValid(neighbor.getSecondStep(), nodeToExpand.getSecondStep(), nodeToExpand.getFirstStep()))
//         {
//            double edgeCost = edgeCostCalculator.computeCost(neighbor.getSecondStep(), nodeToExpand.getSecondStep(), nodeToExpand.getFirstStep());
//            graph.checkAndSetEdge(nodeToExpand, neighbor, edgeCost);
//            iterationData.getValidChildNodes().add(neighbor);
//            stack.add(neighbor);
//         }
//         else
//         {
//            graph.checkAndSetEdge(nodeToExpand, neighbor, Double.POSITIVE_INFINITY);
//            iterationData.getInvalidChildNodes().add(neighbor);
//         }
//      }
//
//      if (partialExpansion)
//      {
//         stack.add(nodeToExpand);
//      }
//      else
//      {
//         expandedNodeSet.add(nodeToExpand);
//      }

      return iterationData;
   }

   public LatticeCell getNextNode()
   {
      while (!stack.isEmpty())
      {
         LatticeCell nextNode = stack.poll();
         if (!expandedNodeSet.contains(nextNode))
         {
            return nextNode;
         }
      }

      return null;
   }

   public DirectedGraph<LatticeCell> getGraph()
   {
      return graph;
   }

   public AStarIterationData<LatticeCell> getIterationData()
   {
      return iterationData;
   }

}
