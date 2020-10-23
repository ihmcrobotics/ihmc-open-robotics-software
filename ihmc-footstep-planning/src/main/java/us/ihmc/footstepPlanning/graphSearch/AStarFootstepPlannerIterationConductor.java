package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.footstepPlanning.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepCheckerInterface;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.FootstepExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostCalculatorInterface;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.function.ToDoubleFunction;

/**
 * Class that performs planning iterations given generic node expansion, checker, cost and heuristic calculators.
 * Interfaces between {@link AStarFootstepPlanner} and {@link DirectedGraph}
 */
public class AStarFootstepPlannerIterationConductor
{
   private final HashSet<FootstepGraphNode> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<FootstepGraphNode> graph = new DirectedGraph<>();
   private final AStarIterationData<FootstepGraphNode> iterationData = new AStarIterationData<>();
   private final List<FootstepGraphNode> neighbors = new ArrayList<>();

   private final PriorityQueue<FootstepGraphNode> stack;
   private final FootstepExpansion nodeExpansion;
   private final FootstepCheckerInterface edgeChecker;
   private final FootstepCostCalculatorInterface edgeCostCalculator;

   /**
    * @param nodeExpansion edge calculator. Calling {@code nodeExpansion.apply} returns all possible neighbor nodes
    * @param edgeChecker checks the validity of an edge
    * @param edgeCostCalculator calculates the cost of an edge (nodes are assumed zero cost), the arguments are {@code edgeCostCalculator.applyAsDouble(parentNode, childNode)}
    * @param heuristicsCalculator calling {@code heuristicsCalculator.applyAsDouble(node)} returns the heuristic cost from the node to the goal
    */
   public AStarFootstepPlannerIterationConductor(FootstepExpansion nodeExpansion,
                                                 FootstepCheckerInterface edgeChecker,
                                                 FootstepCostCalculatorInterface edgeCostCalculator,
                                                 ToDoubleFunction<FootstepGraphNode> heuristicsCalculator)
   {
      this.nodeExpansion = nodeExpansion;
      this.edgeChecker = edgeChecker;
      this.edgeCostCalculator = edgeCostCalculator;

      NodeComparator<FootstepGraphNode> nodeComparator = new NodeComparator<>(graph, heuristicsCalculator);
      this.stack = new PriorityQueue<>(nodeComparator);
   }

   /**
    * Clears the stack and search graph and adds the given node
    * @param startNode
    */
   public void initialize(FootstepGraphNode startNode)
   {
      stack.clear();
      stack.add(startNode);
      graph.initialize(startNode);

      expandedNodeSet.clear();
   }

  /**
    * Performs iteration according to {@link #doPlanningIteration}.
    * The node expanded is the one with the {@code costFromStart + heuristicCostToGoal}
    *
    * @param nodeToExpand the node that will be expanded
    * @return the node that was expanded and all child nodes that were added to the graph
    */
   public AStarIterationData<FootstepGraphNode> doPlanningIteration(FootstepGraphNode nodeToExpand, boolean performIterativeExpansion)
   {
      iterationData.clear();
      iterationData.setParentNode(nodeToExpand);

      boolean partialExpansion;
      if (performIterativeExpansion)
      {
         partialExpansion = nodeExpansion.doIterativeExpansion(nodeToExpand, neighbors);
      }
      else
      {
         nodeExpansion.doFullExpansion(nodeToExpand, neighbors);
         partialExpansion = false;
      }

      for(FootstepGraphNode neighbor : neighbors)
      {
         if(edgeChecker.isStepValid(neighbor.getSecondStep(), nodeToExpand.getSecondStep(), nodeToExpand.getFirstStep()))
         {
            double edgeCost = edgeCostCalculator.computeCost(neighbor.getSecondStep(), nodeToExpand.getSecondStep(), nodeToExpand.getFirstStep());
            graph.checkAndSetEdge(nodeToExpand, neighbor, edgeCost);
            iterationData.getValidChildNodes().add(neighbor);
            stack.add(neighbor);
         }
         else
         {
            graph.checkAndSetEdge(nodeToExpand, neighbor, Double.POSITIVE_INFINITY);
            iterationData.getInvalidChildNodes().add(neighbor);
         }
      }

      if (partialExpansion)
      {
         stack.add(nodeToExpand);
      }
      else
      {
         expandedNodeSet.add(nodeToExpand);
      }

      return iterationData;
   }

   public FootstepGraphNode getNextNode()
   {
      while (!stack.isEmpty())
      {
         FootstepGraphNode nextNode = stack.poll();
         if (!expandedNodeSet.contains(nextNode))
         {
            return nextNode;
         }
      }

      return null;
   }

   public DirectedGraph<FootstepGraphNode> getGraph()
   {
      return graph;
   }

   public AStarIterationData<FootstepGraphNode> getIterationData()
   {
      return iterationData;
   }
}
