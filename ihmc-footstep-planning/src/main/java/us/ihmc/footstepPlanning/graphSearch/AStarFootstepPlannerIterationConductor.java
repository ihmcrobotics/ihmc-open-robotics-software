package us.ihmc.footstepPlanning.graphSearch;

import us.ihmc.footstepPlanning.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.function.BiPredicate;
import java.util.function.ToDoubleBiFunction;
import java.util.function.ToDoubleFunction;

/**
 * Class that performs planning iterations given generic node expansion, checker, cost and heuristic calculators.
 * Interfaces between {@link AStarFootstepPlanner} and {@link DirectedGraph}
 */
public class AStarFootstepPlannerIterationConductor
{
   private final HashSet<FootstanceNode> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<FootstanceNode> graph = new DirectedGraph<>();
   private final AStarIterationData<FootstanceNode> iterationData = new AStarIterationData<>();
   private final List<FootstanceNode> neighbors = new ArrayList<>();

   private final PriorityQueue<FootstanceNode> stack;
   private final FootstepNodeExpansion nodeExpansion;
   private final BiPredicate<FootstanceNode, FootstanceNode> edgeChecker;
   private final ToDoubleBiFunction<FootstanceNode, FootstanceNode> edgeCostCalculator;

   /**
    * @param nodeExpansion edge calculator. Calling {@code nodeExpansion.apply} returns all possible neighbor nodes
    * @param edgeChecker checks the validity of an edge, the arguments are {@code edgeChecker.test(childNode, parentNode)}
    * @param edgeCostCalculator calculates the cost of an edge (nodes are assumed zero cost), the arguments are {@code edgeCostCalculator.applyAsDouble(parentNode, childNode)}
    * @param heuristicsCalculator calling {@code heuristicsCalculator.applyAsDouble(node)} returns the heuristic cost from the node to the goal
    */
   public AStarFootstepPlannerIterationConductor(FootstepNodeExpansion nodeExpansion,
                                                 BiPredicate<FootstanceNode, FootstanceNode> edgeChecker,
                                                 ToDoubleBiFunction<FootstanceNode, FootstanceNode> edgeCostCalculator,
                                                 ToDoubleFunction<FootstanceNode> heuristicsCalculator)
   {
      this.nodeExpansion = nodeExpansion;
      this.edgeChecker = edgeChecker;
      this.edgeCostCalculator = edgeCostCalculator;

      NodeComparator<FootstanceNode> nodeComparator = new NodeComparator<>(graph, heuristicsCalculator);
      this.stack = new PriorityQueue<>(nodeComparator);
   }

   /**
    * Clears the stack and search graph and adds the given node
    * @param startNode
    */
   public void initialize(FootstanceNode startNode)
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
   public AStarIterationData<FootstanceNode> doPlanningIteration(FootstanceNode nodeToExpand, boolean performIterativeExpansion)
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

      for(FootstanceNode neighbor : neighbors)
      {
         if(edgeChecker.test(neighbor, nodeToExpand))
         {
            double edgeCost = edgeCostCalculator.applyAsDouble(nodeToExpand, neighbor);
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

   public FootstanceNode getNextNode()
   {
      while (!stack.isEmpty())
      {
         FootstanceNode nextNode = stack.poll();
         if (!expandedNodeSet.contains(nextNode))
         {
            return nextNode;
         }
      }

      return null;
   }

   public DirectedGraph<FootstanceNode> getGraph()
   {
      return graph;
   }

   public AStarIterationData<FootstanceNode> getIterationData()
   {
      return iterationData;
   }
}
