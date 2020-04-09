package us.ihmc.pathPlanning.graph.search;

import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;

import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.function.*;

/**
 * Generic A* planner class that performs a search given generic node expansion, checker, cost and heuristic calculators.
 * API is set up to perform single iterations at a time so that termination conditions can be determined externally
 *
 * @param <N> node type and input to the expansion, validity and cost calculators
 */
public class AStarPathPlanner<N>
{
   private final HashSet<N> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<N> graph = new DirectedGraph<>();
   private final AStarIterationData<N> iterationData = new AStarIterationData<>();

   private final PriorityQueue<N> stack;
   private final Function<N, List<N>> nodeExpansion;
   private final BiPredicate<N, N> edgeChecker;
   private final ToDoubleBiFunction<N, N> edgeCostCalculator;

   /**
    * @param nodeExpansion edge calculator. Calling {@code nodeExpansion.apply} returns all possible neighbor nodes
    * @param edgeChecker checks the validity of an edge, the arguments are {@code edgeChecker.test(childNode, parentNode)}
    * @param edgeCostCalculator calculates the cost of an edge (nodes are assumed zero cost), the arguments are {@code edgeCostCalculator.applyAsDouble(parentNode, childNode)}
    * @param heuristicsCalculator calling {@code heuristicsCalculator.applyAsDouble(node)} returns the heuristic cost from the node to the goal
    */
   public AStarPathPlanner(Function<N, List<N>> nodeExpansion,
                           BiPredicate<N, N> edgeChecker,
                           ToDoubleBiFunction<N, N> edgeCostCalculator,
                           ToDoubleFunction<N> heuristicsCalculator)
   {
      this.nodeExpansion = nodeExpansion;
      this.edgeChecker = edgeChecker;
      this.edgeCostCalculator = edgeCostCalculator;

      NodeComparator<N> nodeComparator = new NodeComparator<>(graph, heuristicsCalculator);
      this.stack = new PriorityQueue<>(nodeComparator);
   }

   /**
    * Clears the stack and search graph and adds the given node
    * @param startNode
    */
   public void initialize(N startNode)
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
    * @return the node that was expanded and all child nodes that were added to the graph
    */
   public AStarIterationData<N> doPlanningIteration()
   {
      if (stack.isEmpty())
         return iterationData;

      N nodeToExpand = getNextNode();
      if(nodeToExpand == null)
         return iterationData;

      return doPlanningIteration(nodeToExpand);
   }

   /**
    * Does single search iteration. A search iteration consists of expanding a single node and adding the resulting edges to the graph.
    * @param nodeToExpand the node that will be expanded
    * @return the node that was expanded and all child nodes that were added to the graph
    */
   public AStarIterationData<N> doPlanningIteration(N nodeToExpand)
   {
      iterationData.clear();

      iterationData.setParentNode(nodeToExpand);
      expandedNodeSet.add(nodeToExpand);

      List<N> neighbors = nodeExpansion.apply(nodeToExpand);
      for(N neighbor : neighbors)
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

      return iterationData;
   }

   private N getNextNode()
   {
      while (!stack.isEmpty())
      {
         N nextNode = stack.poll();
         if (!expandedNodeSet.contains(nextNode))
            return nextNode;
      }

      return null;
   }

   public DirectedGraph<N> getGraph()
   {
      return graph;
   }

   public PriorityQueue<N> getStack()
   {
      return stack;
   }

   public AStarIterationData<N> getIterationData()
   {
      return iterationData;
   }
}
