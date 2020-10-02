package us.ihmc.footstepPlanning.graphSearch;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.footstepPlanning.BipedalFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;

import java.util.*;
import java.util.function.BiPredicate;
import java.util.function.ToDoubleBiFunction;
import java.util.function.ToDoubleFunction;

/**
 * Class that performs planning iterations given generic node expansion, checker, cost and heuristic calculators.
 * Interfaces between {@link BipedalFootstepPlanner} and {@link DirectedGraph}
 */
public class FootstepPlannerIterationConductor
{
   private final HashSet<FootstepNode> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<FootstepNode> graph = new DirectedGraph<>();
   private final FootstepPlannerIterationData<FootstepNode> iterationData = new FootstepPlannerIterationData<>();
   private final List<FootstepNode> neighbors = new ArrayList<>();

   private final FootstepNodeExpansion nodeExpansion;
   private final BiPredicate<FootstepNode, FootstepNode> edgeChecker;
   private final ToDoubleBiFunction<FootstepNode, FootstepNode> edgeCostCalculator;

   private boolean performAStarSearch;
   /** Used for AStar search */
   private final PriorityQueue<FootstepNode> priorityQueue;
   /** Used for depth-first search */
   private final ArrayDeque<FootstepNode> arrayQueue = new ArrayDeque<>();

   /** List for ranking nodes according to edge cost, only used in depth-first mode */
   private final List<Pair<FootstepNode, Double>> nodeCostList = new ArrayList<>();
   /**
    * @param nodeExpansion edge calculator. Calling {@code nodeExpansion.apply} returns all possible neighbor nodes
    * @param edgeChecker checks the validity of an edge, the arguments are {@code edgeChecker.test(childNode, parentNode)}
    * @param edgeCostCalculator calculates the cost of an edge (nodes are assumed zero cost), the arguments are {@code edgeCostCalculator.applyAsDouble(parentNode, childNode)}
    * @param heuristicsCalculator calling {@code heuristicsCalculator.applyAsDouble(node)} returns the heuristic cost from the node to the goal
    */
   public FootstepPlannerIterationConductor(FootstepNodeExpansion nodeExpansion,
                                            BiPredicate<FootstepNode, FootstepNode> edgeChecker,
                                            ToDoubleBiFunction<FootstepNode, FootstepNode> edgeCostCalculator,
                                            ToDoubleFunction<FootstepNode> heuristicsCalculator)
   {
      this.nodeExpansion = nodeExpansion;
      this.edgeChecker = edgeChecker;
      this.edgeCostCalculator = edgeCostCalculator;

      NodeComparator<FootstepNode> nodeComparator = new NodeComparator<>(graph, heuristicsCalculator);
      priorityQueue = new PriorityQueue<>(nodeComparator);
   }

   /**
    * Clears the stack and search graph and adds the given node
    * @param startNode initial graph node
    * @param performAStarSearch if true, performs AStar search. Otherwise performs depth-first search.
    * The difference is that the depth-first search ignores the heuristics and adds to a stack according to edge cost
    */
   public void initialize(FootstepNode startNode, boolean performAStarSearch)
   {
      this.performAStarSearch = performAStarSearch;
      priorityQueue.clear();
      arrayQueue.clear();

      graph.initialize(startNode);
      addNodeToQueue(startNode);

      expandedNodeSet.clear();
   }

  /**
    * Performs iteration according to {@link #doPlanningIteration}.
    * The node expanded is the one with the {@code costFromStart + heuristicCostToGoal}
    *
    * @param nodeToExpand the node that will be expanded
    * @return the node that was expanded and all child nodes that were added to the graph
    */
   public FootstepPlannerIterationData<FootstepNode> doPlanningIteration(FootstepNode nodeToExpand, boolean performIterativeExpansion)
   {
      iterationData.clear();
      iterationData.setParentNode(nodeToExpand);
      nodeCostList.clear();

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

      if (partialExpansion)
      {
         addNodeToQueue(nodeToExpand);
      }
      else
      {
         expandedNodeSet.add(nodeToExpand);
      }

      for(FootstepNode neighbor : neighbors)
      {
         if(edgeChecker.test(neighbor, nodeToExpand))
         {
            double edgeCost = edgeCostCalculator.applyAsDouble(nodeToExpand, neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, edgeCost);
            iterationData.getValidChildNodes().add(neighbor);
            nodeCostList.add(Pair.of(neighbor, edgeCost));
         }
         else
         {
            graph.checkAndSetEdge(nodeToExpand, neighbor, Double.POSITIVE_INFINITY);
            iterationData.getInvalidChildNodes().add(neighbor);
         }
      }

      if (performAStarSearch)
      {
         nodeCostList.forEach(node -> addNodeToQueue(node.getLeft()));
      }
      else
      {
         nodeCostList.sort(Comparator.comparingDouble(Pair::getRight));
         for (int i = nodeCostList.size() - 1; i >= 0; i--)
         {
            addNodeToQueue(nodeCostList.get(i).getLeft());
         }
      }

      nodeToExpand.getChildNodes().addAll(neighbors);
      return iterationData;
   }

   public FootstepNode getNextNode()
   {
      while (!getActiveQueue().isEmpty())
      {
         FootstepNode nextNode = getActiveQueue().poll();
         if (!expandedNodeSet.contains(nextNode))
         {
            return nextNode;
         }
      }

      return null;
   }

   private Queue<FootstepNode> getActiveQueue()
   {
      return performAStarSearch ? priorityQueue : arrayQueue;
   }

   private void addNodeToQueue(FootstepNode footstepNode)
   {
      if (performAStarSearch)
      {
         priorityQueue.add(footstepNode);
      }
      else
      {
         arrayQueue.addFirst(footstepNode);
      }
   }

   public List<Pair<FootstepNode, Double>> getNodeCostList()
   {
      return nodeCostList;
   }

   public DirectedGraph<FootstepNode> getGraph()
   {
      return graph;
   }

   public FootstepPlannerIterationData<FootstepNode> getIterationData()
   {
      return iterationData;
   }
}
