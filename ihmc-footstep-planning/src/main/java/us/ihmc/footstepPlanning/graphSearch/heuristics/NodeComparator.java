package us.ihmc.footstepPlanning.graphSearch.heuristics;

import java.util.Comparator;
import java.util.function.ToDoubleFunction;

import us.ihmc.footstepPlanning.graphSearch.graph.DirectedGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class NodeComparator<N> implements Comparator<N>
{
   private final DirectedGraph<N> graph;
   private final ToDoubleFunction<N> heuristics;

   public NodeComparator(DirectedGraph<N> graph, ToDoubleFunction<N> heuristics)
   {
      this.graph = graph;
      this.heuristics = heuristics;
   }

   @Override
   public int compare(N o1, N o2)
   {
      double cost1 = graph.getCostFromStart(o1) + heuristics.applyAsDouble(o1);
      double cost2 = graph.getCostFromStart(o2) + heuristics.applyAsDouble(o2);
      if (cost1 == cost2) return 0;
      return cost1 < cost2 ? -1 : 1;
   }
}
