package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;

import java.util.List;

public interface FootstepExpansion
{
   /**
    * Calculates all possible steps from the given stance node
    * @param stanceNode
    * @param expansionToPack list of child steps. Ordering indicates queue priority
    */
   void doFullExpansion(FootstepGraphNode stanceNode, List<FootstepGraphNode> expansionToPack);

   /**
    * Packs part of the full expansion. Successive calls to this method will pack different subsets of the full expansion
    * in decreasing order of queue priority.
    *
    * @return whether the stance node has more child nodes
    */
   default boolean doIterativeExpansion(FootstepGraphNode stanceNode, List<FootstepGraphNode> expansionToPack)
   {
      doFullExpansion(stanceNode, expansionToPack);
      return false;
   }
}
