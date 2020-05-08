package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

import java.util.List;

public interface FootstepNodeExpansion
{
   /**
    * Calculates all possible steps from the given stance node
    * @param stanceNode
    * @param expansionToPack list of child steps. Ordering indicates queue priority
    */
   void doFullExpansion(FootstepNode stanceNode, List<FootstepNode> expansionToPack);

   /**
    * Packs part of the full expansion. Successive calls to this method will pack different subsets of the full expansion
    * in decreasing order of queue priority.
    *
    * @return whether the stance node has more child nodes
    */
   default boolean doIterativeExpansion(FootstepNode stanceNode, List<FootstepNode> expansionToPack)
   {
      doFullExpansion(stanceNode, expansionToPack);
      return false;
   }
}
