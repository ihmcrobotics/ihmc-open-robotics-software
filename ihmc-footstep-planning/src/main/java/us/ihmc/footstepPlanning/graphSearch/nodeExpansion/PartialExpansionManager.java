package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

public class PartialExpansionManager
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final List<FootstanceNode> allChildNodes = new ArrayList<>();

   private int expansionCount = 0;

   public PartialExpansionManager(FootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public void initialize(List<FootstanceNode> allChildNodes)
   {
      this.allChildNodes.clear();
      this.allChildNodes.addAll(allChildNodes);
      expansionCount = 0;
   }

   public void packPartialExpansion(List<FootstanceNode> expansionToPack)
   {
      expansionToPack.clear();

      if (finishedExpansion())
      {
         return;
      }

      int branchFactor = parameters.getMaximumBranchFactor();
      int startIndex = branchFactor * expansionCount;
      int endIndex = Math.min(branchFactor * (expansionCount + 1), allChildNodes.size());

      for (int i = startIndex; i < endIndex; i++)
      {
         expansionToPack.add(allChildNodes.get(i));
      }

      expansionCount++;
   }

   public boolean finishedExpansion()
   {
      return expansionCount * parameters.getMaximumBranchFactor() >= allChildNodes.size();
   }
}
