package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

public class PartialExpansionManager
{
   private final FootstepPlannerParametersReadOnly parameters;
   private final List<FootstepNode> allChildNodes = new ArrayList<>();

   private int expansionCount = 0;

   public PartialExpansionManager(FootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public void initialize(List<FootstepNode> allChildNodes)
   {
      this.allChildNodes.clear();
      this.allChildNodes.addAll(allChildNodes);
      expansionCount = 0;
   }

   public void packPartialExpansion(List<FootstepNode> expansionToPack)
   {
      expansionToPack.clear();

      int branchFactor = parameters.getMaximumBranchFactor();
      int endIndex = Math.min(branchFactor * (expansionCount + 1), allChildNodes.size());
      int startIndex = Math.min(branchFactor * expansionCount, endIndex - 1);

      for (int i = startIndex; i < endIndex; i++)
      {
         expansionToPack.add(allChildNodes.get(i));
      }

      expansionCount++;
   }

   public boolean finishedExpansion()
   {
      return (expansionCount + 1) * parameters.getMaximumBranchFactor() >= allChildNodes.size();
   }
}
