package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;

import java.util.ArrayList;
import java.util.List;

public class PartialExpansionManager
{
   private final DefaultFootstepPlannerParametersReadOnly parameters;
   private final List<FootstepGraphNode> allChildNodes = new ArrayList<>();

   private int expansionCount = 0;

   public PartialExpansionManager(DefaultFootstepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public void initialize(List<FootstepGraphNode> allChildNodes)
   {
      this.allChildNodes.clear();
      this.allChildNodes.addAll(allChildNodes);
      expansionCount = 0;
   }

   public void packPartialExpansion(List<FootstepGraphNode> expansionToPack)
   {
      expansionToPack.clear();

      if (finishedExpansion())
      {
         return;
      }

      int branchFactor = parameters.getMaxBranchFactor();
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
      return expansionCount * parameters.getMaxBranchFactor() >= allChildNodes.size();
   }
}
