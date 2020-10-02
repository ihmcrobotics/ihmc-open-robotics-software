package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;
import java.util.List;

public class FootstepPlannerIterationData<N>
{
   private N parentNode;
   private final List<N> validChildNodes = new ArrayList<>();
   private final List<N> invalidChildNodes = new ArrayList<>();

   public N getParentNode()
   {
      return parentNode;
   }

   public void setParentNode(N parentNode)
   {
      this.parentNode = parentNode;
   }

   public void clear()
   {
      this.parentNode = null;
      validChildNodes.clear();
      invalidChildNodes.clear();
   }

   public List<N> getValidChildNodes()
   {
      return validChildNodes;
   }

   public List<N> getInvalidChildNodes()
   {
      return invalidChildNodes;
   }
}
