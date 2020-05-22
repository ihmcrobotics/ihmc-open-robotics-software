package us.ihmc.footstepPlanning.graphSearch;

import java.util.ArrayList;
import java.util.List;

public class AStarIterationData<N>
{
   private N parentNode;
   private final List<N> validChildNodes = new ArrayList<>();
   private final List<N> invalidChildNodes = new ArrayList<>();

   public N getParentNode()
   {
      return parentNode;
   }

   void setParentNode(N parentNode)
   {
      this.parentNode = parentNode;
   }

   void clear()
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
