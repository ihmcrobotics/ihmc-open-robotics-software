package us.ihmc.footstepPlanning.log;

import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;

import java.util.ArrayList;
import java.util.List;

public class FootstepPlannerIterationData
{
   private FootstepGraphNode parentNode = null;
   private FootstepGraphNode idealChildNode = null;
   private final List<FootstepGraphNode> childNodes = new ArrayList<>();

   // TODO log all snap data separately, or do fancy graphics logging
   private final FootstepSnapData parentEndSnapData = FootstepSnapData.identityData();
   private final FootstepSnapData parentStartSnapData = FootstepSnapData.identityData();

   public FootstepPlannerIterationData()
   {
      clear();
   }

   public void setParentNode(FootstepGraphNode parentNode)
   {
      this.parentNode = parentNode;
   }

   public void setIdealChildNode(FootstepGraphNode idealChildNode)
   {
      this.idealChildNode = idealChildNode;
   }

   public void setParentEndSnapData(FootstepSnapData parentEndSnapData)
   {
      this.parentEndSnapData.set(parentEndSnapData);
   }

   public void setParentStartSnapData(FootstepSnapData parentStartSnapData)
   {
      this.parentStartSnapData.set(parentStartSnapData);
   }

   public void addChildNode(FootstepGraphNode childNode)
   {
      childNodes.add(childNode);
   }

   public FootstepGraphNode getParentNode()
   {
      return parentNode;
   }

   public FootstepGraphNode getIdealChildNode()
   {
      return idealChildNode;
   }

   public List<FootstepGraphNode> getChildNodes()
   {
      return childNodes;
   }

   public FootstepSnapData getParentEndSnapData()
   {
      return parentEndSnapData;
   }

   public FootstepSnapData getParentStartSnapData()
   {
      return parentStartSnapData;
   }

   public void clear()
   {
      parentNode = null;
      idealChildNode = null;
      childNodes.clear();
      parentEndSnapData.clear();
      parentStartSnapData.clear();
   }
}
