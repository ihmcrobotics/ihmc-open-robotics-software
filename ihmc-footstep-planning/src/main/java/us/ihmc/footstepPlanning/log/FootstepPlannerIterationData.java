package us.ihmc.footstepPlanning.log;

import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;

import java.util.ArrayList;
import java.util.List;

public class FootstepPlannerIterationData
{
   private DiscreteFootstep stanceNode = null;
   private DiscreteFootstep idealStep = null;
   private final List<DiscreteFootstep> childNodes = new ArrayList<>();
   private final FootstepSnapData stanceNodeSnapData = FootstepSnapData.identityData();

   public FootstepPlannerIterationData()
   {
      clear();
   }

   public void setStanceNode(DiscreteFootstep stanceNode)
   {
      this.stanceNode = stanceNode;
   }

   public void setIdealStep(DiscreteFootstep idealStep)
   {
      this.idealStep = idealStep;
   }

   public void setStanceNodeSnapData(FootstepSnapData stanceNodeSnapData)
   {
      this.stanceNodeSnapData.set(stanceNodeSnapData);
   }

   public void addChildNode(DiscreteFootstep childNode)
   {
      childNodes.add(childNode);
   }

   public DiscreteFootstep getStanceNode()
   {
      return stanceNode;
   }

   public DiscreteFootstep getIdealStep()
   {
      return idealStep;
   }

   public List<DiscreteFootstep> getChildNodes()
   {
      return childNodes;
   }

   public FootstepSnapData getStanceStepSnapData()
   {
      return stanceNodeSnapData;
   }

   public void clear()
   {
      stanceNode = null;
      idealStep = null;
      childNodes.clear();
      stanceNodeSnapData.clear();
   }
}
