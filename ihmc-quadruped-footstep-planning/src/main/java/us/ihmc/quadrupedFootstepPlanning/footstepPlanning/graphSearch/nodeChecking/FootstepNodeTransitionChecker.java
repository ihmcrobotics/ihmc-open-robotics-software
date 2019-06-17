package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.ArrayList;

public abstract class FootstepNodeTransitionChecker
{
   protected PlanarRegionsList planarRegionsList;
   protected final ArrayList<QuadrupedFootstepPlannerListener> listeners = new ArrayList<>();

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
   }

   protected void rejectNode(FootstepNode node, FootstepNode parentNode, QuadrupedFootstepPlannerNodeRejectionReason rejectionReason)
   {
      for (QuadrupedFootstepPlannerListener listener : listeners)
         listener.rejectNode(node, parentNode, rejectionReason);
   }

   protected boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   public void addPlannerListener(QuadrupedFootstepPlannerListener listener)
   {
      if (listener != null)
         listeners.add(listener);
   }

   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      if(node.equals(previousNode))
      {
         throw new RuntimeException("Cannot check a node with itself");
      }
      else
      {
         return isNodeValidInternal(node, previousNode);
      }
   }

   public abstract boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode);

   public abstract void addStartNode(FootstepNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransforms);
}
