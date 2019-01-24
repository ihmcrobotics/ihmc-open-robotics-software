package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;

public abstract class FootstepNodeChecker
{
   protected PlanarRegionsList planarRegionsList;
   protected final ArrayList<BipedalFootstepPlannerListener> listeners = new ArrayList<>();

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
   }

   protected void rejectNode(FootstepNode node, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      for (BipedalFootstepPlannerListener listener : listeners)
         listener.rejectNode(node, parentNode, rejectionReason);
   }

   protected boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   public void addPlannerListener(BipedalFootstepPlannerListener listener)
   {
      if (listener != null)
         listeners.add(listener);
   }

   public boolean isNodeValid(FootstepNode node, FootstepNode previousNode)
   {
      if(!hasPlanarRegions())
      {
         return true;
      }
      else
      {
         return isNodeValidInternal(node, previousNode);
      }
   }

   abstract boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode);

   public abstract void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform);
}
