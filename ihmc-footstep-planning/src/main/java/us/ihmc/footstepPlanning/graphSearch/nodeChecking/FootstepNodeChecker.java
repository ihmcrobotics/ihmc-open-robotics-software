package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class FootstepNodeChecker
{
   protected PlanarRegionsList planarRegionsList;
   protected BipedalFootstepPlannerListener listener;

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
   }

   public void addPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listener = listener;
   }

   protected void rejectNode(FootstepNode node, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      if(listener != null)
         listener.rejectNode(node, rejectionReason);
   }

   protected boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   public abstract boolean isNodeValid(FootstepNode node, FootstepNode previousNode);

   public abstract void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform);
}
