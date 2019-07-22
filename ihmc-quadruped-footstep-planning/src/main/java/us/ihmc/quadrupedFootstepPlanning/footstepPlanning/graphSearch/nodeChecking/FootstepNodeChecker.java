package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

public abstract class FootstepNodeChecker
{
   protected PlanarRegionsList planarRegionsList;
   protected final ArrayList<QuadrupedFootstepPlannerListener> listeners = new ArrayList<>();
   private HashSet<FootstepNode> rejectedNodes = new HashSet<>();

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
      rejectedNodes.clear();
   }

   protected void rejectNode(FootstepNode node, QuadrupedFootstepPlannerNodeRejectionReason rejectionReason)
   {
      rejectedNodes.add(node);
      for (QuadrupedFootstepPlannerListener listener : listeners)
         listener.rejectNode(node, rejectionReason);
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

   public boolean isNodeValid(FootstepNode node)
   {
      if (rejectedNodes.contains(node))
         return false;

      return isNodeValidInternal(node);
   }

   public abstract boolean isNodeValidInternal(FootstepNode node);

   public abstract void addStartNode(FootstepNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransforms);
}
