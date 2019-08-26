package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawStepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.PawStepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.ArrayList;
import java.util.HashSet;

public abstract class PawNodeChecker
{
   protected PlanarRegionsList planarRegionsList;
   protected final ArrayList<PawStepPlannerListener> listeners = new ArrayList<>();
   private HashSet<PawNode> rejectedNodes = new HashSet<>();

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
      rejectedNodes.clear();
   }

   protected void rejectNode(PawNode node, PawStepPlannerNodeRejectionReason rejectionReason)
   {
      rejectedNodes.add(node);
      for (PawStepPlannerListener listener : listeners)
         listener.rejectNode(node, rejectionReason);
   }

   protected boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   public void addPlannerListener(PawStepPlannerListener listener)
   {
      if (listener != null)
         listeners.add(listener);
   }

   public boolean isNodeValid(PawNode node)
   {
      if (rejectedNodes.contains(node))
         return false;

      return isNodeValidInternal(node);
   }

   public abstract boolean isNodeValidInternal(PawNode node);

   public abstract void addStartNode(PawNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransforms);
}
