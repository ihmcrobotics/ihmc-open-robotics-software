package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.QuadrupedPawPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.PawPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.ArrayList;
import java.util.HashSet;

public abstract class PawNodeChecker
{
   protected PlanarRegionsList planarRegionsList;
   protected final ArrayList<PawPlannerListener> listeners = new ArrayList<>();
   private HashSet<PawNode> rejectedNodes = new HashSet<>();

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
      rejectedNodes.clear();
   }

   protected void rejectNode(PawNode node, QuadrupedPawPlannerNodeRejectionReason rejectionReason)
   {
      rejectedNodes.add(node);
      for (PawPlannerListener listener : listeners)
         listener.rejectNode(node, rejectionReason);
   }

   protected boolean hasPlanarRegions()
   {
      return planarRegionsList != null && !planarRegionsList.isEmpty();
   }

   public void addPlannerListener(PawPlannerListener listener)
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
