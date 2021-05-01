package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.PawStepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.PawStepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.ArrayList;

public abstract class PawNodeTransitionChecker
{
   protected PlanarRegionsList planarRegionsList;
   protected final ArrayList<PawStepPlannerListener> listeners = new ArrayList<>();

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
   }

   protected void rejectNode(PawNode node, PawNode parentNode, PawStepPlannerNodeRejectionReason rejectionReason)
   {
      for (PawStepPlannerListener listener : listeners)
         listener.rejectNode(node, parentNode, rejectionReason);
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

   public boolean isNodeValid(PawNode node, PawNode previousNode)
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

   public abstract boolean isNodeValidInternal(PawNode node, PawNode previousNode);

   public abstract void addStartNode(PawNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransforms);
}
