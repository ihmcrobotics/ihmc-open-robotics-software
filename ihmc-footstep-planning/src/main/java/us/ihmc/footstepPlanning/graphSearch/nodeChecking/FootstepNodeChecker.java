package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;

public abstract class FootstepNodeChecker
{
   protected PlanarRegionsList planarRegionsList;
   protected final ArrayList<BipedalFootstepPlannerListener> listeners = new ArrayList<>();
   protected FootstepGraph graph;

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegionsList = planarRegions;
   }

   protected void rejectNode(FootstepNode node, FootstepNode parentNode, BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      for (BipedalFootstepPlannerListener listener : listeners)
         listener.rejectNode(node, parentNode, rejectionReason);
   }

   boolean hasPlanarRegions()
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
      if(node.equals(previousNode))
      {
         throw new RuntimeException("Cannot check a node with itself");
      }
      else
      {
         return isNodeValidInternal(node, previousNode);
      }
   }

   public void addFootstepGraph(FootstepGraph graph)
   {
      this.graph = graph;
   }

   abstract boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode);

   public abstract void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform);
}
