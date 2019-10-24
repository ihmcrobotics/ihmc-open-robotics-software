package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.listeners.BipedalFootstepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class FootstepNodeCheckerOfCheckers extends FootstepNodeChecker
{
   private final List<FootstepNodeChecker> nodeCheckers;

   public FootstepNodeCheckerOfCheckers(List<FootstepNodeChecker> nodeCheckers)
   {
      this.nodeCheckers = nodeCheckers;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      super.setPlanarRegions(planarRegions);
      nodeCheckers.forEach(checker -> checker.setPlanarRegions(planarRegions));
   }

   public void addFootstepGraph(FootstepGraph graph)
   {
      nodeCheckers.forEach(checker -> checker.addFootstepGraph(graph));
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode)
   {
      for(FootstepNodeChecker checker : nodeCheckers)
      {
         if(!checker.isNodeValid(node, previousNode))
            return false;
      }
      return true;
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
      nodeCheckers.forEach((checker) -> checker.addStartNode(startNode, startNodeTransform));
   }

   @Override
   public void addPlannerListener(BipedalFootstepPlannerListener listener)
   {
      this.listeners.add(listener);
      for (FootstepNodeChecker nodeChecker : nodeCheckers)
         nodeChecker.addPlannerListener(listener);
   }

}
