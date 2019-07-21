package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.List;

public class FootstepNodeTransitionCheckerOfCheckers extends FootstepNodeTransitionChecker
{
   private final List<FootstepNodeTransitionChecker> nodeCheckers;

   public FootstepNodeTransitionCheckerOfCheckers(List<FootstepNodeTransitionChecker> nodeCheckers)
   {
      this.nodeCheckers = nodeCheckers;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      nodeCheckers.forEach(checker -> checker.setPlanarRegions(planarRegions));
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode)
   {
      for(FootstepNodeTransitionChecker checker : nodeCheckers)
      {
         if(!checker.isNodeValid(node, previousNode))
            return false;
      }
      return true;
   }

   @Override
   public void addStartNode(FootstepNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransforms)
   {
      nodeCheckers.forEach((checker) -> checker.addStartNode(startNode, startNodeTransforms));
   }

   @Override
   public void addPlannerListener(QuadrupedFootstepPlannerListener listener)
   {
      for (FootstepNodeTransitionChecker nodeChecker : nodeCheckers)
         nodeChecker.addPlannerListener(listener);
   }

}
