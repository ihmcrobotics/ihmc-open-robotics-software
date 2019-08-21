package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedFootstepPlannerNodeRejectionReason;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.HashSet;
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
      nodeCheckers.forEach(checker -> checker.setPlanarRegions(planarRegions));
   }

   @Override
   public boolean isNodeValidInternal(FootstepNode node)
   {
      for (FootstepNodeChecker checker : nodeCheckers)
      {
         if (!checker.isNodeValid(node))
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
      for (FootstepNodeChecker nodeChecker : nodeCheckers)
         nodeChecker.addPlannerListener(listener);
   }

}
