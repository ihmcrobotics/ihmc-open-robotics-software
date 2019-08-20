package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.PawPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.List;

public class PawNodeTransitionCheckerOfCheckers extends PawNodeTransitionChecker
{
   private final List<PawNodeTransitionChecker> nodeCheckers;

   public PawNodeTransitionCheckerOfCheckers(List<PawNodeTransitionChecker> nodeCheckers)
   {
      this.nodeCheckers = nodeCheckers;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      nodeCheckers.forEach(checker -> checker.setPlanarRegions(planarRegions));
   }

   @Override
   public boolean isNodeValidInternal(PawNode node, PawNode previousNode)
   {
      for (PawNodeTransitionChecker checker : nodeCheckers)
      {
         if (!checker.isNodeValid(node, previousNode))
            return false;
      }
      return true;
   }

   @Override
   public void addStartNode(PawNode startNode, QuadrantDependentList<RigidBodyTransform> startNodeTransforms)
   {
      nodeCheckers.forEach((checker) -> checker.addStartNode(startNode, startNodeTransforms));
   }

   @Override
   public void addPlannerListener(PawPlannerListener listener)
   {
      for (PawNodeTransitionChecker nodeChecker : nodeCheckers)
         nodeChecker.addPlannerListener(listener);
   }

}
