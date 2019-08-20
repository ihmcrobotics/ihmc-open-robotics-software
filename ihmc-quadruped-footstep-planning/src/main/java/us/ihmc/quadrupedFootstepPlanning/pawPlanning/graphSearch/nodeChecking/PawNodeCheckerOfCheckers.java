package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.PawPlannerListener;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;

import java.util.List;

public class PawNodeCheckerOfCheckers extends PawNodeChecker
{
   private final List<PawNodeChecker> nodeCheckers;

   public PawNodeCheckerOfCheckers(List<PawNodeChecker> nodeCheckers)
   {
      this.nodeCheckers = nodeCheckers;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      nodeCheckers.forEach(checker -> checker.setPlanarRegions(planarRegions));
   }

   @Override
   public boolean isNodeValidInternal(PawNode node)
   {
      for (PawNodeChecker checker : nodeCheckers)
      {
         if (!checker.isNodeValid(node))
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
      for (PawNodeChecker nodeChecker : nodeCheckers)
         nodeChecker.addPlannerListener(listener);
   }

}
