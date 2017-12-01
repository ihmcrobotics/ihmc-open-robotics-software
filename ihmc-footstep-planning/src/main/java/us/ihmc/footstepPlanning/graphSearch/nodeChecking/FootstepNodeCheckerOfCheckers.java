package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class FootstepNodeCheckerOfCheckers implements FootstepNodeChecker
{
   private final List<FootstepNodeChecker> nodeCheckers;

   public FootstepNodeCheckerOfCheckers(List<FootstepNodeChecker> nodeCheckers)
   {
      this.nodeCheckers = nodeCheckers;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      nodeCheckers.forEach((checker) -> checker.setPlanarRegions(planarRegions));
   }

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previosNode)
   {
      for(FootstepNodeChecker checker : nodeCheckers)
      {
         if(!checker.isNodeValid(node, previosNode))
            return false;
      }
      return true;
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
      nodeCheckers.forEach((checker) -> checker.addStartNode(startNode, startNodeTransform));
   }
}
