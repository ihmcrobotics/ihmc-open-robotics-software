package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SimpleNodeChecker extends FootstepNodeChecker
{
   @Override
   public boolean isNodeValidInternal(FootstepNode node, FootstepNode previousNode)
   {
      Point2D nodePosition = new Point2D(node.getX(), node.getY());
      List<PlanarRegion> intersection = planarRegionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(nodePosition);
      boolean intersectsPlanarRegions = intersection != null;
      if(!intersectsPlanarRegions)
         rejectNode(node, previousNode, BipedalFootstepPlannerNodeRejectionReason.COULD_NOT_SNAP);

      return intersectsPlanarRegions;
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
   }
}
