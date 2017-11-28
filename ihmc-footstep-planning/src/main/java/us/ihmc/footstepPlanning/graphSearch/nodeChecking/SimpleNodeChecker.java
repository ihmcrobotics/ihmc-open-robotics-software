package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SimpleNodeChecker implements FootstepNodeChecker
{
   private PlanarRegionsList planarRegions;

   @Override
   public boolean isNodeValid(FootstepNode node, FootstepNode previosNode)
   {
      /** In case of flat ground walking */
      if (planarRegions == null)
         return true;

      Point2D nodePosition = new Point2D(node.getX(), node.getY());
      List<PlanarRegion> intersection = planarRegions.findPlanarRegionsContainingPointByProjectionOntoXYPlane(nodePosition);
      if (intersection == null)
         return false;

      return intersection != null;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegions = planarRegions;
   }

   @Override
   public void addStartNode(FootstepNode startNode, RigidBodyTransform startNodeTransform)
   {
   }

}
