package us.ihmc.footstepPlanning.aStar.implementations;

import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.aStar.FootstepNode;
import us.ihmc.footstepPlanning.aStar.FootstepNodeChecker;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SimpleNodeChecker implements FootstepNodeChecker
{
   private PlanarRegionsList planarRegions;

   @Override
   public boolean isNodeValid(FootstepNode node)
   {
      if (planarRegions == null)
         return true;

      Point2D nodePosition = new Point2D(node.getX(), node.getY());
      List<PlanarRegion> intersection = planarRegions.findPlanarRegionsContainingPointByProjectionOntoXYPlane(nodePosition);
      return intersection != null;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      this.planarRegions = planarRegions;
   }

}
