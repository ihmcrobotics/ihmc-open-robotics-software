package us.ihmc.footstepPlanning.aStar;

import java.util.List;

import javax.vecmath.Point2d;

import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class FootstepNodeValidityChecker
{
   private final PlanarRegionsList regionsList;

   public FootstepNodeValidityChecker(PlanarRegionsList regionsList)
   {
      this.regionsList = regionsList;
   }

   public boolean isNodeValid(FootstepNode node)
   {
      Point2d nodePosition = new Point2d(node.getX(), node.getY());
      List<PlanarRegion> intersection = regionsList.findPlanarRegionsContainingPointByProjectionOntoXYPlane(nodePosition);
      return intersection != null;
   }

}
