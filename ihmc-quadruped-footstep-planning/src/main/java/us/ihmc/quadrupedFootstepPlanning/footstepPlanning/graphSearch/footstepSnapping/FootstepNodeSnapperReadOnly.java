package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface FootstepNodeSnapperReadOnly
{
   /**
    * Returns snap data if the snapper has a cache of this node's snap, otherwise returns null
    */
   FootstepNodeSnapData getSnapData(int xIndex, int yIndex);

   PlanarRegionsList getPlanarRegionsList();

   boolean hasPlanarRegions();
}
