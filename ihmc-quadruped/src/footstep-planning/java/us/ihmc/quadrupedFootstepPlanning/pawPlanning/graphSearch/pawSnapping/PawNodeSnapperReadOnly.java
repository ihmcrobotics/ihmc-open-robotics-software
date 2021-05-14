package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface PawNodeSnapperReadOnly
{
   /**
    * Returns snap data if the snapper has a cache of this node's snap, otherwise returns null
    */
   PawNodeSnapData getSnapData(int xIndex, int yIndex);

   PlanarRegionsList getPlanarRegionsList();

   boolean hasPlanarRegions();
}
