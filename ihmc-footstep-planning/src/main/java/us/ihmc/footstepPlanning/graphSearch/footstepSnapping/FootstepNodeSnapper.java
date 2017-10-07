package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class FootstepNodeSnapper
{
   private final FootstepNodeSnapDataHolder snapDataHolder = new FootstepNodeSnapDataHolder();
   protected PlanarRegionsList planarRegionsList;

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public FootstepNodeSnapData snapFootstepNode(FootstepNode footstepNode)
   {
      if(planarRegionsList == null)
      {
         return FootstepNodeSnapData.emptyData();
      }

      if(!snapDataHolder.containsKey(footstepNode))
      {
         FootstepNodeSnapData snapData = snapInternal(footstepNode);
         snapDataHolder.put(footstepNode, snapData);
      }

      return snapDataHolder.get(footstepNode);
   }

   protected abstract FootstepNodeSnapData snapInternal(FootstepNode footstepNode);
}
