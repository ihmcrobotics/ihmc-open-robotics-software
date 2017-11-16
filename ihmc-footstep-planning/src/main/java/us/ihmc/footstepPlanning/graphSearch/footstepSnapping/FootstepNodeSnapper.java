package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import java.util.HashMap;

import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class FootstepNodeSnapper implements FootstepNodeSnapperReadOnly
{
   private final HashMap<FootstepNode, FootstepNodeSnapData> snapDataHolder = new HashMap<>();
   protected PlanarRegionsList planarRegionsList;

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
      snapDataHolder.clear();
   }

   public FootstepNodeSnapData snapFootstepNode(FootstepNode footstepNode)
   {
      if (snapDataHolder.containsKey(footstepNode))
      {
         return snapDataHolder.get(footstepNode);
      }
      else if (planarRegionsList == null)
      {
         FootstepNodeSnapData identityData = FootstepNodeSnapData.identityData();
         addSnapData(footstepNode, identityData);
         return identityData;
      }
      else
      {
         FootstepNodeSnapData snapData = snapInternal(footstepNode);
         addSnapData(footstepNode, snapData);
         return snapData;
      }
   }

   /**
    * Can manually add snap data for a footstep node to bypass the snapper.
    */
   public void addSnapData(FootstepNode footstepNode, FootstepNodeSnapData snapData)
   {
      snapDataHolder.put(footstepNode, snapData);
   }

   @Override
   public FootstepNodeSnapData getSnapData(FootstepNode node)
   {
      return snapDataHolder.get(node);
   }

   protected abstract FootstepNodeSnapData snapInternal(FootstepNode footstepNode);
}
