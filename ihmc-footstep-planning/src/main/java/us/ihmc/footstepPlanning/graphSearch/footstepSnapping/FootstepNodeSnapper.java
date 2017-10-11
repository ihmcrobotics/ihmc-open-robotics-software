package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.HashMap;

public abstract class FootstepNodeSnapper
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
      if(planarRegionsList == null)
      {
         return FootstepNodeSnapData.identityData();
      }

      if(!snapDataHolder.containsKey(footstepNode))
      {
         FootstepNodeSnapData snapData = snapInternal(footstepNode);
         snapDataHolder.put(footstepNode, snapData);
      }

      return snapDataHolder.get(footstepNode);
   }

   /**
    * Helper method so the first node is artificially snapped, since it usually isn't
    * on a planar region.
    *
    * @param footstepNode
    * @return
    */
   public void addStartNode(FootstepNode footstepNode)
   {
      snapDataHolder.put(footstepNode, new FootstepNodeSnapData(new RigidBodyTransform(), new ConvexPolygon2D()));
   }

   protected abstract FootstepNodeSnapData snapInternal(FootstepNode footstepNode);
}
