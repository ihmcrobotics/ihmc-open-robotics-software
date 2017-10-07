package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class FootstepNodeSnapper
{
   private final FootstepNodeSnapDataHolder snapDataHolder = new FootstepNodeSnapDataHolder();

   public abstract void setPlanarRegions(PlanarRegionsList planarRegionsList);

   public FootstepNodeSnapData snapFootstepNode(FootstepNode footstepNode)
   {
      if(!snapDataHolder.containsKey(footstepNode))
      {
         FootstepNodeSnapData snapData = snapInternal(footstepNode);
         snapDataHolder.put(footstepNode, snapData);
      }

      return snapDataHolder.get(footstepNode);
   }

   protected abstract FootstepNodeSnapData snapInternal(FootstepNode footstepNode);
}
