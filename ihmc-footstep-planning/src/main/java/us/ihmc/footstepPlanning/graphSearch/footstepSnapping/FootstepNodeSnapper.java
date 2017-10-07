package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public abstract class FootstepNodeSnapper
{
   private final FootstepNodeSnapDataHolder snapDataHolder = new FootstepNodeSnapDataHolder();

   public abstract void setPlanarRegions(PlanarRegionsList planarRegionsList);

   public RigidBodyTransform snapFootstepNode(FootstepNode footstepNode, ConvexPolygon2D footholdIntersectionToPack)
   {
      if(!snapDataHolder.containsKey(footstepNode))
      {
         RigidBodyTransform transform = snapInternal(footstepNode, footholdIntersectionToPack);
         snapDataHolder.put(footstepNode, new FootstepNodeSnapData(transform, footholdIntersectionToPack));
      }

      FootstepNodeSnapData snapData = snapDataHolder.get(footstepNode);
      return snapData.getSnapTransform();
   }

   protected abstract RigidBodyTransform snapInternal(FootstepNode footstepNode, ConvexPolygon2D footholdIntersectionToPack);
}
