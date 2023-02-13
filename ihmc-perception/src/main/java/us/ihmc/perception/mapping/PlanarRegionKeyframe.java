package us.ihmc.perception.mapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionKeyframe
{
   private PlanarRegionsList planarRegionsList;
   private RigidBodyTransform transformToWorld;

   public PlanarRegionKeyframe(PlanarRegionsList planarRegionsList, RigidBodyTransform transformToWorld)
   {
      this.planarRegionsList = planarRegionsList;
      this.transformToWorld = transformToWorld;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
   }
}
