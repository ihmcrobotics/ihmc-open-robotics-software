package us.ihmc.perception.mapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionKeyframe
{
   private RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private RigidBodyTransform transformToPrevious = new RigidBodyTransform();

   public PlanarRegionKeyframe(RigidBodyTransform transformToPrevious, RigidBodyTransform previousToWorldTransform)
   {
      this.transformToPrevious.set(transformToPrevious);

      this.transformToWorld.set(transformToPrevious);
      this.transformToWorld.multiply(previousToWorldTransform);
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
   }

   public RigidBodyTransform getTransformToPrevious()
   {
      return transformToPrevious;
   }
}
