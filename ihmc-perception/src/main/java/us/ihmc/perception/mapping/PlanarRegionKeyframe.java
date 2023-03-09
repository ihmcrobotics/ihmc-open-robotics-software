package us.ihmc.perception.mapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegionsList;

/**
 * This class is used to store a keyframe of a planar region map. It contains the planar regions list, the transform to the previous keyframe,
 * and the transform to the world frame. It also contains the time index of the keyframe. It may be extended to store more information for loop closures
 * in the future.
 */

public class PlanarRegionKeyframe
{
   private int timeIndex = 0;

   private PlanarRegionsList planarRegionsList = new PlanarRegionsList();
   private RigidBodyTransform transformToWorld = new RigidBodyTransform();
   private RigidBodyTransform transformToPrevious = new RigidBodyTransform();

   public PlanarRegionKeyframe(int index, RigidBodyTransform transformToPrevious, RigidBodyTransform previousToWorldTransform, PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList.addPlanarRegionsList(planarRegionsList);
      this.timeIndex = index;
      this.transformToPrevious.set(transformToPrevious);

      this.transformToWorld.set(transformToPrevious);
      this.transformToWorld.multiply(previousToWorldTransform);
   }

   public PlanarRegionKeyframe(int index, RigidBodyTransform transformToWorld, PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList.addPlanarRegionsList(planarRegionsList);
      this.timeIndex = index;
      this.transformToPrevious.set(transformToPrevious);

      this.transformToWorld.set(transformToWorld);
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
   }

   public RigidBodyTransform getTransformToPrevious()
   {
      return transformToPrevious;
   }

   public int getTimeIndex()
   {
      return timeIndex;
   }
}
