package us.ihmc.perception.mapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegionsList;

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
