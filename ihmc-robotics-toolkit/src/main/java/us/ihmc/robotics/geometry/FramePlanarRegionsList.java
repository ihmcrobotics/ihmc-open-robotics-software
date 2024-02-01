package us.ihmc.robotics.geometry;

import us.ihmc.euclid.transform.RigidBodyTransform;

public class FramePlanarRegionsList
{
   private PlanarRegionsList planarRegionsList;
   private RigidBodyTransform sensorToWorldFrameTransform;

   public FramePlanarRegionsList(PlanarRegionsList planarRegionsList, RigidBodyTransform sensorToWorldFrameTransform)
   {
      this.planarRegionsList = planarRegionsList;
      this.sensorToWorldFrameTransform = sensorToWorldFrameTransform;
   }

   public FramePlanarRegionsList()
   {
      this.sensorToWorldFrameTransform = new RigidBodyTransform();
      this.planarRegionsList = new PlanarRegionsList();
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList = planarRegionsList;
   }

   public RigidBodyTransform getSensorToWorldFrameTransform()
   {
      return sensorToWorldFrameTransform;
   }

   public void setSensorToWorldFrameTransform(RigidBodyTransform sensorToWorldFrameTransform)
   {
      this.sensorToWorldFrameTransform = sensorToWorldFrameTransform;
   }

   public FramePlanarRegionsList copy()
   {
      return new FramePlanarRegionsList(planarRegionsList.copy(), new RigidBodyTransform(sensorToWorldFrameTransform));
   }

   public void clear()
   {
      planarRegionsList.clear();
      sensorToWorldFrameTransform.setIdentity();
   }
}
