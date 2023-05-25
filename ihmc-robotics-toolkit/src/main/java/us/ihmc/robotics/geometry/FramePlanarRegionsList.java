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

   /**
    * Use this method to change the frame of the planar regions in the list to world frame.
    * Warning! Don't call this more than once. This method currently is a one-time conversion
    * to world frame and you can't undo it.
    */
   public void changeFrameToWorld()
   {
      planarRegionsList.applyTransform(sensorToWorldFrameTransform);
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
}
