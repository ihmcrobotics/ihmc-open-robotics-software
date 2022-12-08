package us.ihmc.robotics.geometry;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;

public class PlanarRegionsListWithPose
{
   private PlanarRegionsList planarRegionsList;
   private RigidBodyTransform sensorToWorldFrameTransform;

   public PlanarRegionsListWithPose(PlanarRegionsList planarRegionsList, RigidBodyTransform sensorToWorldFrameTransform)
   {
      this.planarRegionsList = planarRegionsList;
      this.sensorToWorldFrameTransform = sensorToWorldFrameTransform;
   }

   public PlanarRegionsListWithPose()
   {
      this.sensorToWorldFrameTransform = new RigidBodyTransform();
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
