package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.lidar.LidarScanParameters;

public class LidarSensorDescription extends SensorDescription
{
   private LidarScanParameters lidarScanParameters;

   public LidarSensorDescription(String name, RigidBodyTransform transformToJoint, LidarScanParameters lidarScanParameters)
   {
      super(name, transformToJoint);
      this.setLidarScanParameters(lidarScanParameters);
   }

   public LidarScanParameters getLidarScanParameters()
   {
      return lidarScanParameters;
   }

   public void setLidarScanParameters(LidarScanParameters lidarScanParameters)
   {
      this.lidarScanParameters = lidarScanParameters;
   }

}
