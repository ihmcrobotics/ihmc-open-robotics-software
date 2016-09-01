package us.ihmc.robotics.robotDescription;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.lidar.LidarScanParameters;

public class LidarSensorDescription
{
   private String name;
   private RigidBodyTransform linkToSensorInZUp = new RigidBodyTransform();
   private LidarScanParameters lidarScanParameters;

   public LidarSensorDescription(RigidBodyTransform linkToSensorInZUp, LidarScanParameters polarDefinition, String name)
   {
      this.name = name;

      this.linkToSensorInZUp.set(linkToSensorInZUp);
      this.lidarScanParameters = lidarScanParameters;
   }

}
