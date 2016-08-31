package us.ihmc.robotics.robotDescription;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class IMUSensorDescription extends SensorDescription
{

   public IMUSensorDescription(String name, RigidBodyTransform imuTransform)
   {
      super(name, imuTransform);
   }

}
