package us.ihmc.robotics.robotDescription;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class ForceSensorDescription extends SensorDescription
{
   private boolean useGroundContactPoints = true;

   public ForceSensorDescription(String name, RigidBodyTransform transformToJoint)
   {
      super(name, transformToJoint);
   }

   public boolean useGroundContactPoints()
   {
      return useGroundContactPoints;
   }

   public void setUseGroundContactPoints(boolean useGroundContactPoints)
   {
      this.useGroundContactPoints = useGroundContactPoints;
   }

}
