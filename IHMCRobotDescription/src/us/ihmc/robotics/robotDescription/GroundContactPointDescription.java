package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

public class GroundContactPointDescription extends ExternalForcePointDescription
{
   public GroundContactPointDescription(String name, Vector3d offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }
}
