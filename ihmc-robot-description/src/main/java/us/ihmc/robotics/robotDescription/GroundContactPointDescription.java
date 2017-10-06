package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.Vector3D;

public class GroundContactPointDescription extends ExternalForcePointDescription
{
   public GroundContactPointDescription(String name, Vector3D offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }
}
