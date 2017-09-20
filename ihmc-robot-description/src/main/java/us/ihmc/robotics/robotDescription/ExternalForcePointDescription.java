package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.Vector3D;

public class ExternalForcePointDescription extends KinematicPointDescription
{

   public ExternalForcePointDescription(String name, Vector3D offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

}
