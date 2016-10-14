package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

public class ExternalForcePointDescription extends KinematicPointDescription
{

   public ExternalForcePointDescription(String name, Vector3d offsetFromJoint)
   {
      super(name, offsetFromJoint);
   }

}
