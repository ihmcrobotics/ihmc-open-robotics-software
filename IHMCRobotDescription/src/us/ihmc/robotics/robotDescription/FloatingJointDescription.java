package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

public class FloatingJointDescription extends JointDescription
{
   public FloatingJointDescription(String name)
   {
      super(name, new Vector3d());
   }

}

