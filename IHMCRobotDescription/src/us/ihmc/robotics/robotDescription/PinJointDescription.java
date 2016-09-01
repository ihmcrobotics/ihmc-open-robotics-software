package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;

public class PinJointDescription extends OneDoFJointDescription
{

   public PinJointDescription(String name, Vector3d offsetFromParentJoint, Axis jointAxis)
   {
      super(name, offsetFromParentJoint, jointAxis);
   }

   public PinJointDescription(String name, Vector3d offset, Vector3d jointAxis)
   {
      super(name, offset, jointAxis);
   }


}
