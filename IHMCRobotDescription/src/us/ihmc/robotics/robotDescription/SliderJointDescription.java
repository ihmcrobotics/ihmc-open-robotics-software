package us.ihmc.robotics.robotDescription;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.Axis;

public class SliderJointDescription extends OneDoFJointDescription
{

   public SliderJointDescription(String name, Vector3d offsetFromParentJoint, Axis jointAxis)
   {
      super(name, offsetFromParentJoint, jointAxis);
   }

   public SliderJointDescription(String name, Vector3d offset, Vector3d jointAxis)
   {
      super(name, offset, jointAxis);
   }
}
