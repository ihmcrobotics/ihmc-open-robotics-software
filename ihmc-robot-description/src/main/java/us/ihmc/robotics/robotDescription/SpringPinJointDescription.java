package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;

public class SpringPinJointDescription extends OneDoFJointDescription
{

   public SpringPinJointDescription(String name, Vector3D offsetFromParentJoint, Axis jointAxis)
   {
      super(name, offsetFromParentJoint, jointAxis);
   }

   public SpringPinJointDescription(String name, Vector3D offset, Vector3D jointAxis)
   {
      super(name, offset, jointAxis);
   }

}
