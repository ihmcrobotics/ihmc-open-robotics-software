package us.ihmc.robotics.screwTheory;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PassiveRevoluteJoint extends RevoluteJoint
{

   /**
    * In this type of joint:
    * 
    *    1)  You can NOT set Q, Qd, or Tau --> because they are not actuated joints, 
    *    in other words they cannot be controlled because their motion is determined 
    *    by that of another non-passive joint.
    *    
    *    2) getTau() should always return a zero because, since the joint is NOT actuated,
    *    there is no torque.
    */
   public PassiveRevoluteJoint(String name, RigidBody predecessor, ReferenceFrame beforeJointFrame, FrameVector jointAxis)
   {
      super(name, predecessor, beforeJointFrame, jointAxis);

   }

}
