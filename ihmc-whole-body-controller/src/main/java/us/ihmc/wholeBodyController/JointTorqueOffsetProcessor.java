package us.ihmc.wholeBodyController;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface JointTorqueOffsetProcessor
{
   public abstract void subtractTorqueOffset(OneDoFJoint oneDoFJoint, double torqueOffset);
}
