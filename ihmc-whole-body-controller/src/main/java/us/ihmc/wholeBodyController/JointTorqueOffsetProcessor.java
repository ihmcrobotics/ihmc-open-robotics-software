package us.ihmc.wholeBodyController;

import us.ihmc.mecano.multiBodySystem.OneDoFJoint;

public interface JointTorqueOffsetProcessor
{
   public abstract void subtractTorqueOffset(OneDoFJoint oneDoFJoint, double torqueOffset);
}
