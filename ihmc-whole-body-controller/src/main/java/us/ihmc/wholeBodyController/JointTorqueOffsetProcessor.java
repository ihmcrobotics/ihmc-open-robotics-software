package us.ihmc.wholeBodyController;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public interface JointTorqueOffsetProcessor
{
   public abstract void subtractTorqueOffset(OneDoFJointBasics oneDoFJoint, double torqueOffset);
}
