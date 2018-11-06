package us.ihmc.wholeBodyController.diagnostics;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.OneDoFJoint;

public interface JointTorqueOffsetEstimator
{
   public List<OneDoFJoint> getOneDoFJoints();

   public double getEstimatedJointTorqueOffset(OneDoFJoint joint);

   public void resetEstimatedJointTorqueOffset(OneDoFJoint joint);

   public boolean hasTorqueOffsetForJoint(OneDoFJoint joint);
}
