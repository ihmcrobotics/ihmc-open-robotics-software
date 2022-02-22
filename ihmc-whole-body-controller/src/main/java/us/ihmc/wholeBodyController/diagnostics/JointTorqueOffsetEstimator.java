package us.ihmc.wholeBodyController.diagnostics;

import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public interface JointTorqueOffsetEstimator
{
   public List<OneDoFJointBasics> getOneDoFJoints();

   public double getEstimatedJointTorqueOffset(OneDoFJointBasics joint);

   public default void resetEstimatedJointTorqueOffsets()
   {
      List<OneDoFJointBasics> joints = getOneDoFJoints();
      for (int i = 0; i < joints.size(); i++)
         resetEstimatedJointTorqueOffset(joints.get(i));
   }

   public void resetEstimatedJointTorqueOffset(OneDoFJointBasics joint);

   public boolean hasTorqueOffsetForJoint(OneDoFJointBasics joint);
}
