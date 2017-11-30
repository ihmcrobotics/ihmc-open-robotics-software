package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface JointDesiredOutputListReadOnly
{
   public abstract boolean hasDataForJoint(OneDoFJoint joint);

   public abstract OneDoFJoint getOneDoFJoint(int index);

   public abstract JointDesiredOutputReadOnly getJointDesiredOutput(OneDoFJoint joint);

   public abstract JointDesiredOutputReadOnly getJointDesiredOutput(int index);

   public abstract int getNumberOfJointsWithDesiredOutput();

}