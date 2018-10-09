package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface JointDesiredOutputListReadOnly
{
   boolean hasDataForJoint(OneDoFJoint joint);

   OneDoFJoint getOneDoFJoint(int index);

   JointDesiredOutputReadOnly getJointDesiredOutput(OneDoFJoint joint);

   JointDesiredOutputReadOnly getJointDesiredOutput(int index);

   int getNumberOfJointsWithDesiredOutput();

   default void insertDesiredTorquesIntoOneDoFJoints(OneDoFJoint[] oneDoFJoints)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint);
         if (jointDesiredOutput != null)
            joint.setTau(jointDesiredOutput.getDesiredTorque());
      }
   }
}