package us.ihmc.sensorProcessing.outputData;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public interface JointDesiredOutputListReadOnly
{
   boolean hasDataForJoint(OneDoFJointBasics joint);

   OneDoFJointBasics getOneDoFJoint(int index);

   JointDesiredOutputReadOnly getJointDesiredOutput(OneDoFJointBasics joint);

   JointDesiredOutputReadOnly getJointDesiredOutputFromHash(int jointHashCode);

   JointDesiredOutputReadOnly getJointDesiredOutput(int index);

   int getNumberOfJointsWithDesiredOutput();

   default void insertDesiredTorquesIntoOneDoFJoints(OneDoFJointBasics[] oneDoFJoints)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJointBasics joint = oneDoFJoints[i];
         JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint);
         if (jointDesiredOutput != null)
            joint.setTau(jointDesiredOutput.getDesiredTorque());
      }
   }

   default boolean hasControlModeForJoint(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         return false;
      else
         return jointDesiredOutput.hasControlMode();
   }

   default boolean hasControlModeForJoint(int index)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         return false;
      else
         return jointDesiredOutput.hasControlMode();
   }

   default JointDesiredControlMode getJointControlMode(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);
      return jointDesiredOutput.getControlMode();
   }

   default JointDesiredControlMode getJointControlMode(int index)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(index);
      return jointDesiredOutput.getControlMode();
   }

   default boolean hasDesiredTorqueForJoint(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         return false;
      else
         return jointDesiredOutput.hasDesiredTorque();
   }

   default boolean hasDesiredTorqueForJoint(int index)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         return false;
      else
         return jointDesiredOutput.hasDesiredTorque();
   }

   default double getDesiredJointTorque(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);
      return jointDesiredOutput.getDesiredTorque();
   }

   default double getDesiredJointTorque(int index)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(index);
      return jointDesiredOutput.getDesiredTorque();
   }

   default boolean hasDesiredPositionForJoint(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         return false;
      else
         return jointDesiredOutput.hasDesiredPosition();
   }

   default boolean hasDesiredPositionForJoint(int index)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         return false;
      else
         return jointDesiredOutput.hasDesiredPosition();
   }

   default double getDesiredJointPosition(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);
      return jointDesiredOutput.getDesiredPosition();
   }

   default double getDesiredJointPosition(int index)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(index);
      return jointDesiredOutput.getDesiredPosition();
   }

   default boolean hasDesiredVelocityForJoint(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutputFromHash(joint.hashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredVelocity();
   }

   default boolean hasDesiredVelocityForJoint(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredVelocity();
   }

   default double getDesiredJointVelocity(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);
      return jointDesiredOutput.getDesiredVelocity();
   }

   default double getDesiredJointVelocity(int index)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(index);
      return jointDesiredOutput.getDesiredVelocity();
   }

   default boolean hasDesiredAcceleration(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         return false;
      else
         return jointDesiredOutput.hasDesiredAcceleration();
   }

   default boolean hasDesiredAcceleration(int index)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         return false;
      else
         return jointDesiredOutput.hasDesiredAcceleration();
   }

   default double getDesiredJointAcceleration(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);
      return jointDesiredOutput.getDesiredAcceleration();
   }

   default double getDesiredJointAcceleration(int index)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(index);
      return jointDesiredOutput.getDesiredAcceleration();
   }

   default boolean pollResetJointIntegrators(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutputFromHash(joint.hashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.pollResetIntegratorsRequest();
   }

   default boolean pollResetJointIntegrators(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);
      return lowLevelJointData.pollResetIntegratorsRequest();
   }

   default boolean peekResetJointIntegrators(OneDoFJointBasics joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutputFromHash(joint.hashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.peekResetIntegratorsRequest();
   }

   default boolean peekResetJointIntegrators(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);
      return lowLevelJointData.peekResetIntegratorsRequest();
   }

   static void throwJointNotRegisteredException(OneDoFJointBasics joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has not been registered.");
   }

   static void throwJointNotRegisteredException(int index)
   {
      throw new RuntimeException("The joint index : " + index + " has not been registered.");
   }

   default boolean equals(JointDesiredOutputListReadOnly other)
   {
      if (other == null)
      {
         return false;
      }
      else if (other == this)
      {
         return true;
      }
      else
      {
         if (getNumberOfJointsWithDesiredOutput() != other.getNumberOfJointsWithDesiredOutput())
            return false;
         for (int jointIndex = 0; jointIndex < getNumberOfJointsWithDesiredOutput(); jointIndex++)
         {
            OneDoFJointBasics joint = getOneDoFJoint(jointIndex);
            if (!getJointDesiredOutput(jointIndex).equals(other.getJointDesiredOutput(joint)))
               return false;
         }
         return true;
      }
   }
}