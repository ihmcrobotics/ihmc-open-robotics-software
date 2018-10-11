package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface JointDesiredOutputListReadOnly
{
   boolean hasDataForJoint(OneDoFJoint joint);

   OneDoFJoint getOneDoFJoint(int index);

   JointDesiredOutputReadOnly getJointDesiredOutput(OneDoFJoint joint);

   JointDesiredOutputReadOnly getJointDesiredOutput(long jointName);

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

   default boolean hasControlModeForJoint(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default JointDesiredControlMode getJointControlMode(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default boolean hasDesiredTorqueForJoint(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default double getDesiredJointTorque(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default boolean hasDesiredPositionForJoint(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default double getDesiredJointPosition(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default boolean hasDesiredVelocityForJoint(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default double getDesiredJointVelocity(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default boolean hasDesiredAcceleration(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default double getDesiredJointAcceleration(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default boolean pollResetJointIntegrators(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default boolean peekResetJointIntegrators(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   static void throwJointNotRegisteredException(OneDoFJoint joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has not been registered.");
   }

   static void throwJointNotRegisteredException(int index)
   {
      throw new RuntimeException("The joint index : " + index + " has not been registered.");
   }
}