package us.ihmc.sensorProcessing.outputData;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public interface JointDesiredOutputListBasics extends JointDesiredOutputListReadOnly
{
   default void clear()
   {
      for (int i = 0; i < getNumberOfJointsWithDesiredOutput(); i++)
         getJointDesiredOutput(i).clear();
   }

   /**
    * Clear this and copy the data held in other.
    */
   default void overwriteWith(JointDesiredOutputListReadOnly other)
   {
      clear();

      for (int otherIndex = 0; otherIndex < other.getNumberOfJointsWithDesiredOutput(); otherIndex++)
      {
         OneDoFJointBasics otherJoint = other.getOneDoFJoint(otherIndex);
         JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutputFromHash(otherJoint.hashCode());

         if (jointDesiredOutput == null)
            continue;

         jointDesiredOutput.set(other.getJointDesiredOutput(otherIndex));
      }
   }
   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   default void completeWith(JointDesiredOutputListReadOnly other)
   {
      for (int otherIndex = 0; otherIndex < other.getNumberOfJointsWithDesiredOutput(); otherIndex++)
      {
         OneDoFJointBasics otherJoint = other.getOneDoFJoint(otherIndex);
         JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutputFromHash(otherJoint.hashCode());

         if (jointDesiredOutput == null)
            throwJointNotRegisteredException(otherJoint);

         jointDesiredOutput.completeWith(other.getJointDesiredOutput(otherIndex));
      }
   }

   default void setJointControlMode(OneDoFJointBasics joint, JointDesiredControlMode controlMode)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);

      jointDesiredOutput.setControlMode(controlMode);
   }

   default void setJointsControlMode(OneDoFJointBasics[] joints, JointDesiredControlMode controlMode)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJointBasics joint = joints[i];
         setJointControlMode(joint, controlMode);
      }
   }

   default void requestIntegratorReset()
   {
      for (int i = 0; i < getNumberOfJointsWithDesiredOutput(); i++)
         getJointDesiredOutput(i).setResetIntegrators(true);
   }

   default void setJointControlMode(int index, JointDesiredControlMode controlMode)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(index);

      jointDesiredOutput.setControlMode(controlMode);
   }

   default void setDesiredJointTorque(OneDoFJointBasics joint, double desiredTorque)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);

      jointDesiredOutput.setDesiredTorque(desiredTorque);
   }

   default void setDesiredJointTorque(int index, double desiredTorque)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(index);

      jointDesiredOutput.setDesiredTorque(desiredTorque);
   }

   default void setDesiredJointPosition(OneDoFJointBasics joint, double desiredPosition)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);

      jointDesiredOutput.setDesiredPosition(desiredPosition);
   }

   default void setDesiredJointPosition(int index, double desiredPosition)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(index);

      jointDesiredOutput.setDesiredPosition(desiredPosition);
   }

   default void setDesiredJointVelocity(OneDoFJointBasics joint, double desiredVelocity)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);

      jointDesiredOutput.setDesiredVelocity(desiredVelocity);
   }

   default void setDesiredJointVelocity(int index, double desiredVelocity)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(index);

      jointDesiredOutput.setDesiredVelocity(desiredVelocity);
   }

   default void setDesiredJointAcceleration(OneDoFJointBasics joint, double desiredAcceleration)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutputFromHash(joint.hashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);

      jointDesiredOutput.setDesiredAcceleration(desiredAcceleration);
   }

   default void setDesiredJointAcceleration(int index, double desiredAcceleration)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(index);
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(index);

      jointDesiredOutput.setDesiredAcceleration(desiredAcceleration);
   }

   default void setupForForceControl(OneDoFJointBasics joint, double desiredTorque)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutputFromHash(joint.hashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(JointDesiredControlMode.EFFORT);
      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   default void setupForForceControl(int index, double desiredTorque)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);

      lowLevelJointData.setControlMode(JointDesiredControlMode.EFFORT);
      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   default void setupForPositionControl(OneDoFJointBasics joint, double desiredPosition, double desiredVelocity)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutputFromHash(joint.hashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(JointDesiredControlMode.POSITION);
      lowLevelJointData.setDesiredPosition(desiredPosition);
      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   default void setupForPositionControl(int index, double desiredPosition, double desiredVelocity)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);

      lowLevelJointData.setControlMode(JointDesiredControlMode.POSITION);
      lowLevelJointData.setDesiredPosition(desiredPosition);
      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   default void setResetJointIntegrators(OneDoFJointBasics joint, boolean reset)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutputFromHash(joint.hashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setResetIntegrators(reset);
   }

   default void setResetJointIntegrators(int index, boolean reset)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);

      lowLevelJointData.setResetIntegrators(reset);
   }



   @Override
   JointDesiredOutputBasics getJointDesiredOutput(int index);

   @Override
   JointDesiredOutputBasics getJointDesiredOutput(OneDoFJointBasics joint);

   @Override
   JointDesiredOutputBasics getJointDesiredOutputFromHash(int jointHashCode);

   static void throwJointNotRegisteredException(OneDoFJointBasics joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has not been registered.");
   }

   static void throwJointNotRegisteredException(int index)
   {
      throw new RuntimeException("The joint index : " + index + " has not been registered.");
   }
}
