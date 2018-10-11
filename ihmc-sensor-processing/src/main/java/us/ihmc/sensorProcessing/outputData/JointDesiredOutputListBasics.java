package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

import java.util.List;

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
         OneDoFJoint otherJoint = other.getOneDoFJoint(otherIndex);
         JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(otherJoint.getNameBasedHashCode());

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
         OneDoFJoint otherJoint = other.getOneDoFJoint(otherIndex);
         JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(otherJoint.getNameBasedHashCode());

         if (jointDesiredOutput == null)
            throwJointNotRegisteredException(otherJoint);

         jointDesiredOutput.completeWith(other.getJointDesiredOutput(otherIndex));
      }
   }

   default void setJointControlMode(OneDoFJoint joint, JointDesiredControlMode controlMode)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
      if (jointDesiredOutput == null)
         throwJointNotRegisteredException(joint);

      jointDesiredOutput.setControlMode(controlMode);
   }

   default void setJointsControlMode(OneDoFJoint[] joints, JointDesiredControlMode controlMode)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setJointControlMode(joint, controlMode);
      }
   }

   default void setDesiredTorqueFromJoints(List<OneDoFJoint> joints)
   {
      for (int i = 0; i < joints.size(); i++)
      {
         OneDoFJoint joint = joints.get(i);
         setDesiredJointTorque(joint, joint.getTau());
      }
   }

   default void setDesiredTorqueFromJoints(OneDoFJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setDesiredJointTorque(joint, joint.getTau());
      }
   }

   default void setDesiredPositionFromJoints(OneDoFJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setDesiredJointPosition(joint, joint.getqDesired());
      }
   }

   default void setDesiredVelocityFromJoints(OneDoFJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setDesiredJointVelocity(joint, joint.getQdDesired());
      }
   }

   default void setDesiredAccelerationFromJoints(OneDoFJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setDesiredJointAcceleration(joint, joint.getQddDesired());
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

   default void setDesiredJointTorque(OneDoFJoint joint, double desiredTorque)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default void setDesiredJointPosition(OneDoFJoint joint, double desiredPosition)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default void setDesiredJointVelocity(OneDoFJoint joint, double desiredVelocity)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default void setDesiredJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      JointDesiredOutputBasics jointDesiredOutput = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default void setupForForceControl(OneDoFJoint joint, double desiredTorque)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default void setupForPositionControl(OneDoFJoint joint, double desiredPosition, double desiredVelocity)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
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

   default void setResetJointIntegrators(OneDoFJoint joint, boolean reset)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
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
   JointDesiredOutputBasics getJointDesiredOutput(OneDoFJoint joint);

   @Override
   JointDesiredOutputBasics getJointDesiredOutput(long jointName);

   static void throwJointNotRegisteredException(OneDoFJoint joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has not been registered.");
   }

   static void throwJointNotRegisteredException(int index)
   {
      throw new RuntimeException("The joint index : " + index + " has not been registered.");
   }
}
