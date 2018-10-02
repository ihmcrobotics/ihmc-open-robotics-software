package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public interface JointDesiredOutputListBasics extends JointDesiredOutputListReadOnly
{

   JointDesiredOutputBasics getJointDesiredOutput(OneDoFJoint joint);
   JointDesiredOutputBasics getJointDesiredOutput(long jointName);
   JointDesiredOutputBasics getJointDesiredOutput(int index);

   default void clear()
   {
      for (int jointIdx = 0; jointIdx < getNumberOfJointsWithDesiredOutput(); jointIdx++)
      {
         getJointDesiredOutput(jointIdx).clear();
      }
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   default void completeWith(JointDesiredOutputListReadOnly other)
   {
      for (int otherJointIndex = 0; otherJointIndex < other.getNumberOfJointsWithDesiredOutput(); otherJointIndex++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(otherJointIndex);
         long jointName = joint.getNameBasedHashCode();

         JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(jointName);
         JointDesiredOutputReadOnly otherLowLevelJointData = other.getJointDesiredOutput(otherJointIndex);

         if (lowLevelJointData == null)
            throwJointNotRegisteredException(joint);
         lowLevelJointData.completeWith(otherLowLevelJointData);
      }
   }

   /**
    * Clear this and copy the data held in other.
    */
   default void overwriteWith(JointDesiredOutputListReadOnly other)
   {
      clear();

      for (int otherJointIndex = 0; otherJointIndex < other.getNumberOfJointsWithDesiredOutput(); otherJointIndex++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(otherJointIndex);
         JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
         if (lowLevelJointData == null)
            continue;

         lowLevelJointData.set(other.getJointDesiredOutput(otherJointIndex));
      }
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
      for (int jointIdx = 0; jointIdx < getNumberOfJointsWithDesiredOutput(); jointIdx++)
      {
         getJointDesiredOutput(jointIdx).setResetIntegrators(true);
      }
   }


   default void setJointControlMode(OneDoFJoint joint, JointDesiredControlMode controlMode)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(controlMode);
   }

   default void setJointControlMode(int index, JointDesiredControlMode controlMode)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);

      lowLevelJointData.setControlMode(controlMode);
   }

   default void setDesiredJointTorque(OneDoFJoint joint, double desiredTorque)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   default void setDesiredJointTorque(int index, double desiredTorque)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);

      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   default void setDesiredJointPosition(OneDoFJoint joint, double desiredPosition)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredPosition(desiredPosition);
   }

   default void setDesiredJointPosition(int index, double desiredPosition)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);

      lowLevelJointData.setDesiredPosition(desiredPosition);
   }

   default void setDesiredJointVelocity(OneDoFJoint joint, double desiredVelocity)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   default void setDesiredJointVelocity(int index, double desiredVelocity)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);

      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   default void setDesiredJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredAcceleration(desiredAcceleration);
   }


   default void setDesiredJointAcceleration(int index, double desiredAcceleration)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);

      lowLevelJointData.setDesiredAcceleration(desiredAcceleration);
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






   default void overwriteJointData(OneDoFJoint joint, JointDesiredOutputReadOnly jointData)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.set(jointData);
   }

   default void overwriteJointData(int index, JointDesiredOutputReadOnly jointData)
   {
      JointDesiredOutputBasics lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);

      lowLevelJointData.set(jointData);
   }

}
