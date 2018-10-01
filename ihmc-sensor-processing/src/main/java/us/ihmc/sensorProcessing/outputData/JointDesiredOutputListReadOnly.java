package us.ihmc.sensorProcessing.outputData;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface JointDesiredOutputListReadOnly
{
   boolean hasDataForJoint(OneDoFJoint joint);

   OneDoFJoint getOneDoFJoint(int index);

   JointDesiredOutputReadOnly getJointDesiredOutput(OneDoFJoint joint);

   JointDesiredOutputReadOnly getJointDesiredOutput(int index);

   int getNumberOfJointsWithDesiredOutput();

   default void throwJointNotRegisteredException(OneDoFJoint joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has not been registered.");
   }

   default void throwJointNotRegisteredException(int index)
   {
      throw new RuntimeException("The joint: " + getOneDoFJoint(index).getName() + " has not been registered.");
   }


   default JointDesiredControlMode getJointControlMode(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getControlMode();
   }

   default double getDesiredJointTorque(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredTorque();
   }

   default double getDesiredJointPosition(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredPosition();
   }

   default double getDesiredJointVelocity(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredVelocity();
   }

   default double getDesiredJointAcceleration(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredAcceleration();
   }



   default JointDesiredControlMode getJointControlMode(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);
      return lowLevelJointData.getControlMode();
   }

   default double getDesiredJointTorque(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);
      return lowLevelJointData.getDesiredTorque();
   }

   default double getDesiredJointPosition(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);
      return lowLevelJointData.getDesiredPosition();
   }

   default double getDesiredJointVelocity(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);
      return lowLevelJointData.getDesiredVelocity();
   }

   default double getDesiredJointAcceleration(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(index);
      return lowLevelJointData.getDesiredAcceleration();
   }

   default boolean peekResetJointIntegrators(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
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

   default boolean pollResetJointIntegrators(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
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



   default boolean hasControlModeForJoint(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasControlMode();
   }

   default boolean hasControlModeForJoint(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasControlMode();
   }

   default boolean hasDesiredTorqueForJoint(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredTorque();
   }

   default boolean hasDesiredTorqueForJoint(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredTorque();
   }

   default boolean hasDesiredPositionForJoint(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredPosition();
   }

   default boolean hasDesiredPositionForJoint(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredPosition();
   }

   default boolean hasDesiredVelocityForJoint(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
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

   default boolean hasDesiredAcceleration(OneDoFJoint joint)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(joint);
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredAcceleration();
   }

   default boolean hasDesiredAcceleration(int index)
   {
      JointDesiredOutputReadOnly lowLevelJointData = getJointDesiredOutput(index);
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredAcceleration();
   }
}