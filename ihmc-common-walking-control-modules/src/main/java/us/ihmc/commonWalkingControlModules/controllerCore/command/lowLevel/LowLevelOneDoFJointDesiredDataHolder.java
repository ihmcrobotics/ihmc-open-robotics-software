package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;

public class LowLevelOneDoFJointDesiredDataHolder implements JointDesiredOutputListReadOnly
{
   private final List<JointDesiredOutput> unusedData;
   private final List<OneDoFJoint> jointsWithDesiredData;
   private final TLongObjectHashMap<JointDesiredOutput> lowLevelJointDataMap;

   public LowLevelOneDoFJointDesiredDataHolder()
   {
      this(50);
   }

   public LowLevelOneDoFJointDesiredDataHolder(int initialCapacity)
   {
      unusedData = new ArrayList<>(initialCapacity);

      while (unusedData.size() < initialCapacity)
         unusedData.add(new JointDesiredOutput());

      jointsWithDesiredData = new ArrayList<>(initialCapacity);

      /**
       * A autoCompactionFactor of 0 disables auto-compacting, which ensures no garbage
       * is created by emptying and filling the map repeatedly. @dcalvert
       */
      float disableAutoCompaction = 0;
      lowLevelJointDataMap = new TLongObjectHashMap<JointDesiredOutput>(initialCapacity);
      lowLevelJointDataMap.setAutoCompactionFactor(disableAutoCompaction);
   }

   public void clear()
   {
      while (!jointsWithDesiredData.isEmpty())
      {
         OneDoFJoint lastJoint = jointsWithDesiredData.remove(jointsWithDesiredData.size() - 1);
         JointDesiredOutput jointDataToReset = lowLevelJointDataMap.remove(lastJoint.getNameBasedHashCode());
         jointDataToReset.clear();
         unusedData.add(jointDataToReset);
      }
   }

   public void registerJointsWithEmptyData(OneDoFJoint[] joint)
   {
      for (int i = 0; i < joint.length; i++)
         registerJointWithEmptyData(joint[i]);
   }

   public JointDesiredOutput registerJointWithEmptyData(OneDoFJoint joint)
   {
      if (lowLevelJointDataMap.containsKey(joint.getNameBasedHashCode()))
         throwJointAlreadyRegisteredException(joint);

      return registerJointWithEmptyDataUnsafe(joint);
   }

   public void registerLowLevelJointData(OneDoFJoint joint, JointDesiredOutputReadOnly jointDataToRegister)
   {
      if (lowLevelJointDataMap.containsKey(joint.getNameBasedHashCode()))
         throwJointAlreadyRegisteredException(joint);

      registerLowLevelJointDataUnsafe(joint, jointDataToRegister);
   }

   private void registerLowLevelJointDataUnsafe(OneDoFJoint joint, JointDesiredOutputReadOnly jointDataToRegister)
   {
      JointDesiredOutput jointData = registerJointWithEmptyDataUnsafe(joint);
      jointData.set(jointDataToRegister);
   }

   private JointDesiredOutput registerJointWithEmptyDataUnsafe(OneDoFJoint joint)
   {
      JointDesiredOutput jointData;

      if (unusedData.isEmpty())
         jointData = new JointDesiredOutput();
      else
         jointData = unusedData.remove(unusedData.size() - 1);
      lowLevelJointDataMap.put(joint.getNameBasedHashCode(), jointData);
      jointsWithDesiredData.add(joint);
      return jointData;
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < jointsWithDesiredData.size(); i++)
      {
         jointsWithDesiredData.set(i, nameToJointMap.get(jointsWithDesiredData.get(i).getName()));
      }
   }

   public void setJointsControlMode(OneDoFJoint[] joints, JointDesiredControlMode controlMode)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setJointControlMode(joint, controlMode);
      }
   }

   public void setDesiredTorqueFromJoints(List<OneDoFJoint> joints)
   {
      for (int i = 0; i < joints.size(); i++)
      {
         OneDoFJoint joint = joints.get(i);
         setDesiredJointTorque(joint, joint.getTau());
      }
   }

   public void setDesiredTorqueFromJoints(OneDoFJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setDesiredJointTorque(joint, joint.getTau());
      }
   }

   public void setDesiredPositionFromJoints(OneDoFJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setDesiredJointPosition(joint, joint.getqDesired());
      }
   }

   public void setDesiredVelocityFromJoints(OneDoFJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setDesiredJointVelocity(joint, joint.getQdDesired());
      }
   }

   public void setDesiredAccelerationFromJoints(OneDoFJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setDesiredJointAcceleration(joint, joint.getQddDesired());
      }
   }

   public void setJointControlMode(OneDoFJoint joint, JointDesiredControlMode controlMode)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(controlMode);
   }

   public void setDesiredJointTorque(OneDoFJoint joint, double desiredTorque)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   public void setupForForceControl(OneDoFJoint joint, double desiredTorque)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(JointDesiredControlMode.EFFORT);
      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   public void setDesiredJointPosition(OneDoFJoint joint, double desiredPosition)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredPosition(desiredPosition);
   }

   public void setDesiredJointVelocity(OneDoFJoint joint, double desiredVelocity)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   public void setDesiredJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredAcceleration(desiredAcceleration);
   }

   public void setupForPositionControl(OneDoFJoint joint, double desiredPosition, double desiredVelocity)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(JointDesiredControlMode.POSITION);
      lowLevelJointData.setDesiredPosition(desiredPosition);
      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   public void setResetJointIntegrators(OneDoFJoint joint, boolean reset)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setResetIntegrators(reset);
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(JointDesiredOutputListReadOnly other)
   {
      if (other == null)
         return;

      for (int i = 0; i < other.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);

         JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
         JointDesiredOutputReadOnly otherLowLevelJointData = other.getJointDesiredOutput(joint);

         if (lowLevelJointData != null)
         {
            lowLevelJointData.completeWith(otherLowLevelJointData);
         }
         else
         {
            registerLowLevelJointDataUnsafe(joint, otherLowLevelJointData);
         }
      }
   }

   /**
    * Clear this and copy the data held in other.
    */
   public void overwriteWith(JointDesiredOutputListReadOnly other)
   {
      clear();

      if (other == null)
         return;

      for (int i = 0; i < other.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);
         registerLowLevelJointDataUnsafe(joint, other.getJointDesiredOutput(joint));
      }
   }

   public JointDesiredControlMode getJointControlMode(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getControlMode();
   }

   public double getDesiredJointTorque(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredTorque();
   }

   public double getDesiredJointPosition(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredPosition();
   }

   public double getDesiredJointVelocity(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredVelocity();
   }

   public double getDesiredJointAcceleration(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredAcceleration();
   }

   public boolean pollResetJointIntegrators(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.pollResetIntegratorsRequest();
   }

   public boolean peekResetJointIntegrators(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.peekResetIntegratorsRequest();
   }

   @Override
   public boolean hasDataForJoint(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.containsKey(joint.getNameBasedHashCode());
   }

   public boolean hasControlModeForJoint(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasControlMode();
   }

   public boolean hasDesiredTorqueForJoint(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredTorque();
   }

   public boolean hasDesiredPositionForJoint(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredPosition();
   }

   public boolean hasDesiredVelocityForJoint(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredVelocity();
   }

   public boolean hasDesiredAcceleration(OneDoFJoint joint)
   {
      JointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredAcceleration();
   }

   static void throwJointAlreadyRegisteredException(OneDoFJoint joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has already been registered.");
   }

   static void throwJointNotRegisteredException(OneDoFJoint joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has not been registered.");
   }

   @Override
   public OneDoFJoint getOneDoFJoint(int index)
   {
      return jointsWithDesiredData.get(index);
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.get(joint.getNameBasedHashCode());
   }

   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return jointsWithDesiredData.size();
   }

   @Override
   public JointDesiredOutput getJointDesiredOutput(int index)
   {
      return getJointDesiredOutput(getOneDoFJoint(index));
   }
}
