package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class LowLevelOneDoFJointDesiredDataHolder implements LowLevelOneDoFJointDesiredDataHolderReadOnly
{
   private final List<LowLevelJointData> unusedData;
   private final List<OneDoFJoint> jointsWithDesiredData;
   private final TLongObjectHashMap<LowLevelJointData> lowLevelJointDataMap;

   public LowLevelOneDoFJointDesiredDataHolder()
   {
      this(50);
   }

   public LowLevelOneDoFJointDesiredDataHolder(int initialCapacity)
   {
      unusedData = new ArrayList<>(initialCapacity);

      while (unusedData.size() < initialCapacity)
         unusedData.add(new LowLevelJointData());

      jointsWithDesiredData = new ArrayList<>(initialCapacity);
      
      /**
       * A autoCompactionFactor of 0 disables auto-compacting, which ensures no garbage
       * is created by emptying and filling the map repeatedly. @dcalvert
       */
      float disableAutoCompaction = 0;
      lowLevelJointDataMap = new TLongObjectHashMap<LowLevelJointData>(initialCapacity);
      lowLevelJointDataMap.setAutoCompactionFactor(disableAutoCompaction);
   }

   public void clear()
   {
      while (!jointsWithDesiredData.isEmpty())
      {
         OneDoFJoint lastJoint = jointsWithDesiredData.remove(jointsWithDesiredData.size() - 1);
         LowLevelJointData jointDataToReset = lowLevelJointDataMap.remove(lastJoint.nameBasedHashCode());
         jointDataToReset.clear();
         unusedData.add(jointDataToReset);
      }
   }

   public void registerJointsWithEmptyData(OneDoFJoint[] joint)
   {
      for (int i = 0; i < joint.length; i++)
         registerJointWithEmptyData(joint[i]);
   }

   public void registerJointWithEmptyData(OneDoFJoint joint)
   {
      if (lowLevelJointDataMap.containsKey(joint.nameBasedHashCode()))
         throwJointAlreadyRegisteredException(joint);

      registerJointWithEmptyDataUnsafe(joint);
   }

   public void registerLowLevelJointData(OneDoFJoint joint, LowLevelJointDataReadOnly jointDataToRegister)
   {
      if (lowLevelJointDataMap.containsKey(joint.nameBasedHashCode()))
         throwJointAlreadyRegisteredException(joint);

      registerLowLevelJointDataUnsafe(joint, jointDataToRegister);
   }

   private void registerLowLevelJointDataUnsafe(OneDoFJoint joint, LowLevelJointDataReadOnly jointDataToRegister)
   {
      LowLevelJointData jointData = registerJointWithEmptyDataUnsafe(joint);
      jointData.set(jointDataToRegister);
   }

   private LowLevelJointData registerJointWithEmptyDataUnsafe(OneDoFJoint joint)
   {
      LowLevelJointData jointData;

      if (unusedData.isEmpty())
         jointData = new LowLevelJointData();
      else
         jointData = unusedData.remove(unusedData.size() - 1);
      lowLevelJointDataMap.put(joint.nameBasedHashCode(), jointData);
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

   public void extractAllDataFromJoints(OneDoFJoint[] joints, LowLevelJointControlMode controlMode)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setJointControlMode(joint, controlMode);
         setDesiredJointTorque(joint, joint.getTau());
         setDesiredJointPosition(joint, joint.getqDesired());
         setDesiredJointVelocity(joint, joint.getQdDesired());
         setDesiredJointAcceleration(joint, joint.getQddDesired());
         setResetJointIntegrators(joint, joint.getResetDesiredAccelerationIntegrator());
      }
   }

   public void setJointsControlMode(OneDoFJoint[] joints, LowLevelJointControlMode controlMode)
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

   public void setResetIntegratorsromJoints(OneDoFJoint[] joints)
   {
      for (int i = 0; i < joints.length; i++)
      {
         OneDoFJoint joint = joints[i];
         setResetJointIntegrators(joint, joint.getResetDesiredAccelerationIntegrator());
      }
   }

   public void setJointControlMode(OneDoFJoint joint, LowLevelJointControlMode controlMode)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(controlMode);
   }

   public void setDesiredJointTorque(OneDoFJoint joint, double desiredTorque)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   public void setupForForceControl(OneDoFJoint joint, double desiredTorque)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(LowLevelJointControlMode.FORCE_CONTROL);
      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   public void setDesiredJointPosition(OneDoFJoint joint, double desiredPosition)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredPosition(desiredPosition);
   }

   public void setDesiredJointVelocity(OneDoFJoint joint, double desiredVelocity)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   public void setDesiredJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredAcceleration(desiredAcceleration);
   }

   public void setDesiredJointCurrent(OneDoFJoint joint, double desiredCurrent)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredCurrent(desiredCurrent);
   }

   public void setupForPositionControl(OneDoFJoint joint, double desiredPosition, double desiredVelocity)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(LowLevelJointControlMode.POSITION_CONTROL);
      lowLevelJointData.setDesiredPosition(desiredPosition);
      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   public void setResetJointIntegrators(OneDoFJoint joint, boolean reset)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setResetIntegrators(reset);
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(LowLevelOneDoFJointDesiredDataHolderReadOnly other)
   {
      if (other == null)
         return;

      for (int i = 0; i < other.getNumberOfJointsWithLowLevelData(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);

         LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
         LowLevelJointDataReadOnly otherLowLevelJointData = other.getLowLevelJointData(joint);

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
   public void overwriteWith(LowLevelOneDoFJointDesiredDataHolderReadOnly other)
   {
      clear();

      if (other == null)
         return;

      for (int i = 0; i < other.getNumberOfJointsWithLowLevelData(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);
         registerLowLevelJointDataUnsafe(joint, other.getLowLevelJointData(joint));
      }
   }

   @Override
   public LowLevelJointControlMode getJointControlMode(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getControlMode();
   }

   @Override
   public double getDesiredJointTorque(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredTorque();
   }

   @Override
   public double getDesiredJointPosition(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredPosition();
   }

   @Override
   public double getDesiredJointVelocity(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredVelocity();
   }

   @Override
   public double getDesiredJointAcceleration(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredAcceleration();
   }

   @Override
   public double getDesiredJointCurrent(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredCurrent();
   }

   @Override
   public boolean pollResetJointIntegrators(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.pollResetIntegratorsRequest();
   }

   @Override
   public boolean peekResetJointIntegrators(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.peekResetIntegratorsRequest();
   }

   @Override
   public boolean hasDataForJoint(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.containsKey(joint.nameBasedHashCode());
   }

   @Override
   public boolean hasControlModeForJoint(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasControlMode();
   }

   @Override
   public boolean hasDesiredTorqueForJoint(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredTorque();
   }

   @Override
   public boolean hasDesiredPositionForJoint(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredPosition();
   }

   @Override
   public boolean hasDesiredVelocityForJoint(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredVelocity();
   }

   @Override
   public boolean hasDesiredAcceleration(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredAcceleration();
   }

   @Override
   public boolean hasDesiredCurrentForJoint(OneDoFJoint joint)
   {
      LowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.nameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredCurrent();
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
   public LowLevelJointData getLowLevelJointData(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.get(joint.nameBasedHashCode());
   }

   @Override
   public int getNumberOfJointsWithLowLevelData()
   {
      return jointsWithDesiredData.size();
   }
}
