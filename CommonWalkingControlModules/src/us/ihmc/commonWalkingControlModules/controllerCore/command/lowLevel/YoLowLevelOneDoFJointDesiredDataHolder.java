package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder.throwJointNotRegisteredException;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.tools.string.StringTools;

public class YoLowLevelOneDoFJointDesiredDataHolder implements LowLevelOneDoFJointDesiredDataHolderReadOnly
{
   private final List<OneDoFJoint> jointsWithDesiredData;
   private final Map<String, YoLowLevelJointData> lowLevelJointDataMap;

   public YoLowLevelOneDoFJointDesiredDataHolder(OneDoFJoint[] controlledJoints, YoVariableRegistry parentRegistry)
   {

      YoVariableRegistry registry = new YoVariableRegistry("LowLevelOneDoFJointDesiredData" + parentRegistry.getName());
      parentRegistry.addChild(registry);

      int capacity = controlledJoints.length;
      jointsWithDesiredData = new ArrayList<>(capacity);
      lowLevelJointDataMap = new HashMap<>(capacity);

      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJoint joint = controlledJoints[i];
         jointsWithDesiredData.add(joint);
         String jointName = joint.getName();
         YoLowLevelJointData jointData = new YoLowLevelJointData(jointName, registry, StringTools.getEveryUppercaseLetter(parentRegistry.getName()));
         lowLevelJointDataMap.put(jointName, jointData);
      }
   }

   public void clear()
   {
      for (int i = 0; i < jointsWithDesiredData.size(); i++)
      {
         OneDoFJoint joint = jointsWithDesiredData.get(i);
         YoLowLevelJointData jointDataToReset = lowLevelJointDataMap.get(joint.getName());
         jointDataToReset.clear();
      }
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < jointsWithDesiredData.size(); i++)
      {
         jointsWithDesiredData.set(i, nameToJointMap.get(jointsWithDesiredData.get(i).getName()));
      }
   }

   public void setJointControlMode(OneDoFJoint joint, LowLevelJointControlMode controlMode)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(controlMode);
   }

   public void setDesiredJointTorque(OneDoFJoint joint, double desiredTorque)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   public void setupForForceControl(OneDoFJoint joint, double desiredTorque)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(LowLevelJointControlMode.FORCE_CONTROL);
      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   public void setDesiredJointPosition(OneDoFJoint joint, double desiredPosition)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredPosition(desiredPosition);
   }

   public void setDesiredJointVelocity(OneDoFJoint joint, double desiredVelocity)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   public void setDesiredJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredAcceleration(desiredAcceleration);
   }

   public void setDesiredJointCurrent(OneDoFJoint joint, double desiredCurrent)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredCurrent(desiredCurrent);
   }

   public void setupForPositionControl(OneDoFJoint joint, double desiredPosition, double desiredVelocity)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(LowLevelJointControlMode.POSITION_CONTROL);
      lowLevelJointData.setDesiredPosition(desiredPosition);
      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   public void setResetJointIntegrators(OneDoFJoint joint, boolean reset)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setResetIntegrators(reset);
   }

   public void overwriteJointData(OneDoFJoint joint, LowLevelJointDataReadOnly jointData)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.set(jointData);
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(LowLevelOneDoFJointDesiredDataHolderReadOnly other)
   {
      for (int i = 0; i < other.getNumberOfJointsWithLowLevelData(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);
         String jointName = joint.getName();

         YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(jointName);
         LowLevelJointDataReadOnly otherLowLevelJointData = other.getLowLevelJointData(joint);

         if (lowLevelJointData == null)
            throwJointNotRegisteredException(joint);
         lowLevelJointData.completeWith(otherLowLevelJointData);
      }
   }

   /**
    * Clear this and copy the data held in other.
    */
   public void overwriteWith(LowLevelOneDoFJointDesiredDataHolderReadOnly other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfJointsWithLowLevelData(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);
         YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
         if (lowLevelJointData == null)
            continue;

         lowLevelJointData.set(other.getLowLevelJointData(joint));
      }
   }

   public void insertDesiredTorquesIntoOneDoFJoints(OneDoFJoint[] oneDoFJoints)
   {
      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setTau(lowLevelJointDataMap.get(joint.getName()).getDesiredTorque());
      }
   }

   public YoLowLevelJointData getYoLowLevelJointData(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.get(joint.getName());
   }

   @Override
   public LowLevelJointControlMode getJointControlMode(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getControlMode();
   }

   @Override
   public double getDesiredJointTorque(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredTorque();
   }

   @Override
   public double getDesiredJointPosition(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredPosition();
   }

   @Override
   public double getDesiredJointVelocity(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredVelocity();
   }

   @Override
   public double getDesiredJointAcceleration(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredAcceleration();
   }

   @Override
   public double getDesiredJointCurrent(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredCurrent();
   }

   @Override
   public boolean pollResetJointIntegrators(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.pollResetIntegratorsRequest();
   }

   @Override
   public boolean peekResetJointIntegrators(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.peekResetIntegratorsRequest();
   }

   @Override
   public boolean hasDataForJoint(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.containsKey(joint.getName());
   }

   @Override
   public boolean hasControlModeForJoint(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasControlMode();
   }

   @Override
   public boolean hasDesiredTorqueForJoint(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredTorque();
   }

   @Override
   public boolean hasDesiredPositionForJoint(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredPosition();
   }

   @Override
   public boolean hasDesiredVelocityForJoint(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredVelocity();
   }

   @Override
   public boolean hasDesiredAcceleration(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredAcceleration();
   }

   @Override
   public boolean hasDesiredCurrentForJoint(OneDoFJoint joint)
   {
      YoLowLevelJointData lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredCurrent();
   }

   @Override
   public OneDoFJoint getOneDoFJoint(int index)
   {
      return jointsWithDesiredData.get(index);
   }

   @Override
   public LowLevelJointDataReadOnly getLowLevelJointData(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.get(joint.getName());
   }

   @Override
   public int getNumberOfJointsWithLowLevelData()
   {
      return jointsWithDesiredData.size();
   }
}
