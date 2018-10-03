package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.*;
import us.ihmc.tools.string.StringTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoLowLevelOneDoFJointDesiredDataHolder implements JointDesiredOutputListBasics
{
   private final OneDoFJoint[] jointsWithDesiredData;
   private final YoJointDesiredOutput[] lowLevelJointDataList;
   private final TLongObjectHashMap<YoJointDesiredOutput> lowLevelJointDataMap;

   public YoLowLevelOneDoFJointDesiredDataHolder(OneDoFJoint[] controlledJoints, YoVariableRegistry parentRegistry)
   {
      this("", controlledJoints, parentRegistry);
   }

   public YoLowLevelOneDoFJointDesiredDataHolder(String prefix, OneDoFJoint[] controlledJoints, YoVariableRegistry parentRegistry)
   {
      YoVariableRegistry registry = new YoVariableRegistry(prefix + "LowLevelOneDoFJointDesiredData" + parentRegistry.getName());
      parentRegistry.addChild(registry);

      int capacity = controlledJoints.length;
      jointsWithDesiredData = controlledJoints;
      lowLevelJointDataList = new YoJointDesiredOutput[capacity];

      float disableAutoCompaction = 0;
      lowLevelJointDataMap = new TLongObjectHashMap<>(capacity);
      lowLevelJointDataMap.setAutoCompactionFactor(disableAutoCompaction);


      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJoint joint = controlledJoints[i];
         YoJointDesiredOutput jointData = new YoJointDesiredOutput(joint.getName(), registry, StringTools.getEveryUppercaseLetter(parentRegistry.getName()));
         lowLevelJointDataList[i] = jointData;
         lowLevelJointDataMap.put(joint.getNameBasedHashCode(), jointData);
      }
   }

   @Override
   public void clear()
   {
      for (YoJointDesiredOutput jointDataToReset : lowLevelJointDataList)
         jointDataToReset.clear();
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < jointsWithDesiredData.length; i++)
      {
         jointsWithDesiredData[i] = nameToJointMap.get(jointsWithDesiredData[i].getName());
      }
   }

   public void setJointControlMode(OneDoFJoint joint, JointDesiredControlMode controlMode)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(controlMode);
   }

   public void setDesiredJointTorque(OneDoFJoint joint, double desiredTorque)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   public void setupForForceControl(OneDoFJoint joint, double desiredTorque)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(JointDesiredControlMode.EFFORT);
      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   public void setDesiredJointPosition(OneDoFJoint joint, double desiredPosition)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredPosition(desiredPosition);
   }

   public void setDesiredJointVelocity(OneDoFJoint joint, double desiredVelocity)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   public void setDesiredJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredAcceleration(desiredAcceleration);
   }

   public void setupForPositionControl(OneDoFJoint joint, double desiredPosition, double desiredVelocity)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(JointDesiredControlMode.POSITION);
      lowLevelJointData.setDesiredPosition(desiredPosition);
      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   public void setResetJointIntegrators(OneDoFJoint joint, boolean reset)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setResetIntegrators(reset);
   }

   public void overwriteJointData(OneDoFJoint joint, JointDesiredOutputReadOnly jointData)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.set(jointData);
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   @Override
   public void completeWith(JointDesiredOutputListReadOnly other)
   {
      for (int i = 0; i < other.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);

         YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
         JointDesiredOutputReadOnly otherLowLevelJointData = other.getJointDesiredOutput(joint);

         if (lowLevelJointData == null)
            throwJointNotRegisteredException(joint);
         lowLevelJointData.completeWith(otherLowLevelJointData);
      }
   }

   /**
    * Clear this and copy the data held in other.
    */
   @Override
   public void overwriteWith(JointDesiredOutputListReadOnly other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);
         YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
         if (lowLevelJointData == null)
            continue;

         lowLevelJointData.set(other.getJointDesiredOutput(joint));
      }
   }

   public YoJointDesiredOutput getYoLowLevelJointData(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.get(joint.getNameBasedHashCode());
   }

   public JointDesiredControlMode getJointControlMode(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getControlMode();
   }

   public double getDesiredJointTorque(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredTorque();
   }

   public double getDesiredJointPosition(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredPosition();
   }

   public double getDesiredJointVelocity(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredVelocity();
   }

   public double getDesiredJointAcceleration(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredAcceleration();
   }

   public boolean pollResetJointIntegrators(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.pollResetIntegratorsRequest();
   }

   public boolean peekResetJointIntegrators(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
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
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasControlMode();
   }

   public boolean hasDesiredTorqueForJoint(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredTorque();
   }

   public boolean hasDesiredPositionForJoint(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredPosition();
   }

   public boolean hasDesiredVelocityForJoint(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredVelocity();
   }

   public boolean hasDesiredAcceleration(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredAcceleration();
   }

   @Override
   public OneDoFJoint getOneDoFJoint(int index)
   {
      return jointsWithDesiredData[index];
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.get(joint.getNameBasedHashCode());
   }

   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return jointsWithDesiredData.length;
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(int index)
   {
      return lowLevelJointDataList[index];
   }
}
