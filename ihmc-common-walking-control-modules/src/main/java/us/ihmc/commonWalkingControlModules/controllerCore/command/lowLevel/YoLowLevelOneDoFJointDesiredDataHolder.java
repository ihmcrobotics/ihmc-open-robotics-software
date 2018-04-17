package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import static us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder.throwJointNotRegisteredException;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.tools.string.StringTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoLowLevelOneDoFJointDesiredDataHolder implements JointDesiredOutputListReadOnly
{
   private final List<OneDoFJoint> jointsWithDesiredData;
   private final Map<String, YoJointDesiredOutput> lowLevelJointDataMap;

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
         YoJointDesiredOutput jointData = new YoJointDesiredOutput(jointName, registry, StringTools.getEveryUppercaseLetter(parentRegistry.getName()));
         lowLevelJointDataMap.put(jointName, jointData);
      }
   }

   public void clear()
   {
      for (int i = 0; i < jointsWithDesiredData.size(); i++)
      {
         OneDoFJoint joint = jointsWithDesiredData.get(i);
         YoJointDesiredOutput jointDataToReset = lowLevelJointDataMap.get(joint.getName());
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

   public void setJointControlMode(OneDoFJoint joint, JointDesiredControlMode controlMode)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(controlMode);
   }

   public void setDesiredJointTorque(OneDoFJoint joint, double desiredTorque)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   public void setupForForceControl(OneDoFJoint joint, double desiredTorque)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(JointDesiredControlMode.EFFORT);
      lowLevelJointData.setDesiredTorque(desiredTorque);
   }

   public void setDesiredJointPosition(OneDoFJoint joint, double desiredPosition)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredPosition(desiredPosition);
   }

   public void setDesiredJointVelocity(OneDoFJoint joint, double desiredVelocity)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   public void setDesiredJointAcceleration(OneDoFJoint joint, double desiredAcceleration)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setDesiredAcceleration(desiredAcceleration);
   }

   public void setupForPositionControl(OneDoFJoint joint, double desiredPosition, double desiredVelocity)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setControlMode(JointDesiredControlMode.POSITION);
      lowLevelJointData.setDesiredPosition(desiredPosition);
      lowLevelJointData.setDesiredVelocity(desiredVelocity);
   }

   public void setResetJointIntegrators(OneDoFJoint joint, boolean reset)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.setResetIntegrators(reset);
   }

   public void overwriteJointData(OneDoFJoint joint, JointDesiredOutputReadOnly jointData)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);

      lowLevelJointData.set(jointData);
   }

   /**
    * Complete the information held in this using other.
    * Does not overwrite the data already set in this.
    */
   public void completeWith(JointDesiredOutputListReadOnly other)
   {
      for (int i = 0; i < other.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);
         String jointName = joint.getName();

         YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(jointName);
         JointDesiredOutputReadOnly otherLowLevelJointData = other.getJointDesiredOutput(joint);

         if (lowLevelJointData == null)
            throwJointNotRegisteredException(joint);
         lowLevelJointData.completeWith(otherLowLevelJointData);
      }
   }

   /**
    * Clear this and copy the data held in other.
    */
   public void overwriteWith(JointDesiredOutputListReadOnly other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(i);
         YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
         if (lowLevelJointData == null)
            continue;

         lowLevelJointData.set(other.getJointDesiredOutput(joint));
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

   public YoJointDesiredOutput getYoLowLevelJointData(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.get(joint.getName());
   }

   public JointDesiredControlMode getJointControlMode(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getControlMode();
   }

   public double getDesiredJointTorque(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredTorque();
   }

   public double getDesiredJointPosition(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredPosition();
   }

   public double getDesiredJointVelocity(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredVelocity();
   }

   public double getDesiredJointAcceleration(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.getDesiredAcceleration();
   }

   public boolean pollResetJointIntegrators(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.pollResetIntegratorsRequest();
   }

   public boolean peekResetJointIntegrators(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         throwJointNotRegisteredException(joint);
      return lowLevelJointData.peekResetIntegratorsRequest();
   }

   @Override
   public boolean hasDataForJoint(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.containsKey(joint.getName());
   }

   public boolean hasControlModeForJoint(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasControlMode();
   }

   public boolean hasDesiredTorqueForJoint(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredTorque();
   }

   public boolean hasDesiredPositionForJoint(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredPosition();
   }

   public boolean hasDesiredVelocityForJoint(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredVelocity();
   }

   public boolean hasDesiredAcceleration(OneDoFJoint joint)
   {
      YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getName());
      if (lowLevelJointData == null)
         return false;
      else
         return lowLevelJointData.hasDesiredAcceleration();
   }

   @Override
   public OneDoFJoint getOneDoFJoint(int index)
   {
      return jointsWithDesiredData.get(index);
   }

   @Override
   public JointDesiredOutputReadOnly getJointDesiredOutput(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.get(joint.getName());
   }

   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return jointsWithDesiredData.size();
   }

   @Override
   public JointDesiredOutputReadOnly getJointDesiredOutput(int index)
   {
      return getJointDesiredOutput(getOneDoFJoint(index));
   }
}
