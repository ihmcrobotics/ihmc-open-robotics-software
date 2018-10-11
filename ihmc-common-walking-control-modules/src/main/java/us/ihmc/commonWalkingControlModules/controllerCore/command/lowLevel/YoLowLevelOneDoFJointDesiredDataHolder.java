package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.tools.string.StringTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Map;

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

      for (int otherIndex = 0; otherIndex < other.getNumberOfJointsWithDesiredOutput(); otherIndex++)
      {
         OneDoFJoint joint = other.getOneDoFJoint(otherIndex);
         YoJointDesiredOutput lowLevelJointData = lowLevelJointDataMap.get(joint.getNameBasedHashCode());
         if (lowLevelJointData == null)
            continue;

         lowLevelJointData.set(other.getJointDesiredOutput(otherIndex));
      }
   }

   static void throwJointNotRegisteredException(OneDoFJoint joint)
   {
      throw new RuntimeException("The joint: " + joint.getName() + " has not been registered.");
   }

   @Override
   public boolean hasDataForJoint(OneDoFJoint joint)
   {
      return lowLevelJointDataMap.containsKey(joint.getNameBasedHashCode());
   }

   @Override
   public OneDoFJoint getOneDoFJoint(int index)
   {
      return jointsWithDesiredData[index];
   }



   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return jointsWithDesiredData.length;
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(OneDoFJoint joint)
   {
      return getJointDesiredOutput(joint.getNameBasedHashCode());
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(long jointName)
   {
      return lowLevelJointDataMap.get(jointName);
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(int index)
   {
      return lowLevelJointDataList[index];
   }
}
