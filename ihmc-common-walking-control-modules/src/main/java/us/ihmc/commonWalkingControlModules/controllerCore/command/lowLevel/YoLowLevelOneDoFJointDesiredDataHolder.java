package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
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
