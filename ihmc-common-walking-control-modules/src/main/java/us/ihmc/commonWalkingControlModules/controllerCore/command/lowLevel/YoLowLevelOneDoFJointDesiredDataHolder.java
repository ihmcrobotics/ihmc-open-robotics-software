package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.tools.string.StringTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public class YoLowLevelOneDoFJointDesiredDataHolder implements JointDesiredOutputListBasics
{
   private final OneDoFJointReadOnly[] jointsWithDesiredData;
   private final YoJointDesiredOutput[] lowLevelJointDataList;
   private final TLongObjectHashMap<YoJointDesiredOutput> lowLevelJointDataMap;

   public YoLowLevelOneDoFJointDesiredDataHolder(OneDoFJointReadOnly[] controlledJoints, YoRegistry parentRegistry)
   {
      this("", controlledJoints, parentRegistry);
   }

   public YoLowLevelOneDoFJointDesiredDataHolder(String prefix, OneDoFJointReadOnly[] controlledJoints, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(prefix + "LowLevelOneDoFJointDesiredData" + parentRegistry.getName());
      parentRegistry.addChild(registry);

      int capacity = controlledJoints.length;
      jointsWithDesiredData = controlledJoints;
      lowLevelJointDataList = new YoJointDesiredOutput[capacity];

      float disableAutoCompaction = 0;
      lowLevelJointDataMap = new TLongObjectHashMap<>(capacity);
      lowLevelJointDataMap.setAutoCompactionFactor(disableAutoCompaction);


      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJointReadOnly joint = controlledJoints[i];
         YoJointDesiredOutput jointData = new YoJointDesiredOutput(joint.getName(), registry, StringTools.getEveryUppercaseLetter(parentRegistry.getName()));
         lowLevelJointDataList[i] = jointData;
         lowLevelJointDataMap.put(joint.hashCode(), jointData);
      }
   }

   @Override
   public boolean hasDataForJoint(OneDoFJointReadOnly joint)
   {
      return lowLevelJointDataMap.containsKey(joint.hashCode());
   }

   @Override
   public OneDoFJointReadOnly getOneDoFJoint(int index)
   {
      return jointsWithDesiredData[index];
   }

   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return jointsWithDesiredData.length;
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(OneDoFJointReadOnly joint)
   {
      return getJointDesiredOutputFromHash(joint.hashCode());
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutputFromHash(int jointHashCode)
   {
      return lowLevelJointDataMap.get(jointHashCode);
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(int index)
   {
      return lowLevelJointDataList[index];
   }
}
