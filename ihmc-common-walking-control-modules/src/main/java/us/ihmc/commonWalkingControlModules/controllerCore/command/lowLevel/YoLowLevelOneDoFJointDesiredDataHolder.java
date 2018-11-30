package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import gnu.trove.map.hash.TLongObjectHashMap;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.tools.string.StringTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoLowLevelOneDoFJointDesiredDataHolder implements JointDesiredOutputListBasics
{
   private final OneDoFJointBasics[] jointsWithDesiredData;
   private final YoJointDesiredOutput[] lowLevelJointDataList;
   private final TLongObjectHashMap<YoJointDesiredOutput> lowLevelJointDataMap;

   public YoLowLevelOneDoFJointDesiredDataHolder(OneDoFJointBasics[] controlledJoints, YoVariableRegistry parentRegistry)
   {
      this("", controlledJoints, parentRegistry);
   }

   public YoLowLevelOneDoFJointDesiredDataHolder(String prefix, OneDoFJointBasics[] controlledJoints, YoVariableRegistry parentRegistry)
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
         OneDoFJointBasics joint = controlledJoints[i];
         YoJointDesiredOutput jointData = new YoJointDesiredOutput(joint.getName(), registry, StringTools.getEveryUppercaseLetter(parentRegistry.getName()));
         lowLevelJointDataList[i] = jointData;
         lowLevelJointDataMap.put(joint.hashCode(), jointData);
      }
   }

   @Override
   public boolean hasDataForJoint(OneDoFJointBasics joint)
   {
      return lowLevelJointDataMap.containsKey(joint.hashCode());
   }

   @Override
   public OneDoFJointBasics getOneDoFJoint(int index)
   {
      return jointsWithDesiredData[index];
   }

   @Override
   public int getNumberOfJointsWithDesiredOutput()
   {
      return jointsWithDesiredData.length;
   }

   @Override
   public JointDesiredOutputBasics getJointDesiredOutput(OneDoFJointBasics joint)
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
