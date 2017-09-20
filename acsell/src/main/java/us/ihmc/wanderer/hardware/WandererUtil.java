package us.ihmc.wanderer.hardware;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;

public class WandererUtil
{
   public static EnumMap<WandererJoint, OneDoFJoint> createJointMap(OneDoFJoint[] jointList)
   {
      return createJointMap(Arrays.asList(jointList));
   }
   
   public static EnumMap<WandererJoint, OneDoFJoint> createJointMap(List<OneDoFJoint> jointList)
   {
      EnumMap<WandererJoint, OneDoFJoint> wandererJoints = new EnumMap<>(WandererJoint.class);
      for (OneDoFJoint oneDoFJoint : jointList)
      {
         for (WandererJoint joint : WandererJoint.values)
         {
            if (joint.getSdfName().equals(oneDoFJoint.getName()))
            {
               wandererJoints.put(joint, oneDoFJoint);
            }
         }
      }
      return wandererJoints;
   }
   
   public static EnumMap<WandererJoint, LowLevelJointData> createOutputMap(LowLevelOneDoFJointDesiredDataHolderList lowLevelOutuput)
   {
      EnumMap<WandererJoint, LowLevelJointData> wandererJoints = new EnumMap<>(WandererJoint.class);
      for (int i = 0; i < lowLevelOutuput.getNumberOfJointsWithLowLevelData(); i++)
      {
         for (WandererJoint joint : WandererJoint.values)
         {
            if (joint.getSdfName().equals(lowLevelOutuput.getJointName(i)))
            {
               wandererJoints.put(joint, lowLevelOutuput.getLowLevelJointData(i));
            }
         }
      }
      return wandererJoints;
   }
}
