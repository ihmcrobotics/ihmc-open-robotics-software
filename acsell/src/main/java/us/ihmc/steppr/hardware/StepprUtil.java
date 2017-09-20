package us.ihmc.steppr.hardware;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;

public class StepprUtil
{
   public static EnumMap<StepprJoint, OneDoFJoint> createJointMap(OneDoFJoint[] jointList)
   {
      return createJointMap(Arrays.asList(jointList));
   }
   
   public static EnumMap<StepprJoint, OneDoFJoint> createJointMap(List<OneDoFJoint> jointList)
   {
      EnumMap<StepprJoint, OneDoFJoint> stepprJoints = new EnumMap<>(StepprJoint.class);
      for (OneDoFJoint oneDoFJoint : jointList)
      {
         for (StepprJoint joint : StepprJoint.values)
         {
            if (joint.getSdfName().equals(oneDoFJoint.getName()))
            {
               stepprJoints.put(joint, oneDoFJoint);
            }
         }
      }
      return stepprJoints;
   }

   public static EnumMap<StepprJoint, LowLevelJointData> createOutputMap(LowLevelOneDoFJointDesiredDataHolderList lowLevelOutuput)
   {
      EnumMap<StepprJoint, LowLevelJointData> stepprJoints = new EnumMap<>(StepprJoint.class);
      for (int i = 0; i < lowLevelOutuput.getNumberOfJointsWithLowLevelData(); i++)
      {
         for (StepprJoint joint : StepprJoint.values)
         {
            if (joint.getSdfName().equals(lowLevelOutuput.getJointName(i)))
            {
               stepprJoints.put(joint, lowLevelOutuput.getLowLevelJointData(i));
            }
         }
      }
      return stepprJoints;
   }
   
   
   
}
