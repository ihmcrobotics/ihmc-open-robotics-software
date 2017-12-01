package us.ihmc.steppr.hardware;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;

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

   public static EnumMap<StepprJoint, JointDesiredOutput> createOutputMap(JointDesiredOutputList lowLevelOutuput)
   {
      EnumMap<StepprJoint, JointDesiredOutput> stepprJoints = new EnumMap<>(StepprJoint.class);
      for (int i = 0; i < lowLevelOutuput.getNumberOfJointsWithDesiredOutput(); i++)
      {
         for (StepprJoint joint : StepprJoint.values)
         {
            if (joint.getSdfName().equals(lowLevelOutuput.getJointName(i)))
            {
               stepprJoints.put(joint, lowLevelOutuput.getJointDesiredOutput(i));
            }
         }
      }
      return stepprJoints;
   }



}
