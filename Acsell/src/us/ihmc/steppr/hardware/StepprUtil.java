package us.ihmc.steppr.hardware;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.robotics.screwTheory.OneDoFJoint;

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
}
