package us.ihmc.valkyrieRosControl.upperBody;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.rosControl.EffortJointHandle;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.valkyrieRosControl.ValkyrieTorqueOffsetPrinter;

import java.util.Map;

public class ValkyrieUpperBodyOutputWriter
{
   private static final Map<String, Double> torqueOffsets = ValkyrieTorqueOffsetPrinter.loadTorqueOffsetsFromFile();

   static
   {
      if (torqueOffsets == null)
      {
         throw new RuntimeException("Torque offsets file could not load");
      }
   }

   private final OneDoFJointBasics[] controlledJoints;
   private final JointDesiredOutputListBasics jointDesiredOutputList;
   private final Map<String, EffortJointHandle> nameToEffortHandleMap;

   public ValkyrieUpperBodyOutputWriter(OneDoFJointBasics[] controlledJoints,
                                        JointDesiredOutputListBasics jointDesiredOutputList,
                                        Map<String, EffortJointHandle> nameToEffortHandleMap)
   {
      this.controlledJoints = controlledJoints;
      this.jointDesiredOutputList = jointDesiredOutputList;
      this.nameToEffortHandleMap = nameToEffortHandleMap;
   }

   public void write()
   {
      for (int i = 0; i < controlledJoints.length; i++)
      {
         OneDoFJointBasics joint = controlledJoints[i];
         String jointName = joint.getName();
         JointDesiredOutputBasics jointDesiredOutput = jointDesiredOutputList.getJointDesiredOutput(joint);
         EffortJointHandle effortJointHandle = nameToEffortHandleMap.get(jointName);
         Double torqueOffset = torqueOffsets.get(jointName);
         if (torqueOffset == null)
            continue;

         double desiredTorque = 0.0;
         if (jointDesiredOutput.hasDesiredTorque())
            desiredTorque = jointDesiredOutput.getDesiredTorque();
         effortJointHandle.setDesiredEffort(-torqueOffset + desiredTorque);
      }
   }
}