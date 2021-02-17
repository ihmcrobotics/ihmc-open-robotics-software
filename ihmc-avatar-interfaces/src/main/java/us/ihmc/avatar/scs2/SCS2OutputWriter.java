package us.ihmc.avatar.scs2;

import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SCS2OutputWriter implements JointDesiredOutputWriter
{
   private final ControllerOutput controllerOutput;
   private final boolean writeBeforeEstimatorTick;
   private final PairList<OneDoFJointStateBasics, JointDesiredOutputReadOnly> jointSimInputList = new PairList<>();

   public SCS2OutputWriter(ControllerOutput controllerOutput, boolean writeBeforeEstimatorTick)
   {
      this.controllerOutput = controllerOutput;
      this.writeBeforeEstimatorTick = writeBeforeEstimatorTick;
   }

   @Override
   public void setJointDesiredOutputList(JointDesiredOutputListBasics jointDesiredOutputList)
   {
      jointSimInputList.clear();

      for (int i = 0; i < jointDesiredOutputList.getNumberOfJointsWithDesiredOutput(); i++)
      {
         jointSimInputList.add(controllerOutput.getOneDoFJointOutput(jointDesiredOutputList.getOneDoFJoint(i)),
                               jointDesiredOutputList.getJointDesiredOutput(i));
      }
   }

   private void write()
   {
      for (int i = 0; i < jointSimInputList.size(); i++)
      {
         OneDoFJointStateBasics jointState = jointSimInputList.get(i).getLeft();
         JointDesiredOutputReadOnly jointDesiredState = jointSimInputList.get(i).getRight();

         if (jointDesiredState.hasDesiredTorque())
         {
            jointState.setEffort(jointDesiredState.getDesiredTorque());
         }

         if (jointDesiredState.hasStiffness())
         {
            //            jointState.setKp(jointDesiredState.getStiffness());
         }
         if (jointDesiredState.hasDamping())
         {
            //            jointState.setKd(jointDesiredState.getDamping());
         }
         if (jointDesiredState.hasDesiredPosition())
         {
            //            jointState.setqDesired(jointDesiredState.getDesiredPosition());
         }
         if (jointDesiredState.hasDesiredVelocity())
         {
            //            jointState.setQdDesired(jointDesiredState.getDesiredVelocity());
         }
      }
   }

   @Override
   public void writeBefore(long timestamp)
   {
      if (writeBeforeEstimatorTick)
      {
         write();
      }
   }

   @Override
   public void writeAfter()
   {
      if (!writeBeforeEstimatorTick)
      {
         write();
      }
   }

   @Override
   public YoRegistry getYoVariableRegistry()
   {
      return null;
   }
}
