package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;

public class IntermediateDesiredJointDataHolder
{
   private final JointDesiredOutputList estimatorDataHolder;
   private final JointDesiredOutputList controllerDataHolder;

   private final JointDesiredOutput[] intermediateDesiredJointData;

   public IntermediateDesiredJointDataHolder(JointDesiredOutputList estimatorDataHolder, JointDesiredOutputList controllerDataHolder)
   {
      this.estimatorDataHolder = estimatorDataHolder;
      this.controllerDataHolder = controllerDataHolder;

      for(int i = 0; i < estimatorDataHolder.getNumberOfJointsWithDesiredOutput(); i++)
      {
         if(estimatorDataHolder.getOneDoFJoint(i).getName() != controllerDataHolder.getOneDoFJoint(i).getName())
         {
            throw new RuntimeException("Models do not match");
         }
      }
      intermediateDesiredJointData = new JointDesiredOutput[estimatorDataHolder.getNumberOfJointsWithDesiredOutput()];
      for(int i = 0; i < estimatorDataHolder.getNumberOfJointsWithDesiredOutput(); i++)
      {
         intermediateDesiredJointData[i] = new JointDesiredOutput();
      }
   }

   public void copyFromController()
   {
      for(int i = 0; i < controllerDataHolder.getNumberOfJointsWithDesiredOutput(); i++)
      {
         intermediateDesiredJointData[i].set(controllerDataHolder.getJointDesiredOutput(i));
      }

   }

   public void readIntoEstimator()
   {
      for(int i = 0; i < estimatorDataHolder.getNumberOfJointsWithDesiredOutput(); i++)
      {
         estimatorDataHolder.getJointDesiredOutput(i).set(intermediateDesiredJointData[i]);
      }
   }
}
