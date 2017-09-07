package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import us.ihmc.sensorProcessing.outputData.LowLevelJointData;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;

public class IntermediateDesiredJointDataHolder
{
   private final LowLevelOneDoFJointDesiredDataHolderList estimatorDataHolder;
   private final LowLevelOneDoFJointDesiredDataHolderList controllerDataHolder;
   
   private final LowLevelJointData[] intermediateDesiredJointData;
   
   public IntermediateDesiredJointDataHolder(LowLevelOneDoFJointDesiredDataHolderList estimatorDataHolder, LowLevelOneDoFJointDesiredDataHolderList controllerDataHolder)
   {
      this.estimatorDataHolder = estimatorDataHolder;
      this.controllerDataHolder = controllerDataHolder;
      
      for(int i = 0; i < estimatorDataHolder.getNumberOfJointsWithLowLevelData(); i++)
      {
         if(estimatorDataHolder.getOneDoFJoint(i).getName() != controllerDataHolder.getOneDoFJoint(i).getName())
         {
            throw new RuntimeException("Models do not match");
         }
      }
      intermediateDesiredJointData = new LowLevelJointData[estimatorDataHolder.getNumberOfJointsWithLowLevelData()];
      for(int i = 0; i < estimatorDataHolder.getNumberOfJointsWithLowLevelData(); i++)
      {
         intermediateDesiredJointData[i] = new LowLevelJointData();
      }      
   }
   
   public void copyFromController()
   {
      for(int i = 0; i < controllerDataHolder.getNumberOfJointsWithLowLevelData(); i++)
      {
         intermediateDesiredJointData[i].set(controllerDataHolder.getLowLevelJointData(i));
      }
      
   }

   public void readIntoEstimator()
   {
      for(int i = 0; i < estimatorDataHolder.getNumberOfJointsWithLowLevelData(); i++)
      {
         estimatorDataHolder.getLowLevelJointData(i).set(intermediateDesiredJointData[i]);
      }
   }
}
