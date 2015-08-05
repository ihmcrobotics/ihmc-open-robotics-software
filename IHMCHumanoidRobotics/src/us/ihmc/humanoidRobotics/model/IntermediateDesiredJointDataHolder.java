package us.ihmc.humanoidRobotics.model;

import us.ihmc.humanoidRobotics.model.DesiredJointDataHolder.DesiredJointData;

public class IntermediateDesiredJointDataHolder
{
   private final DesiredJointDataHolder estimatorDataHolder;
   private final DesiredJointDataHolder controllerDataHolder;
   
   private final DesiredJointData[] intermediateDesiredJointData;
   
   public IntermediateDesiredJointDataHolder(DesiredJointDataHolder estimatorDataHolder, DesiredJointDataHolder controllerDataHolder)
   {
      this.estimatorDataHolder = estimatorDataHolder;
      this.controllerDataHolder = controllerDataHolder;
      
      for(int i = 0; i < estimatorDataHolder.getJoints().length; i++)
      {
         if(estimatorDataHolder.getJoints()[i].getName() != controllerDataHolder.getJoints()[i].getName())
         {
            throw new RuntimeException("Models do not match");
         }
      }
      intermediateDesiredJointData = new DesiredJointData[estimatorDataHolder.getJoints().length];
      for(int i = 0; i < estimatorDataHolder.getJoints().length; i++)
      {
         intermediateDesiredJointData[i] = new DesiredJointData();
      }      
   }
   
   public void copyFromController()
   {
      controllerDataHolder.updateFromModel();
      for(int i = 0; i < controllerDataHolder.getJoints().length; i++)
      {
         intermediateDesiredJointData[i].set(controllerDataHolder.get(i));
      }
      
   }

   public void readIntoEstimator()
   {
      for(int i = 0; i < estimatorDataHolder.getJoints().length; i++)
      {
         estimatorDataHolder.get(i).set(intermediateDesiredJointData[i]);
      }
   }
}
