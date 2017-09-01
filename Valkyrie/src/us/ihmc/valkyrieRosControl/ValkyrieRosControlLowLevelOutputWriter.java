package us.ihmc.valkyrieRosControl;

import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;
import us.ihmc.sensorProcessing.outputData.LowLevelOutputWriter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ValkyrieRosControlLowLevelOutputWriter implements LowLevelOutputWriter
{

   @Override
   public void setLowLevelOneDoFJointDesiredDataHolderList(LowLevelOneDoFJointDesiredDataHolderList lowLevelDataHolder)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setForceSensorDataHolder(ForceSensorDataHolderReadOnly forceSensorDataHolderForEstimator)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void writeBefore(long timestamp)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void writeAfter()
   {
      // TODO Auto-generated method stub
      
   }

}
