package us.ihmc.valkyrieRosControl;

import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ValkyrieRosControlLowLevelOutputWriter implements JointDesiredOutputWriter
{

   @Override
   public void setJointDesiredOutputList(JointDesiredOutputList lowLevelDataHolder)
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
