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
      
   }

   @Override
   public void setForceSensorDataHolder(ForceSensorDataHolderReadOnly forceSensorDataHolderForEstimator)
   {
      
   }

   @Override
   public void initialize()
   {
      
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return null;
   }

   @Override
   public void writeBefore(long timestamp)
   {
      
   }

   @Override
   public void writeAfter()
   {
      
   }

}
