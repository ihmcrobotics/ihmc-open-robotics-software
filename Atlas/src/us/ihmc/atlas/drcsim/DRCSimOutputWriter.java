package us.ihmc.atlas.drcsim;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.outputs.DRCOutputWriter;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class DRCSimOutputWriter implements DRCOutputWriter
{
   private final YoVariableRegistry registry = new YoVariableRegistry(DRCSimSensorReaderFactory.class.getSimpleName());

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void writeAfterController(long timestamp)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setFullRobotModel(SDFFullRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolder forceSensorDataHolderForController)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      return registry;
   }

}
