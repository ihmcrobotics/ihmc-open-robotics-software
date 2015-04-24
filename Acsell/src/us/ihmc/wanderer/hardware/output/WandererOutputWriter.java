package us.ihmc.wanderer.hardware.output;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerStateChangedListener;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;
import us.ihmc.wanderer.parameters.WandererRobotModel;
import us.ihmc.wholeBodyController.DRCOutputWriter;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class WandererOutputWriter implements DRCOutputWriter, ControllerStateChangedListener
{
   
   public WandererOutputWriter(WandererRobotModel robotModel)
   {
      throw new RuntimeException("TODO: Implement me");
   }

   @Override
   public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState)
   {
      // TODO Auto-generated method stub
      
   }

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
      // TODO Auto-generated method stub
      return null;
   }

}
