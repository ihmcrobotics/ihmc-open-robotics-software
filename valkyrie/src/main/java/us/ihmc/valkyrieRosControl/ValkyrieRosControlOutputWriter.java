package us.ihmc.valkyrieRosControl;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.controllers.ControllerStateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.wholeBodyController.DRCOutputProcessor;

public class ValkyrieRosControlOutputWriter implements DRCOutputProcessor, ControllerStateChangedListener, ControllerFailureListener
{

   public ValkyrieRosControlOutputWriter(ValkyrieRobotModel robotModel)
   {
      // TODO Auto-generated constructor stub
   }

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void processAfterController(long timestamp)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setLowLevelControllerCoreOutput(FullHumanoidRobotModel controllerRobotModel, LowLevelOneDoFJointDesiredDataHolderList lowLevelControllerCoreOutput, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setForceSensorDataHolderForController(ForceSensorDataHolderReadOnly forceSensorDataHolderForController)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public YoVariableRegistry getControllerYoVariableRegistry()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void controllerStateHasChanged(Enum<?> oldState, Enum<?> newState)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void controllerFailed(FrameVector2D fallingDirection)
   {
      // TODO Auto-generated method stub
      
   }

}
