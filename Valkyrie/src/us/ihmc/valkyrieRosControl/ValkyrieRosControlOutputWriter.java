package us.ihmc.valkyrieRosControl;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureListener;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerStateChangedListener;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.wholeBodyController.DRCOutputWriter;

public class ValkyrieRosControlOutputWriter implements DRCOutputWriter, ControllerStateChangedListener, ControllerFailureListener
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
   public void writeAfterController(long timestamp)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void setFullRobotModel(SDFFullHumanoidRobotModel controllerModel, RawJointSensorDataHolderMap rawJointSensorDataHolderMap)
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
   public void controllerFailed(FrameVector2d fallingDirection)
   {
      // TODO Auto-generated method stub
      
   }

}
