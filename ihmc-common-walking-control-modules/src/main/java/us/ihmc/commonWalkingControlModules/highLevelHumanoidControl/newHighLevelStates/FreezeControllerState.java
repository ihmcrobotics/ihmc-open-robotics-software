package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;

public class FreezeControllerState extends HoldPositionControllerState
{
   private static final NewHighLevelControllerStates controllerState = NewHighLevelControllerStates.FREEZE_STATE;

   public FreezeControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters)
   {
      super(controllerState, controllerToolbox, highLevelControllerParameters.getPositionControlParameters());

      OneDoFJoint[] oneDoFJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      for (OneDoFJoint joint : oneDoFJoints)
      {
         LowLevelJointControlMode jointControlMode = highLevelControllerParameters.getLowLevelJointControlMode(joint.getName(), controllerState);
         lowLevelOneDoFJointDesiredDataHolder.setJointControlMode(joint, jointControlMode);
      }
   }
}
