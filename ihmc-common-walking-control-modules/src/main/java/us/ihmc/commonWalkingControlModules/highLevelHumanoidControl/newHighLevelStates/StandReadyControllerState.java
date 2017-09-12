package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;

public class StandReadyControllerState extends HoldPositionControllerState
{
   private static final NewHighLevelControllerStates controllerState = NewHighLevelControllerStates.STAND_READY;

   public StandReadyControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                    LowLevelOneDoFJointDesiredDataHolderList highLevelControllerOutput)
   {
      super(controllerState, controllerToolbox, highLevelControllerOutput);

      OneDoFJoint[] oneDoFJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      for (OneDoFJoint joint : oneDoFJoints)
      {
         LowLevelJointControlMode jointControlMode = highLevelControllerParameters.getLowLevelJointControlMode(joint.getName(), controllerState);
         lowLevelOneDoFJointDesiredDataHolder.setJointControlMode(joint, jointControlMode);
      }
   }
}
