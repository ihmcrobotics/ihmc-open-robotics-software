package us.ihmc.commonWalkingControlModules.falling;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.log.LogTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class FallingControllerState extends HighLevelControllerState
{
   private static final HighLevelControllerName controllerState = HighLevelControllerName.FALLING_STATE;
   private final LowLevelOneDoFJointDesiredDataHolder lowLevelOneDoFJointDesiredDataHolder;

   public FallingControllerState(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                 HighLevelControlManagerFactory managerFactory, HighLevelHumanoidControllerToolbox controllerToolbox,
                                 HighLevelControllerParameters highLevelControllerParameters, WalkingControllerParameters walkingControllerParameters)
   {
      super(controllerState, highLevelControllerParameters, controllerToolbox.getControlledOneDoFJoints());
      lowLevelOneDoFJointDesiredDataHolder = new LowLevelOneDoFJointDesiredDataHolder(controlledJoints.length);
      lowLevelOneDoFJointDesiredDataHolder.registerJointsWithEmptyData(controlledJoints);
   }

   @Override
   public void doAction(double timeInState)
   {
      LogTools.info("Falling! Do something!");

      for (int i = 0; i < controlledJoints.length; i++)
      {
         controlledJoints[i].setTau(0.0);
         lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoints[i]).clear();
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointTorque(controlledJoints[i], 0.0);
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointPosition(controlledJoints[i], controlledJoints[i].getQ());
         lowLevelOneDoFJointDesiredDataHolder.setDesiredJointVelocity(controlledJoints[i], controlledJoints[i].getQd());
      }

      lowLevelOneDoFJointDesiredDataHolder.completeWith(getStateSpecificJointSettings());
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void onExit()
   {
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return lowLevelOneDoFJointDesiredDataHolder;
   }
}
