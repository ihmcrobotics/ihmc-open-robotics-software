package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ArmTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

import java.util.List;

/**
 * This controller state accepts controller API commands, manages the trajectories, but
 * passes the desired positions directly to the joint desired output list.
 */
public class PositionPassthroughControllerState extends HighLevelControllerState
{
   private final static HighLevelControllerName controllerState = HighLevelControllerName.POSITION_PASSTHROUGH;
   private final CommandInputManager commandInputManager;
   private final YoLowLevelOneDoFJointDesiredDataHolder jointDesiredDataHolder;
   /** not used **/
   private final RootJointDesiredConfigurationData rootJointDesiredConfigurationData = new RootJointDesiredConfigurationData();

   public PositionPassthroughControllerState(CommandInputManager commandInputManager,
                                             HighLevelHumanoidControllerToolbox controllerToolbox,
                                             HighLevelControllerParameters highLevelControllerParameters)
   {
      super(controllerState, highLevelControllerParameters, controllerToolbox.getControlledOneDoFJoints());

      this.commandInputManager = commandInputManager;

      jointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(controllerToolbox.getControlledOneDoFJoints(),
                                                                          controllerToolbox.getYoVariableRegistry());
   }

   @Override
   public void onEntry()
   {

   }

   @Override
   public void doAction(double timeInState)
   {
      List<ArmTrajectoryCommand> armTrajectoryCommands = commandInputManager.pollNewCommands(ArmTrajectoryCommand.class);

      for (ArmTrajectoryCommand armTrajectoryCommand : armTrajectoryCommands)
      {
         for (int i = 0; i < armTrajectoryCommand.getJointspaceTrajectory().getNumberOfJoints(); i++)
         {

//            jointDesiredDataHolder.setDesiredJointPosition();
         }
      }


   }

   @Override
   public void onExit(double timeInState)
   {

   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return jointDesiredDataHolder;
   }

   @Override
   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return rootJointDesiredConfigurationData;
   }
}
