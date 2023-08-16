package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.external.ExternalControlCommandConsumer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.external.WholeBodyConfigurationManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;

public class ExternalControllerState extends HighLevelControllerState
{

   private final CommandInputManager commandInputManager;
   private final ExternalControlCommandConsumer externalControlCommandConsumer;
   private final WholeBodyConfigurationManager wholeBodyConfigurationManager;

   public ExternalControllerState(CommandInputManager commandInputManager,
                                  OneDoFJointBasics[] controlledJoints,
                                  HighLevelHumanoidControllerToolbox controllerToolbox)
   {
      super("externalController", HighLevelControllerName.EXTERNAL, controlledJoints);

      this.commandInputManager = commandInputManager;

      wholeBodyConfigurationManager = new WholeBodyConfigurationManager(controllerToolbox.getYoTime(),
                                                                        controllerToolbox.getFullRobotModel(),
                                                                        controlledJoints,
                                                                        registry);
      externalControlCommandConsumer = new ExternalControlCommandConsumer(commandInputManager, wholeBodyConfigurationManager, controllerToolbox.getYoTime());
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return wholeBodyConfigurationManager.getControlOutput();
   }

   @Override
   public void onEntry()
   {
      commandInputManager.clearAllCommands();

      wholeBodyConfigurationManager.initialize();
   }

   @Override
   public void doAction(double timeInState)
   {
      externalControlCommandConsumer.consumeExternalControlCommands();

      wholeBodyConfigurationManager.doControl();
   }

   @Override
   public void onExit(double timeInState)
   {
   }
}