package us.ihmc.externalControl;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class ExternalControllerStateFactory implements HighLevelControllerStateFactory
{
   private ExternalControllerState externalControlState;

   public ExternalControllerStateFactory()
   {
   }

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (externalControlState == null)
      {
         OneDoFJointBasics[] controlledJoints = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints();
         CommandInputManager commandInputManager = controllerFactoryHelper.getCommandInputManager();

         externalControlState = new ExternalControllerState(controllerFactoryHelper.getHighLevelControllerParameters(),
                                                            controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                            controlledJoints,
                                                            controllerFactoryHelper.getLowLevelControllerOutput(),
                                                            commandInputManager);
      }

      return externalControlState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.EXTERNAL;
   }
}
