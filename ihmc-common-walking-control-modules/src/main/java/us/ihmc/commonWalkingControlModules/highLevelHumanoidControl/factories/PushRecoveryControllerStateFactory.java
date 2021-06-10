package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class PushRecoveryControllerStateFactory implements HighLevelControllerStateFactory
{
   private PushRecoveryControllerState pushRecoveryControllerState;
   private final PushRecoveryControlManagerFactory managerFactory;
   public PushRecoveryControllerStateFactory(PushRecoveryControlManagerFactory managerFactory)
   {
      this.managerFactory = managerFactory;
   }

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (pushRecoveryControllerState == null)
      {
         pushRecoveryControllerState = new PushRecoveryControllerState(controllerFactoryHelper.getCommandInputManager(), controllerFactoryHelper.getStatusMessageOutputManager(),
                                                             managerFactory, controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                             controllerFactoryHelper.getHighLevelControllerParameters(),
                                                             controllerFactoryHelper.getPushRecoveryControllerParameters());
      }

      return pushRecoveryControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.PUSH_RECOVERY;
   }
}
