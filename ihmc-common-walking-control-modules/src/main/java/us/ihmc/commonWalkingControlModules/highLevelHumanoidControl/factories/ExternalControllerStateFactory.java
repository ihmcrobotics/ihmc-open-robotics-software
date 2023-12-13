package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.ExternalControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class ExternalControllerStateFactory implements HighLevelControllerStateFactory
{
   private ExternalControllerState externalControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (externalControllerState == null)
      {
         externalControllerState = new ExternalControllerState(controllerFactoryHelper.getCommandInputManager(),
                                                               controllerFactoryHelper.getStatusMessageOutputManager(),
                                                               controllerFactoryHelper.getHighLevelHumanoidControllerToolbox());
      }

      return externalControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.EXTERNAL;
   }
}
