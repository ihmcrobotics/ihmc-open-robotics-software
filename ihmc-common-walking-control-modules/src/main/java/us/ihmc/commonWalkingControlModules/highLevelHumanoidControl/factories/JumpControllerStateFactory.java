package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JumpControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class JumpControllerStateFactory implements HighLevelControllerStateFactory
{
   private static final HighLevelControllerName stateEnum = HighLevelControllerName.JUMPING;
   private JumpControllerState jumpControllerState = null;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (jumpControllerState == null)
      {
         jumpControllerState = new JumpControllerState(controllerFactoryHelper.getCommandInputManager(),
                                                       controllerFactoryHelper.getStatusMessageOutputManager(),
                                                       controllerFactoryHelper.getHighLevelControllerParameters(),
                                                       controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                       controllerFactoryHelper.getJumpControlManagerFactory(),
                                                       controllerFactoryHelper.getJumpControllerParameters());
      }
      return jumpControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return stateEnum;
   }
}
