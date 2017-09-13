package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewDoNothingControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;

public class NewDoNothingControllerStateFactory implements HighLevelControllerStateFactory
{
   private NewDoNothingControllerState doNothingControllerState;

   @Override
   public NewHighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (doNothingControllerState == null)
         doNothingControllerState = new NewDoNothingControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                    controllerFactoryHelper.getHighLevelControllerParameters());

      return doNothingControllerState;
   }

   @Override
   public HighLevelControllerState getStateEnum()
   {
      return HighLevelControllerState.DO_NOTHING_BEHAVIOR;
   }
}
