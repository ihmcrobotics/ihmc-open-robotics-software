package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandPrepControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;

public class StandPrepControllerStateFactory implements HighLevelControllerStateFactory
{
   private StandPrepControllerState standPrepControllerState;

   @Override
   public NewHighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (standPrepControllerState == null)
         standPrepControllerState = new StandPrepControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                 controllerFactoryHelper.getHighLevelControllerParameters());

      return standPrepControllerState;
   }

   @Override
   public HighLevelControllerState getStateEnum()
   {
      return HighLevelControllerState.STAND_PREP_STATE;
   }
}
