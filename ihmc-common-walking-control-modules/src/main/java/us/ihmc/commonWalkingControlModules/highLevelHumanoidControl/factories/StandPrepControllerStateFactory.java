package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;

public class StandPrepControllerStateFactory implements HighLevelControllerStateFactory
{
   private StandPrepControllerState standPrepControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (standPrepControllerState == null)
         standPrepControllerState = new StandPrepControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                 controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                 controllerFactoryHelper.getLowLevelControllerOutput());

      return standPrepControllerState;
   }

   @Override
   public HighLevelController getStateEnum()
   {
      return HighLevelController.STAND_PREP_STATE;
   }
}
