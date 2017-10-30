package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

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
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.STAND_PREP_STATE;
   }
}
