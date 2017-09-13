package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandTransitionControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;

import java.util.EnumMap;

public class StandTransitionControllerStateFactory implements HighLevelControllerStateFactory
{
   private StandTransitionControllerState standTransitionControllerState;

   @Override
   public NewHighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (standTransitionControllerState == null)
      {
         EnumMap<HighLevelControllerState, HighLevelControllerStateFactory> controllerFactoriesMap = controllerFactoryHelper.getControllerFactories();
         HighLevelControllerStateFactory standReadyControllerStateFactory = controllerFactoriesMap.get(HighLevelControllerState.STAND_READY);
         HighLevelControllerStateFactory walkingControllerStateFactory = controllerFactoriesMap.get(HighLevelControllerState.WALKING);

         NewHighLevelControllerState standReadyControllerState = standReadyControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         NewHighLevelControllerState walkingControllerState = walkingControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);

         standTransitionControllerState = new  StandTransitionControllerState(standReadyControllerState, walkingControllerState,
                                                                              controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                              controllerFactoryHelper.getHighLevelControllerParameters());
      }

      return standTransitionControllerState;
   }

   @Override
   public HighLevelControllerState getStateEnum()
   {
      return HighLevelControllerState.STAND_TRANSITION_STATE;
   }
}
