package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandTransitionControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

import java.util.EnumMap;

public class StandTransitionControllerStateFactory implements HighLevelControllerStateFactory
{
   private StandTransitionControllerState standTransitionControllerState;

   @Override
   public NewHighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (standTransitionControllerState == null)
      {
         EnumMap<NewHighLevelControllerStates, HighLevelControllerStateFactory> controllerFactoriesMap = controllerFactoryHelper.getControllerFactories();
         HighLevelControllerStateFactory standReadyControllerStateFactory = controllerFactoriesMap.get(NewHighLevelControllerStates.STAND_READY);
         HighLevelControllerStateFactory walkingControllerStateFactory = controllerFactoriesMap.get(NewHighLevelControllerStates.WALKING_STATE);

         NewHighLevelControllerState standReadyControllerState = standReadyControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         NewHighLevelControllerState walkingControllerState = walkingControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);

         standTransitionControllerState = new  StandTransitionControllerState(standReadyControllerState, walkingControllerState,
                                                                              controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                              controllerFactoryHelper.getHighLevelControllerParameters());
      }

      return standTransitionControllerState;
   }

   @Override
   public NewHighLevelControllerStates getStateEnum()
   {
      return NewHighLevelControllerStates.STAND_TRANSITION_STATE;
   }
}
