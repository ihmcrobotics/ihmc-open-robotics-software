package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandTransitionControllerState;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;

import java.util.EnumMap;

public class StandTransitionControllerStateFactory implements HighLevelControllerStateFactory
{
   private StandTransitionControllerState standTransitionControllerState;

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (standTransitionControllerState == null)
      {
         EnumMap<HighLevelController, HighLevelControllerStateFactory> controllerFactoriesMap = controllerFactoryHelper.getControllerFactories();
         HighLevelControllerStateFactory standReadyControllerStateFactory = controllerFactoriesMap.get(HighLevelController.STAND_READY);
         HighLevelControllerStateFactory walkingControllerStateFactory = controllerFactoriesMap.get(HighLevelController.WALKING);

         HighLevelControllerState standReadyControllerState = standReadyControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         HighLevelControllerState walkingControllerState = walkingControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);

         standTransitionControllerState = new  StandTransitionControllerState(standReadyControllerState, walkingControllerState,
                                                                              controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                              controllerFactoryHelper.getHighLevelControllerParameters());
      }

      return standTransitionControllerState;
   }

   @Override
   public HighLevelController getStateEnum()
   {
      return HighLevelController.STAND_TRANSITION_STATE;
   }
}
