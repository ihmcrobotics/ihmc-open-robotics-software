package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandTransitionControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

import java.util.EnumMap;

public class StandTransitionControllerStateFactory implements HighLevelControllerStateFactory
{
   private StandTransitionControllerState standTransitionControllerState;

   @Override
   public NewHighLevelControllerState getOrCreateControllerState(EnumMap<NewHighLevelControllerStates, HighLevelControllerStateFactory> controllerFactoriesMap,
                                                                 HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                                                 CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                                                 HighLevelControlManagerFactory managerFactory, WalkingControllerParameters walkingControllerParameters,
                                                                 ICPTrajectoryPlannerParameters capturePointPlannerParameters)
   {
      if (standTransitionControllerState == null)
      {
         HighLevelControllerStateFactory standReadyControllerStateFactory = controllerFactoriesMap.get(NewHighLevelControllerStates.STAND_READY);
         HighLevelControllerStateFactory walkingControllerStateFactory = controllerFactoriesMap.get(NewHighLevelControllerStates.WALKING_STATE);

         NewHighLevelControllerState standReadyControllerState = standReadyControllerStateFactory.getOrCreateControllerState(controllerFactoriesMap, controllerToolbox, highLevelControllerParameters,
                                                                                                                             commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters,
                                                                                                                             capturePointPlannerParameters);
         NewHighLevelControllerState walkingControllerState = walkingControllerStateFactory.getOrCreateControllerState(controllerFactoriesMap, controllerToolbox, highLevelControllerParameters,
                                                                                                                       commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters,
                                                                                                                       capturePointPlannerParameters);

         standTransitionControllerState = new  StandTransitionControllerState(standReadyControllerState, walkingControllerState, controllerToolbox, highLevelControllerParameters);
      }

      return standTransitionControllerState;
   }

   @Override
   public NewHighLevelControllerStates getStateEnum()
   {
      return NewHighLevelControllerStates.STAND_TRANSITION_STATE;
   }
}
