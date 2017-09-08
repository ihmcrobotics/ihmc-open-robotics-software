package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewWalkingControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

import java.util.EnumMap;

public class NewWalkingControllerStateFactory implements HighLevelControllerStateFactory
{
   private NewWalkingControllerState walkingControllerState;

   @Override
   public NewHighLevelControllerState getOrCreateControllerState(EnumMap<NewHighLevelControllerStates, HighLevelControllerStateFactory> controllerFactoriesMap,
                                                                 HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                                                 CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                                                 HighLevelControlManagerFactory managerFactory, WalkingControllerParameters walkingControllerParameters,
                                                                 ICPTrajectoryPlannerParameters capturePointPlannerParameters)
   {
      if (walkingControllerState == null)
      {
         walkingControllerState = new NewWalkingControllerState(commandInputManager, statusOutputManager, managerFactory, controllerToolbox, highLevelControllerParameters,
                                                                capturePointPlannerParameters, walkingControllerParameters);
      }

      return walkingControllerState;
   }

   @Override
   public NewHighLevelControllerStates getStateEnum()
   {
      return NewHighLevelControllerStates.WALKING_STATE;
   }
}
