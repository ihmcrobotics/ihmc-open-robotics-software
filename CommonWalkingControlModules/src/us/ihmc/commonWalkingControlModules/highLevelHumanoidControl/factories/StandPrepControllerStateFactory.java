package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandPrepControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;

import java.util.EnumMap;
import java.util.HashMap;

public class StandPrepControllerStateFactory implements HighLevelControllerStateFactory
{
   private StandPrepControllerState standPrepControllerState;

   @Override
   public NewHighLevelControllerState getOrCreateControllerState(EnumMap<NewHighLevelControllerStates, HighLevelControllerStateFactory> controllerFactoriesMap,
                                                                 HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                                                 CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                                                 HighLevelControlManagerFactory managerFactory, WalkingControllerParameters walkingControllerParameters,
                                                                 ICPTrajectoryPlannerParameters capturePointPlannerParameters, WholeBodyControllerCore controllerCore)
   {
      if (standPrepControllerState == null)
         standPrepControllerState = new StandPrepControllerState(controllerToolbox, highLevelControllerParameters);

      return standPrepControllerState;
   }

   @Override
   public NewHighLevelControllerStates getStateEnum()
   {
      return NewHighLevelControllerStates.STAND_PREP_STATE;
   }
}
