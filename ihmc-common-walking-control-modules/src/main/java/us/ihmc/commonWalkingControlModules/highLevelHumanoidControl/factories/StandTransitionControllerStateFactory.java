package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.SmoothTransitionControllerState;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;

public class StandTransitionControllerStateFactory implements HighLevelControllerStateFactory
{
   private SmoothTransitionControllerState standTransitionControllerState;
   private final HighLevelControllerName startState, endState;

   public StandTransitionControllerStateFactory()
   {
      this(HighLevelControllerName.STAND_READY, HighLevelControllerName.WALKING);
   }

   public StandTransitionControllerStateFactory(HighLevelControllerName startState, HighLevelControllerName endState)
   {
      this.startState = startState;
      this.endState = endState;
   }

   @Override
   public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
   {
      if (standTransitionControllerState == null)
      {
         EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerFactoriesMap = controllerFactoryHelper.getControllerFactories();
         HighLevelControllerStateFactory standReadyControllerStateFactory = controllerFactoriesMap.get(startState);
         HighLevelControllerStateFactory walkingControllerStateFactory = controllerFactoriesMap.get(endState);

         HighLevelControllerState standReadyControllerState = standReadyControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         HighLevelControllerState walkingControllerState = walkingControllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);
         OneDoFJointBasics[] controlledJoints = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints();
         CommandInputManager commandInputManager = controllerFactoryHelper.getCommandInputManager();

         standTransitionControllerState = new SmoothTransitionControllerState("toWalking",
                                                                              HighLevelControllerName.STAND_TRANSITION_STATE,
                                                                              standReadyControllerState,
                                                                              walkingControllerState,
                                                                              controlledJoints,
                                                                              controllerFactoryHelper.getHighLevelControllerParameters(),
                                                                              commandInputManager);
      }

      return standTransitionControllerState;
   }

   @Override
   public HighLevelControllerName getStateEnum()
   {
      return HighLevelControllerName.STAND_TRANSITION_STATE;
   }
}
