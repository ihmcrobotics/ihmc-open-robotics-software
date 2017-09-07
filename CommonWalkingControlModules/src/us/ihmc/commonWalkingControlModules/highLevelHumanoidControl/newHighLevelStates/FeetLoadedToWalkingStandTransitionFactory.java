package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.EnumMap;

public class FeetLoadedToWalkingStandTransitionFactory implements ControllerStateTransitionFactory<NewHighLevelControllerStates>
{
   private StateTransition<NewHighLevelControllerStates> stateTransition;

   private final NewHighLevelControllerStates stateToAttach;
   private final NewHighLevelControllerStates nextStateEnum;

   private final YoEnum<NewHighLevelControllerStates> requestedState;
   private final SideDependentList<String> feetContactSensors;
   private final HighLevelControllerParameters highLevelControllerParameters;

   private final double controlDT;


   public FeetLoadedToWalkingStandTransitionFactory(NewHighLevelControllerStates stateToAttach, NewHighLevelControllerStates nextStateEnum,
                                                    YoEnum<NewHighLevelControllerStates> requestedState,
                                                    SideDependentList<String> feetContactSensors, double controlDT,
                                                    HighLevelControllerParameters highLevelControllerParameters)
   {
      this.stateToAttach = stateToAttach;
      this.nextStateEnum = nextStateEnum;
      this.requestedState = requestedState;
      this.feetContactSensors = feetContactSensors;
      this.controlDT = controlDT;
      this.highLevelControllerParameters = highLevelControllerParameters;
   }

   @Override
   public StateTransition<NewHighLevelControllerStates> getOrCreateStateTransition(EnumMap<NewHighLevelControllerStates, ? extends FinishableState<NewHighLevelControllerStates>> controllerStateMap,
                                                                                   ForceSensorDataHolderReadOnly forceSensorDataHolder, double totalMass, double gravityZ, YoVariableRegistry parentRegistry)
   {
      if (stateTransition != null)
         return stateTransition;

      StateTransitionCondition stateTransitionCondition = new FeetLoadedToWalkingStandTransition(controllerStateMap.get(stateToAttach), nextStateEnum,
                                                                                                 requestedState, forceSensorDataHolder, feetContactSensors,
                                                                                                 controlDT, totalMass, gravityZ, highLevelControllerParameters, parentRegistry);
      stateTransition = new StateTransition<>(nextStateEnum, stateTransitionCondition);

      return stateTransition;
   }

   @Override
   public NewHighLevelControllerStates getStateToAttachEnum()
   {
      return stateToAttach;
   }
}
