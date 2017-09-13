package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.ControllerFailedTransition;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.EnumMap;

public class ControllerFailedTransitionFactory implements ControllerStateTransitionFactory<HighLevelControllerState>
{
   private StateTransition<HighLevelControllerState> stateTransition;

   private final HighLevelControllerState stateToAttachEnum;
   private final HighLevelControllerState nextStateEnum;

   public ControllerFailedTransitionFactory(HighLevelControllerState stateToAttachEnum, HighLevelControllerState nextStateEnum)
   {
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
   }

   @Override
   public StateTransition<HighLevelControllerState> getOrCreateStateTransition(EnumMap<HighLevelControllerState, ? extends FinishableState<HighLevelControllerState>> controllerStateMap,
                                                                                   HighLevelControllerFactoryHelper controllerFactoryHelper, ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                                                                   YoVariableRegistry parentRegistry)
   {
      if (stateTransition != null)
         return stateTransition;

      StateTransitionCondition stateTransitionCondition = new ControllerFailedTransition(stateToAttachEnum, nextStateEnum, controllerFactoryHelper.getFallbackControllerForFailure(),
                                                                                         controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(), parentRegistry);
      stateTransition = new StateTransition<>(nextStateEnum, stateTransitionCondition);

      return stateTransition;
   }

   @Override
   public HighLevelControllerState getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
