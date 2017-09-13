package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.ControllerFailedTransition;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.EnumMap;

public class ControllerFailedTransitionFactory implements ControllerStateTransitionFactory<HighLevelController>
{
   private StateTransition<HighLevelController> stateTransition;

   private final HighLevelController stateToAttachEnum;
   private final HighLevelController nextStateEnum;

   public ControllerFailedTransitionFactory(HighLevelController stateToAttachEnum, HighLevelController nextStateEnum)
   {
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
   }

   @Override
   public StateTransition<HighLevelController> getOrCreateStateTransition(EnumMap<HighLevelController, ? extends FinishableState<HighLevelController>> controllerStateMap,
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
   public HighLevelController getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
