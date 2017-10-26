package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.ControllerFailedTransition;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ControllerFailedTransitionFactory implements ControllerStateTransitionFactory<HighLevelController>
{
   private StateTransition<HighLevelController> stateTransition;

   private final HighLevelController stateToAttachEnum;
   private final HighLevelController nextStateEnum;

   /**
    * This transition will transition the robot from its current state into another state if the standard controller failure listener returns true.
    *
    * @param stateToAttachEnum state to check if the controller has failed.
    * @param nextStateEnum state to transition to if the controller fails.
    */
   public ControllerFailedTransitionFactory(HighLevelController stateToAttachEnum, HighLevelController nextStateEnum)
   {
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
   }

   /** {@inheritDoc} */
   @Override
   public StateTransition<HighLevelController> getOrCreateStateTransition(EnumMap<HighLevelController, ? extends FinishableState<HighLevelController>> controllerStateMap,
                                                                          HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                                          YoVariableRegistry parentRegistry)
   {
      if (stateTransition != null)
         return stateTransition;

      StateTransitionCondition stateTransitionCondition = new ControllerFailedTransition(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox(),
                                                                                         parentRegistry);
      stateTransition = new StateTransition<>(nextStateEnum, stateTransitionCondition);

      return stateTransition;
   }

   /** {@inheritDoc} */
   @Override
   public HighLevelController getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
