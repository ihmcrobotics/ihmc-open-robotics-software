package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.ControllerFailedTransition;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ControllerFailedTransitionFactory implements ControllerStateTransitionFactory<HighLevelControllerName>
{
   private StateTransition<HighLevelControllerName> stateTransition;

   private final HighLevelControllerName stateToAttachEnum;
   private final HighLevelControllerName nextStateEnum;

   /**
    * This transition will transition the robot from its current state into another state if the
    * standard controller failure listener returns true.
    *
    * @param stateToAttachEnum state to check if the controller has failed.
    * @param nextStateEnum state to transition to if the controller fails.
    */
   public ControllerFailedTransitionFactory(HighLevelControllerName stateToAttachEnum, HighLevelControllerName nextStateEnum)
   {
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
   }

   /** {@inheritDoc} */
   @Override
   public StateTransition<HighLevelControllerName> getOrCreateStateTransition(EnumMap<HighLevelControllerName, ? extends State> controllerStateMap,
                                                                              HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                                              YoVariableRegistry parentRegistry)
   {
      if (stateTransition != null)
         return stateTransition;

      StateTransitionCondition stateTransitionCondition = new ControllerFailedTransition(
            controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControllerFailedBoolean());
      stateTransition = new StateTransition<>(nextStateEnum, stateTransitionCondition);

      return stateTransition;
   }

   /** {@inheritDoc} */
   @Override
   public HighLevelControllerName getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
