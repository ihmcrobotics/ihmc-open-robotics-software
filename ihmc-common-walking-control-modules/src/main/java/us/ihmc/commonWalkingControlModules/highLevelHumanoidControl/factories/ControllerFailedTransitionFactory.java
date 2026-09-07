package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.ControllerFailedTransition;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ControllerFailedTransitionFactory implements ControllerStateTransitionFactory<HighLevelControllerName>
{
   private StateTransition<HighLevelControllerName> stateTransition;

   private final HighLevelControllerName stateToAttachEnum;
   private final HighLevelControllerName nextStateEnum;
   private final boolean transitionToFallingWhenSupported;

   /**
    * This transition will transition the robot from its current state into another state if the
    * standard controller failure listener returns true.
    *
    * @param stateToAttachEnum state to check if the controller has failed.
    * @param nextStateEnum state to transition to if the controller fails.
    */
   public ControllerFailedTransitionFactory(HighLevelControllerName stateToAttachEnum, HighLevelControllerName nextStateEnum)
   {
      this(stateToAttachEnum, nextStateEnum, false);
   }

   /**
    * @param transitionToFallingWhenSupported when {@code true}, a controller failure may enter
    *                                          {@code FALLING_STATE} even while the robot has support.
    */
   public ControllerFailedTransitionFactory(HighLevelControllerName stateToAttachEnum,
                                            HighLevelControllerName nextStateEnum,
                                            boolean transitionToFallingWhenSupported)
   {
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
      this.transitionToFallingWhenSupported = transitionToFallingWhenSupported;
   }

   /** {@inheritDoc} */
   @Override
   public StateTransition<HighLevelControllerName> getOrCreateStateTransition(EnumMap<HighLevelControllerName, ? extends State> controllerStateMap,
                                                                              HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                                              YoRegistry parentRegistry)
   {
      if (stateTransition != null)
         return stateTransition;

      StateTransitionCondition stateTransitionCondition = new ControllerFailedTransition(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox()
                                                                                                                .getControllerFailedBoolean(),
                                                                                         () -> controllerFactoryHelper.getHighLevelControllerParameters()
                                                                                                                      .getIsRobotOffSupport(),
                                                                                         nextStateEnum,
                                                                                         transitionToFallingWhenSupported);
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
