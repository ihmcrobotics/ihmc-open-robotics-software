package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.yoVariables.registry.YoRegistry;

public interface ControllerStateTransitionFactory<E extends Enum<E>>
{
   /**
    * Gets the new transition. If it has not been created previously, this creates it, and then returns
    * it.
    */
   StateTransition<E> getOrCreateStateTransition(EnumMap<E, ? extends State> stateMap, HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                 YoRegistry parentRegistry);

   /**
    * The state in which this transition is to be checked.
    */
   E getStateToAttachEnum();
}
