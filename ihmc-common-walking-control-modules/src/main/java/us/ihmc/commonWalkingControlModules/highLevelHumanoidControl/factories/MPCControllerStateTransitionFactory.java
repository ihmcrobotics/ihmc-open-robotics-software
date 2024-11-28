package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelMPCControllerFactoryHelper;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.EnumMap;

public interface MPCControllerStateTransitionFactory<E extends Enum<E>>
{
   /**
    * Gets the new transition. If it has not been created previously, this creates it, and then returns
    * it.
    */
   StateTransition<E> getOrCreateStateTransition(EnumMap<E, ? extends State> stateMap, HighLevelMPCControllerFactoryHelper controllerFactoryHelper,
                                                 YoRegistry parentRegistry);

   /**
    * The state in which this transition is to be checked.
    */
   E getStateToAttachEnum();
}
