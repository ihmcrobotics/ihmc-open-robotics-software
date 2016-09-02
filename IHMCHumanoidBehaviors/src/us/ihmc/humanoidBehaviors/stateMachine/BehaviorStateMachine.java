package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.stateMachines.GenericStateMachine;

public class BehaviorStateMachine <E extends Enum<E>> extends GenericStateMachine<E, BehaviorStateWrapper<E>>
{
   public BehaviorStateMachine(String name, String switchTimeName, Class<E> enumType, DoubleYoVariable t, YoVariableRegistry registry)
   {
      super(name, switchTimeName, enumType, t, registry);
   }

   public void initialize()
   {
      BehaviorStateWrapper<E> currentState = getCurrentState();
      currentState.doTransitionIntoAction();
   }

   public void pause()
   {
      BehaviorStateWrapper<E> currentState = getCurrentState();
      currentState.pause();
   }

   public void resume()
   {
      BehaviorStateWrapper<E> currentState = getCurrentState();
      currentState.resume();
   }

   public void stop()
   {
      BehaviorStateWrapper<E> currentState = getCurrentState();
      currentState.stop();
   }

   public void doPostBehaviorCleanup()
   {
      BehaviorStateWrapper<E> currentState = getCurrentState();
      currentState.doPostBehaviorCleanup();
   }

   public void enableActions()
   {
      BehaviorStateWrapper<E> currentState = getCurrentState();
      currentState.enableActions();
   }
}
