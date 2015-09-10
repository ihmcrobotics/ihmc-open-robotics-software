package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.stateMachines.GenericStateMachine;

public class BehaviorStateMachine <E extends Enum<E>> extends GenericStateMachine<E, BehaviorStateWrapper<E>>
{
   public BehaviorStateMachine(String name, String switchTimeName, Class<E> enumType, DoubleYoVariable t, YoVariableRegistry registry)
   {
      super(name, switchTimeName, enumType, t, registry);
   }

   public void initialize()
   {
      BehaviorStateWrapper<E> currentState = getAndCheckCurrentState();
      currentState.doTransitionIntoAction();
   }

   public void pause()
   {
      BehaviorStateWrapper<E> currentState = getAndCheckCurrentState();
      currentState.pause();
   }

   public void resume()
   {
      BehaviorStateWrapper<E> currentState = getAndCheckCurrentState();
      currentState.resume();
   }

   public void stop()
   {
      BehaviorStateWrapper<E> currentState = getAndCheckCurrentState();
      currentState.stop();
   }

   public void doPostBehaviorCleanup()
   {
      BehaviorStateWrapper<E> currentState = getAndCheckCurrentState();
      currentState.doPostBehaviorCleanup();
   }

   public void enableActions()
   {
      BehaviorStateWrapper<E> currentState = getAndCheckCurrentState();
      currentState.enableActions();
   }
}
