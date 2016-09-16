package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public abstract class StateMachineBehavior<E extends Enum<E>> extends AbstractBehavior
{

   protected BehaviorStateMachine<E> statemachine;

   public StateMachineBehavior(String name, String switchTimeName, Class<E> enumType, DoubleYoVariable yoTime,
         BehaviorCommunicationBridge outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
      statemachine = new BehaviorStateMachine<E>(name, switchTimeName, enumType, yoTime, registry);
   }

   protected BehaviorStateWrapper<E> createState(E stateEnum,DoubleYoVariable yoTime, AbstractBehavior... behavior)
   {
      BehaviorStateWrapper<E> newState = new BehaviorStateWrapper<E>(stateEnum, yoTime, behavior);
      getStateMachine().addState(newState);
      return newState;
   }

   public BehaviorStateMachine<E> getStateMachine()
   {
      return statemachine;
   }

   public void initialize()
   {
      statemachine.initialize();
   }

   public void pause()
   {
      statemachine.pause();
   }

   public void resume()
   {
      statemachine.resume();
   }

   public void abort()
   {
      statemachine.stop();
   }

   public void doPostBehaviorCleanup()
   {
      statemachine.doPostBehaviorCleanup();
   }

  
   @Override
   public void doControl()
   {
      statemachine.doAction();
      statemachine.checkTransitionConditions();
   }

   @Override
   public boolean hasInputBeenSet()
   {
      return true;
   }

}
