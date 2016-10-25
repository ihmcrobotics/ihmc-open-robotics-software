package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.BehaviorCommunicationBridge;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public abstract class StateMachineBehavior<E extends Enum<E>> extends AbstractBehavior
{

   protected BehaviorStateMachine<E> statemachine;

   public StateMachineBehavior(String name, Class<E> enumType, DoubleYoVariable yoTime, BehaviorCommunicationBridge outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);
      statemachine = new BehaviorStateMachine<E>(name, name + "SwitchTime", enumType, yoTime, registry);
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

   @Override 
   public void doControl()
   {
      statemachine.doAction();
      statemachine.checkTransitionConditions();
   }

  

   @Override
   public boolean isDone()
   {
      //if your current state has finished and there is no transition out of that state... the entire state machine is finished
      if (statemachine.getCurrentState().isDone() && statemachine.getCurrentState().getStateTransitions().size() == 0)
      {
         return true;
      }
      return false;

   }

}
