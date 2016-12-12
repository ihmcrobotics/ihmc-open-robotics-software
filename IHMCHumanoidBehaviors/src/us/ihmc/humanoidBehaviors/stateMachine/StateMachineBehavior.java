package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public abstract class StateMachineBehavior<E extends Enum<E>> extends AbstractBehavior
{

   protected BehaviorStateMachine<E> statemachine;

   public StateMachineBehavior(String stateMachineName, Class<E> enumType, DoubleYoVariable yoTime, CommunicationBridge outgoingCommunicationBridge)
   {
      this(null, stateMachineName, enumType, yoTime, outgoingCommunicationBridge);
   }

   public StateMachineBehavior(String namePrefix, String stateMachineName, Class<E> enumType, DoubleYoVariable yoTime, CommunicationBridge outgoingCommunicationBridge)
   {
      super(namePrefix, outgoingCommunicationBridge);
      statemachine = new BehaviorStateMachine<E>(stateMachineName, stateMachineName + "SwitchTime", enumType, yoTime, registry);
   }

   public BehaviorStateMachine<E> getStateMachine()
   {
      return statemachine;
   }

   @Override
   public void onBehaviorEntered()
   {
      statemachine.initialize();
   }

   @Override
   public void onBehaviorPaused()
   {
      statemachine.pause();
   }

   public void onBehaviorResumed()
   {
      statemachine.resume();
   }

   public void onBehaviorAborted()
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
