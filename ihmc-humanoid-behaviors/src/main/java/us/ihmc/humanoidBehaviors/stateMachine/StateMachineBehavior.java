package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;

public abstract class StateMachineBehavior<E extends Enum<E>> extends AbstractBehavior
{
   private final Class<E> keyType;
   private BehaviorStateMachine<E> stateMachine = null;

   public StateMachineBehavior(String stateMachineName, Class<E> keyType, DoubleProvider timeProvider, CommunicationBridge outgoingCommunicationBridge)
   {
      this(null, stateMachineName, keyType, timeProvider, outgoingCommunicationBridge);
   }

   public StateMachineBehavior(String namePrefix, String stateMachineName, Class<E> keyType, DoubleProvider timeProvider,
                               CommunicationBridge outgoingCommunicationBridge)
   {
      super(namePrefix, outgoingCommunicationBridge);
      this.keyType = keyType;
   }

   protected void setupStateMachine()
   {
      StateMachineFactory<E, BehaviorAction> stateMachineFactory = new StateMachineFactory<>(keyType);
      E initialBehaviorKey = configureStateMachineAndReturnInitialKey(stateMachineFactory);
      stateMachine = new BehaviorStateMachine<>(stateMachineFactory.build(initialBehaviorKey));
   }

   protected abstract E configureStateMachineAndReturnInitialKey(StateMachineFactory<E, BehaviorAction> factory);

   public BehaviorStateMachine<E> getStateMachine()
   {
      return stateMachine;
   }

   @Override
   public void onBehaviorEntered()
   {
      stateMachine.initialize();
   }

   @Override
   public void onBehaviorPaused()
   {
      stateMachine.pause();
   }

   public void onBehaviorResumed()
   {
      stateMachine.resume();
   }

   public void onBehaviorAborted()
   {
      stateMachine.stop();
   }

   @Override
   public void doControl()
   {
      stateMachine.doControlAndTransitions();
   }

   @Override
   public boolean isDone()
   {
      //if your current state has finished and there is no transition out of that state... the entire state machine is finished

      if (stateMachine.getCurrentBehavior().isDone() && stateMachine.isCurrentBehaviorTerminal())
      {
         return true;
      }
      return false;

   }

}
