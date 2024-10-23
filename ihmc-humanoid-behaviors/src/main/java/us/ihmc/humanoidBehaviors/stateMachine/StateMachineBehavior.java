package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.commons.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.providers.DoubleProvider;

public abstract class StateMachineBehavior<E extends Enum<E>> extends AbstractBehavior
{
   private final Class<E> keyType;
   private BehaviorStateMachine<E> stateMachine = null;
   private final String namePrefix;
   private final DoubleProvider timeProvider;

   public StateMachineBehavior(String robotName, String stateMachineName, Class<E> keyType, DoubleProvider timeProvider, ROS2Node ros2Node)
   {
      this(robotName, stateMachineName, stateMachineName, keyType, timeProvider, ros2Node);
   }

   public StateMachineBehavior(String robotName, String namePrefix, String stateMachineName, Class<E> keyType,
                               DoubleProvider timeProvider, ROS2Node ros2Node)
   {
      super(robotName, namePrefix, ros2Node);
      this.keyType = keyType;
      this.namePrefix = namePrefix;
      this.timeProvider = timeProvider;
   }

   
   //TODO JJC this should be called from the onBehaviorEntered method of this class instead of classes that extend it, i need to move it and then test to make sure there was not a reason for this decision.
   protected void setupStateMachine()
   {
      StateMachineFactory<E, BehaviorAction> stateMachineFactory = new StateMachineFactory<>(keyType);
      E initialBehaviorKey = configureStateMachineAndReturnInitialKey(stateMachineFactory);
      stateMachineFactory.setNamePrefix(namePrefix).setRegistry(registry).buildYoClock(timeProvider);

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
      if (stateMachine.getCurrentAction().isDone() && stateMachine.isCurrentActionTerminal())
      {
         return true;
      }
      return false;

   }


}
