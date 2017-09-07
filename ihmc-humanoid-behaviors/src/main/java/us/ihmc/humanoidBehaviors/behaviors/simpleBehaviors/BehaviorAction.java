package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import java.util.ArrayList;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveBehaviorTools;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.tools.taskExecutor.Task;

// a behavior action can be used in either a StateMachine or a pipeline.

public class BehaviorAction<E extends Enum<E>> extends FinishableState<E> implements Task
{
   private final ArrayList<AbstractBehavior> behaviors;
   private final Boolean initializeOnTransitionIntoAction;

   public BehaviorAction(AbstractBehavior... behavior)
   {
      this(null, true, behavior);
   }

   public BehaviorAction(E stateEnum, AbstractBehavior... behavior)
   {
      this(stateEnum, true, behavior);
   }

   public BehaviorAction(boolean initializeOnTransitionIntoAction, AbstractBehavior... behavior)
   {
      this(null, initializeOnTransitionIntoAction, behavior);
   }

   public BehaviorAction(E stateEnum, boolean initializeOnTransitionIntoAction, AbstractBehavior... behavior)
   {
      super(stateEnum);
      behaviors = new ArrayList<AbstractBehavior>();
      for (AbstractBehavior currentBehavior : behavior)
      {
         behaviors.add(currentBehavior);
         CoactiveElement coactiveElement = currentBehavior.getCoactiveElement();
         if (coactiveElement != null)
         {
            CoactiveBehaviorTools.synchronizeCoactiveElementMachineSideUsingTCPYoWhiteBoard(coactiveElement);
         }
      }
      this.initializeOnTransitionIntoAction = initializeOnTransitionIntoAction;

   }

   public void createDefaultTransition(E nextState)
   {
      StateTransitionCondition transition = new StateTransitionCondition()
      {
         @Override
         public boolean checkCondition()
         {
            return isDone();
         }
      };
      StateTransition<E> defaultToNextState = new StateTransition<E>(nextState, transition);
      addStateTransition(defaultToNextState);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (initializeOnTransitionIntoAction)
      {

         for (int i = 0; i < behaviors.size(); i++)
         {

            //TODO merge abstract behavior and behavior task, add transitioninto behavior here 
            //TODO: Should the setBehaviorInput be called before initialize?
            behaviors.get(i).initialize();
            setBehaviorInput();

            CoactiveElement coactiveElement = behaviors.get(i).getCoactiveElement();
            if (coactiveElement != null)
            {
               coactiveElement.initializeMachineSide();
            }
         }
      }
   }

   protected void setBehaviorInput()
   {

   }

   @Override
   public void doAction()
   {
      for (int i = 0; i < behaviors.size(); i++)
      {

         
            
            behaviors.get(i).doControl();

            CoactiveElement coactiveElement = behaviors.get(i).getCoactiveElement();
            if (coactiveElement != null)
            {
               coactiveElement.updateMachineSide();
            }
         
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      doPostBehaviorCleanup();
   }

   @Override
   public boolean isDone()
   {
      boolean isDone = true;
      for (int i = 0; i < behaviors.size(); i++)
      {
         if (!behaviors.get(i).isDone())
         {
            isDone = false;
            break;
         }
      }
      return isDone;
   }

   public void pause()
   {

      for (int i = 0; i < behaviors.size(); i++)
      {
         behaviors.get(i).pause();
      }

   }

   public void resume()
   {

      for (int i = 0; i < behaviors.size(); i++)
      {
         behaviors.get(i).resume();
      }

   }

   public void abort()
   {
      for (int i = 0; i < behaviors.size(); i++)
      {
         behaviors.get(i).abort();
      }
   }

   public void doPostBehaviorCleanup()
   {
      for (int i = 0; i < behaviors.size(); i++)
      {
         behaviors.get(i).doPostBehaviorCleanup();
      }

   }

   public ArrayList<AbstractBehavior> getBehaviors()
   {
      return behaviors;
   }

}