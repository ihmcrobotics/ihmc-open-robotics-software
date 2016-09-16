package us.ihmc.humanoidBehaviors.stateMachine;

import java.util.ArrayList;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveBehaviorTools;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.stateMachines.FinishableState;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.tools.io.printing.PrintTools;

public class BehaviorStateWrapper<E extends Enum<E>> extends FinishableState<E>
{
   private final ArrayList<AbstractBehavior> behaviors;
   private final Boolean initializeOnTransitionIntoAction;
   protected final DoubleYoVariable yoTime;

   public BehaviorStateWrapper(E stateEnum, DoubleYoVariable yoTime, AbstractBehavior... behavior)
   {
      this(stateEnum, true, yoTime, behavior);
   }

   public BehaviorStateWrapper(E stateEnum, boolean initializeOnTransitionIntoAction, DoubleYoVariable yoTime, AbstractBehavior... behavior)
   {
      super(stateEnum);
      behaviors = new ArrayList<AbstractBehavior>();
      this.yoTime = yoTime;
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
         PrintTools.debug(this, "Initializing " + getStateEnum().name());

         for (int i = 0; i < behaviors.size(); i++)
         {

               //TODO merge abstract behavior and behavior task, add transitioninto behavior here 
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
   //TODO this should not be here the behavior input should be called on each behavior after the merge
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

//TODO this should call the transition out for each behavior after the merge
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

      PrintTools.debug(this, "Pausing " + getStateEnum().name());
      for (int i = 0; i < behaviors.size(); i++)
      {
         behaviors.get(i).pause();
      }

   }

   public void resume()
   {

      PrintTools.debug(this, "Resuming " + getStateEnum().name());
      for (int i = 0; i < behaviors.size(); i++)
      {
         behaviors.get(i).resume();
      }

   }

   public void stop()
   {
      PrintTools.debug(this, "Stopping " + getStateEnum().name());
      for (int i = 0; i < behaviors.size(); i++)
      {
         behaviors.get(i).abort();
      }
   }

   public void doPostBehaviorCleanup()
   {
      PrintTools.debug(this, "Cleaning up " + getStateEnum().name());
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