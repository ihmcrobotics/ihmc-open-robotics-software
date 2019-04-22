package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import java.util.ArrayList;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveBehaviorTools;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.taskExecutor.Task;

// a behavior action can be used in either a StateMachine or a pipeline.

public class BehaviorAction implements Task, State
{
   private final ArrayList<AbstractBehavior> behaviors;
   private final Boolean initializeOnTransitionIntoAction;

   public BehaviorAction(AbstractBehavior... behavior)
   {
      this(true, behavior);
   }

   public BehaviorAction(boolean initializeOnTransitionIntoAction, AbstractBehavior... behavior)
   {
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

   @Override
   public void onEntry()
   {
      doTransitionIntoAction();
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

            CoactiveElement coactiveElement = behaviors.get(i).getCoactiveElement();
            if (coactiveElement != null)
            {
               coactiveElement.initializeMachineSide();
            }
         }
         setBehaviorInput();

         
      }
   }

   protected void setBehaviorInput()
   {

   }

   @Override
   public void doAction(double timeInState)
   {
      doAction();
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
   public void onExit()
   {
      doTransitionOutOfAction();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      doPostBehaviorCleanup();
   }

   @Override
   public boolean isDone()
   {
      return isDone(Double.NaN);
   }

   @Override
   public boolean isDone(double timeInState)
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