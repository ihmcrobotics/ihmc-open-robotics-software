package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import java.util.ArrayList;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.robotics.stateMachine.core.State;

// a behavior action can be used in either a StateMachine or a pipeline.

public class BehaviorAction implements State
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
      }
      this.initializeOnTransitionIntoAction = initializeOnTransitionIntoAction;
   }

   @Override
   public void onEntry()
   {
      if (initializeOnTransitionIntoAction)
      {

         for (int i = 0; i < behaviors.size(); i++)
         {
            //TODO merge abstract behavior and behavior task, add transitioninto behavior here 
            //TODO: Should the setBehaviorInput be called before initialize?
            behaviors.get(i).initialize();
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
      for (int i = 0; i < behaviors.size(); i++)
      {
         behaviors.get(i).doControl();
      }
   }

   @Override
   public void onExit()
   {
      doPostBehaviorCleanup();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return isDone();
   }
  
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