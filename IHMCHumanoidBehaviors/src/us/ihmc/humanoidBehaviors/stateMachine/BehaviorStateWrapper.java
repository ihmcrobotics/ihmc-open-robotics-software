package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.yoUtilities.stateMachines.State;

public class BehaviorStateWrapper <E extends Enum<E>> extends State<E>
{
   private final BehaviorInterface behavior;

   public BehaviorStateWrapper(E stateEnum, BehaviorInterface behavior)
   {
      super(stateEnum);

      this.behavior = behavior;
   }

   @Override
   public void doAction()
   {
      behavior.doControl();
   }
   
   public void pause()
   {
      behavior.pause();
   }

   public void resume()
   {
      behavior.resume();
   }

   public void stop()
   {
      behavior.stop();
   }

   public void enableActions()
   {
      behavior.enableActions();
   }

   public void finalize()
   {
      behavior.finalize();
   }

   @Override
   public void doTransitionIntoAction()
   {
      behavior.initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      behavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      return behavior.isDone();
   }

   public BehaviorInterface getBehavior()
   {
      return behavior;
   }
}