package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveBehaviorTools;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.robotics.stateMachines.State;

public class BehaviorStateWrapper <E extends Enum<E>> extends State<E>
{   
   private final BehaviorInterface behavior;
   private final Boolean initializeOnTransitionIntoAction;

   public BehaviorStateWrapper(E stateEnum, BehaviorInterface behavior)
   {
      this(stateEnum, behavior, true);
   }
   
   public BehaviorStateWrapper(E stateEnum, BehaviorInterface behavior, boolean initializeOnTransitionIntoAction)
   {
      super(stateEnum);

      this.behavior = behavior;
      this.initializeOnTransitionIntoAction = initializeOnTransitionIntoAction;

      CoactiveElement coactiveElement = behavior.getCoactiveElement();
      if (coactiveElement != null)
      {
         CoactiveBehaviorTools.synchronizeCoactiveElementMachineSideUsingTCPYoWhiteBoard(coactiveElement);
      }
      
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (initializeOnTransitionIntoAction)
      {
         behavior.initialize();

         CoactiveElement coactiveElement = behavior.getCoactiveElement();
         if (coactiveElement != null)
         {
            coactiveElement.initializeMachineSide();
         }
      }
   }
   
   @Override
   public void doAction()
   {
      behavior.doControl();

      CoactiveElement coactiveElement = behavior.getCoactiveElement();
      if (coactiveElement != null)
      {
         coactiveElement.updateMachineSide();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      behavior.doPostBehaviorCleanup();
   }

   @Override
   public boolean isDone()
   {
      return behavior.isDone();
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
   
   public void doPostBehaviorCleanup()
   {
      behavior.doPostBehaviorCleanup();
   }

   public BehaviorInterface getBehavior()
   {
      return behavior;
   }
}