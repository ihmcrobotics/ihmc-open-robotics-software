package us.ihmc.humanoidBehaviors.stateMachine;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveBehaviorTools;
import us.ihmc.humanoidBehaviors.coactiveDesignFramework.CoactiveElement;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.tools.io.printing.PrintTools;

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
         PrintTools.debug(this, "Initializing " + getStateEnum().name());
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
      doPostBehaviorCleanup();
   }

   @Override
   public boolean isDone()
   {
      return behavior.isDone();
   }
   
   public void pause()
   {
      PrintTools.debug(this, "Pausing " + getStateEnum().name());
      behavior.pause();
   }

   public void resume()
   {
      PrintTools.debug(this, "Resuming " + getStateEnum().name());
      behavior.resume();
   }

   public void stop()
   {
      PrintTools.debug(this, "Stopping " + getStateEnum().name());
      behavior.stop();
   }

   public void enableActions()
   {
      PrintTools.debug(this, "Enabling " + getStateEnum().name());
      behavior.enableActions();
   }
   
   public void doPostBehaviorCleanup()
   {
      PrintTools.debug(this, "Cleaning up " + getStateEnum().name());
      behavior.doPostBehaviorCleanup();
   }

   public BehaviorInterface getBehavior()
   {
      return behavior;
   }
}