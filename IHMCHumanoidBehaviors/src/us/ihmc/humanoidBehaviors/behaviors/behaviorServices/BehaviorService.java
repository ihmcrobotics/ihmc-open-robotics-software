package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;

public abstract class BehaviorService
{
   private final AbstractBehavior behaviorInterface;
   
   public BehaviorService(AbstractBehavior behaviorInterface)
   {
      this.behaviorInterface = behaviorInterface;
   }
   
   public abstract void initialize();
   
   public abstract void pause();
   
   public abstract void stop();
   
   public abstract void resume();
   
   protected AbstractBehavior getBehaviorInterface()
   {
      return behaviorInterface;
   }
}
