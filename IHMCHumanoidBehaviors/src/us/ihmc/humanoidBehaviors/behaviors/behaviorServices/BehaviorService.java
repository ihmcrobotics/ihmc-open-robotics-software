package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;

public abstract class BehaviorService
{
   private final BehaviorInterface behaviorInterface;
   
   public BehaviorService(BehaviorInterface behaviorInterface)
   {
      this.behaviorInterface = behaviorInterface;
   }
   
   public abstract void initialize();
   
   public abstract void pause();
   
   public abstract void stop();
   
   public abstract void resume();
   
   protected BehaviorInterface getBehaviorInterface()
   {
      return behaviorInterface;
   }
}
