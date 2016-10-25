package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.GoHomeBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;

public class GoHomeTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final GoHomeMessage goHomeMessage;
   private final GoHomeBehavior goHomeBehavior;

   public GoHomeTask(GoHomeMessage goHomeMessage, GoHomeBehavior goHomeBehavior)
   {
      this(null, goHomeMessage, goHomeBehavior);
   }
   
   public GoHomeTask(E stateEnum,GoHomeMessage goHomeMessage, GoHomeBehavior goHomeBehavior)
   {
      super(stateEnum,goHomeBehavior);
      this.goHomeBehavior = goHomeBehavior;
      this.goHomeMessage = goHomeMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      goHomeBehavior.setInput(goHomeMessage);
   }
}
