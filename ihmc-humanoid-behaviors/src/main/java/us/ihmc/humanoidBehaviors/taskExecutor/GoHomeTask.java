package us.ihmc.humanoidBehaviors.taskExecutor;

import controller_msgs.msg.dds.GoHomeMessage;
import us.ihmc.humanoidBehaviors.behaviors.primitives.GoHomeBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;

public class GoHomeTask extends BehaviorAction
{
   private final GoHomeMessage goHomeMessage;
   private final GoHomeBehavior goHomeBehavior;

   public GoHomeTask(GoHomeMessage goHomeMessage, GoHomeBehavior goHomeBehavior)
   {
      super(goHomeBehavior);
      this.goHomeBehavior = goHomeBehavior;
      this.goHomeMessage = goHomeMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      goHomeBehavior.setInput(goHomeMessage);
   }
}
