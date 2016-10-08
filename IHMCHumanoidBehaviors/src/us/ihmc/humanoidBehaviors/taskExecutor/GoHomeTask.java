package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.GoHomeBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class GoHomeTask extends BehaviorTask
{
   private final GoHomeMessage goHomeMessage;
   private final GoHomeBehavior goHomeBehavior;

   public GoHomeTask(GoHomeMessage goHomeMessage, GoHomeBehavior goHomeBehavior, DoubleYoVariable yoTime)
   {
      super(goHomeBehavior, yoTime);
      this.goHomeBehavior = goHomeBehavior;
      this.goHomeMessage = goHomeMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      goHomeBehavior.setInput(goHomeMessage);
   }
}
