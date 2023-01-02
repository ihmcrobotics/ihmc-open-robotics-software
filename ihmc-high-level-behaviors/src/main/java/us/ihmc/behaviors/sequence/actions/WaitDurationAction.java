package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.tools.Timer;

public class WaitDurationAction extends WaitDurationActionData implements BehaviorAction
{
   private final Timer timer = new Timer();

   @Override
   public void performAction()
   {
      timer.reset();
   }

   @Override
   public boolean isExecuting()
   {
      return timer.isRunning(getWaitDuration());
   }
}
