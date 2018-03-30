package us.ihmc.humanoidBehaviors.taskExecutor;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;

public class FootstepListTask extends BehaviorAction
{
   private final FootstepListBehavior footstepListBehavior;
   private final FootstepDataListMessage footStepList;

   public FootstepListTask(FootstepListBehavior footstepListBehavior, FootstepDataListMessage footStepList)
   {
      super(footstepListBehavior);
      this.footstepListBehavior = footstepListBehavior;
      this.footStepList = footStepList;
   }

   @Override
   protected void setBehaviorInput()
   {
      footstepListBehavior.set(footStepList);
   }
}
