package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;

public class FootstepListTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final FootstepListBehavior footstepListBehavior;
   private final FootstepDataListMessage footStepList;
   
   public FootstepListTask(FootstepListBehavior footstepListBehavior,FootstepDataListMessage footStepList)
   {
      this(null,footstepListBehavior, footStepList);
   }
   public FootstepListTask(E stateEnum,FootstepListBehavior footstepListBehavior,FootstepDataListMessage footStepList)
   {
      super(stateEnum, footstepListBehavior);
      this.footstepListBehavior = footstepListBehavior;
      this.footStepList= footStepList;
   }

   
   @Override
   protected void setBehaviorInput()
   {
      footstepListBehavior.set(footStepList);
   }
}
