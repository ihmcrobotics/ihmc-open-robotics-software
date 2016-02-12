package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class FootstepListTask extends BehaviorTask
{
   private final FootstepListBehavior footstepListBehavior;
   private final FootstepDataListMessage footStepList;
   
   public FootstepListTask(FootstepListBehavior footstepListBehavior,FootstepDataListMessage footStepList ,DoubleYoVariable yoTime,double sleepTime)
   {
      super(footstepListBehavior, yoTime, sleepTime);
      this.footstepListBehavior = footstepListBehavior;
      this.footStepList= footStepList;
   }

   public FootstepListTask(FootstepListBehavior footstepListBehavior, FootstepDataListMessage footStepList, DoubleYoVariable yoTime)
   {
      this(footstepListBehavior,footStepList, yoTime, 0.0);
   }

   @Override
   protected void setBehaviorInput()
   {
      footstepListBehavior.set(footStepList);
   }
}
