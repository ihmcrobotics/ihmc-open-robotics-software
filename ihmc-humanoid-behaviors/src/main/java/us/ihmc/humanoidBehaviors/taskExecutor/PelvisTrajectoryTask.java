package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.PelvisTrajectoryBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;

public class PelvisTrajectoryTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final PelvisTrajectoryMessage pelvisTrajectoryMessage;
   private final PelvisTrajectoryBehavior pelvisTrajectoryBehavior;

   public PelvisTrajectoryTask(PelvisTrajectoryMessage pelvisTrajectoryMessage,  PelvisTrajectoryBehavior pelvisTrajectoryBehavior)
   {
      this(null, pelvisTrajectoryMessage, pelvisTrajectoryBehavior);
   }

   
   public PelvisTrajectoryTask(E stateEnum,PelvisTrajectoryMessage pelvisTrajectoryMessage,  PelvisTrajectoryBehavior pelvisTrajectoryBehavior)
   {  
      super(stateEnum, pelvisTrajectoryBehavior);
      this.pelvisTrajectoryBehavior = pelvisTrajectoryBehavior;
      this.pelvisTrajectoryMessage = pelvisTrajectoryMessage;
   }

   @Override
   protected void setBehaviorInput()
   {
      pelvisTrajectoryBehavior.setInput(pelvisTrajectoryMessage); 
   }
}
