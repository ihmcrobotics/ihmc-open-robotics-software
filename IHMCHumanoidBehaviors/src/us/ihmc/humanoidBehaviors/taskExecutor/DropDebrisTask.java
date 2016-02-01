package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.DropDebrisBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;

public class DropDebrisTask extends BehaviorTask
{
   private static final boolean DEBUG = false;
   private final DropDebrisBehavior dropDebrisBehavior;
   private final RobotSide robotSide;
   
   
   public DropDebrisTask(DropDebrisBehavior dropDebrisBehavior, RobotSide robotSide, DoubleYoVariable yoTime)
   {
      super(dropDebrisBehavior, yoTime);
      this.dropDebrisBehavior = dropDebrisBehavior;
      this.robotSide = robotSide;
   }

   @Override
   protected void setBehaviorInput()
   {
      dropDebrisBehavior.setInputs(robotSide);
   }
}
