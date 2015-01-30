package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.midLevel.DropDebrisBehavior;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.taskExecutor.Task;

public class DropDebrisTask implements Task
{
   private static final boolean DEBUG = false;
   private final DropDebrisBehavior dropDebrisBehavior;
   private final RobotSide robotSide;
   
   
   public DropDebrisTask(DropDebrisBehavior dropDebrisBehavior, RobotSide robotSide)
   {
      this.dropDebrisBehavior = dropDebrisBehavior;
      this.robotSide = robotSide;
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (DEBUG)
         System.out.println("started DropDebrisTask");
      dropDebrisBehavior.initialize();
      dropDebrisBehavior.setInputs(robotSide);
   }

   @Override
   public void doAction()
   {
      dropDebrisBehavior.doControl();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (DEBUG)
         System.out.println("Finished DropDebrisTask");
      dropDebrisBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      return dropDebrisBehavior.isDone();
   }
}
