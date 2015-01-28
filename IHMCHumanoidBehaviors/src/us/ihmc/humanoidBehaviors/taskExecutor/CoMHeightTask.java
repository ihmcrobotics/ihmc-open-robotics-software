package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.utilities.taskExecutor.Task;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class CoMHeightTask implements Task
{
   private static final boolean DEBUG = false;
   private final ComHeightPacket comHeightPacket;
   private final ComHeightBehavior comHeightBehavior;

   private final DoubleYoVariable yoTime;
   private double behaviorDoneTime = Double.NaN;
   private final double sleepTime;

   public CoMHeightTask(double heightOffset, DoubleYoVariable yoTime, ComHeightBehavior comHeightBehavior, double trajectoryTime)
   {
      this(heightOffset, yoTime, comHeightBehavior, trajectoryTime, 0.0);
   }

   public CoMHeightTask(double heightOffset, DoubleYoVariable yoTime, ComHeightBehavior comHeightBehavior, double trajectoryTime, double sleepTime)
   {
      this.comHeightBehavior = comHeightBehavior;
      this.yoTime = yoTime;
      this.sleepTime = sleepTime;
      comHeightPacket = new ComHeightPacket(heightOffset, trajectoryTime);
   }

   @Override
   public void doTransitionIntoAction()
   {
      if (DEBUG)
         System.out.println("Started CoM height offset");
      comHeightBehavior.initialize();
      comHeightBehavior.setInput(comHeightPacket);
   }

   @Override
   public void doAction()
   {
      comHeightBehavior.doControl();
      if (Double.isNaN(behaviorDoneTime) && comHeightBehavior.isDone())
      {
         behaviorDoneTime = yoTime.getDoubleValue();
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
      if (DEBUG)
         System.out.println("Finished CoM height offset");
      comHeightBehavior.finalize();
   }

   @Override
   public boolean isDone()
   {
      boolean sleepTimeAchieved = yoTime.getDoubleValue() > behaviorDoneTime + sleepTime;
      return comHeightBehavior.isDone() && sleepTimeAchieved;
   }
}
