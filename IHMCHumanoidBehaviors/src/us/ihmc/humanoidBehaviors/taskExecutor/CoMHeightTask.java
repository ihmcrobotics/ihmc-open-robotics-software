package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.ComHeightBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class CoMHeightTask extends BehaviorTask
{
   private static final boolean DEBUG = false;
   private final ComHeightPacket comHeightPacket;
   private final ComHeightBehavior comHeightBehavior;

   public CoMHeightTask(double heightOffset, DoubleYoVariable yoTime, ComHeightBehavior comHeightBehavior, double trajectoryTime)
   {
      this(heightOffset, yoTime, comHeightBehavior, trajectoryTime, 0.0);
   }

   public CoMHeightTask(double heightOffset, DoubleYoVariable yoTime, ComHeightBehavior comHeightBehavior, double trajectoryTime, double sleepTime)
   {
      super(comHeightBehavior, yoTime, sleepTime);
      this.comHeightBehavior = comHeightBehavior;
      comHeightPacket = new ComHeightPacket(heightOffset, trajectoryTime);
   }

   @Override
   protected void setBehaviorInput()
   {
      comHeightBehavior.setInput(comHeightPacket);
   }


}
