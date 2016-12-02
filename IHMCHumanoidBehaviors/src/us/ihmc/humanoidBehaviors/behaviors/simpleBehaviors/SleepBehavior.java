package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class SleepBehavior extends AbstractBehavior
{

   protected final DoubleYoVariable yoTime;
   protected double behaviorDoneTime = Double.NaN;
   protected double sleepTime = Double.MIN_VALUE;

   public SleepBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      super(outgoingCommunicationBridge);
      this.yoTime = yoTime;
   }

   public SleepBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime, double sleepTime)
   {
      super(outgoingCommunicationBridge);
      this.yoTime = yoTime;
      this.sleepTime = sleepTime;
   }

   @Override
   public void doControl()
   {
      if (hasInputBeenSet())
      {
         if (Double.isNaN(behaviorDoneTime))
         {
            behaviorDoneTime = yoTime.getDoubleValue();
         }
      }
   }

   public void setSleepTime(double sleepTime)
   {
      this.sleepTime = sleepTime;
   }

   @Override
   public void onBehaviorExited()
   {
   }

   @Override
   public boolean isDone()
   {
      if (!hasInputBeenSet())
      {
         return false;
      }
      boolean sleepTimeAchieved = yoTime.getDoubleValue() > behaviorDoneTime + sleepTime;
      return sleepTimeAchieved;
   }

   public boolean hasInputBeenSet()
   {
      return sleepTime != Double.MIN_VALUE;
   }

   @Override
   public void onBehaviorEntered()
   {
      behaviorDoneTime = Double.NaN;
      sleepTime = Double.MIN_VALUE;
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }
}
