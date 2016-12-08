package us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.time.YoTimer;

public class SleepBehavior extends AbstractBehavior
{
   private final DoubleYoVariable sleepTime;
   private final YoTimer timer;

   public SleepBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime)
   {
      this(outgoingCommunicationBridge, yoTime, 1.0);
   }

   public SleepBehavior(CommunicationBridgeInterface outgoingCommunicationBridge, DoubleYoVariable yoTime, double sleepTime)
   {
      super(outgoingCommunicationBridge);

      this.sleepTime = new DoubleYoVariable("sleepTime", registry);
      this.sleepTime.set(sleepTime);

      timer = new YoTimer(yoTime);
   }

   @Override
   public void doControl()
   {
   }

   public void setSleepTime(double sleepTime)
   {
      this.sleepTime.set(sleepTime);
   }

   @Override
   public void onBehaviorExited()
   {
   }

   @Override
   public boolean isDone()
   {
      return (timer.totalElapsed() > sleepTime.getDoubleValue());
   }

   @Override
   public void onBehaviorEntered()
   {
      timer.reset();
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
