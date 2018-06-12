package us.ihmc.humanoidBehaviors.behaviors.primitives;

import controller_msgs.msg.dds.FootLoadBearingMessage;
import controller_msgs.msg.dds.FootLoadBearingMessagePubSubType;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;

public class FootLoadBearingBehavior extends AbstractBehavior
{
   private final YoBoolean packetHasBeenSent = new YoBoolean("packetHasBeenSent" + behaviorName, registry);
   private FootLoadBearingMessage outgoingFootLoadBearingMessage;
   private final IHMCROS2Publisher<FootLoadBearingMessage> publisher;

   public FootLoadBearingBehavior(Ros2Node ros2Node)
   {
      this(null, ros2Node);
   }

   public FootLoadBearingBehavior(String prefix, Ros2Node ros2Node)
   {
      super(prefix, ros2Node);
      publisher = createPublisher(new FootLoadBearingMessagePubSubType(), "/ihmc/foot_load_bearing");
   }

   public void setInput(FootLoadBearingMessage endEffectorLoadBearingMessage)
   {
      this.outgoingFootLoadBearingMessage = endEffectorLoadBearingMessage;
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && (outgoingFootLoadBearingMessage != null))
      {
         sendFootStateToController();
      }
   }

   private void sendFootStateToController()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         publisher.publish(outgoingFootLoadBearingMessage);
         packetHasBeenSent.set(true);
      }
   }

   @Override
   public void onBehaviorExited()
   {
      packetHasBeenSent.set(false);
      outgoingFootLoadBearingMessage = null;

      isPaused.set(false);
      isAborted.set(false);
   }

   @Override
   public boolean isDone()
   {
      return packetHasBeenSent.getBooleanValue() && !isPaused.getBooleanValue();
   }

   public boolean hasInputBeenSet()
   {
      if (outgoingFootLoadBearingMessage != null)
         return true;
      else
         return false;
   }

   @Override
   public void onBehaviorEntered()
   {
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
