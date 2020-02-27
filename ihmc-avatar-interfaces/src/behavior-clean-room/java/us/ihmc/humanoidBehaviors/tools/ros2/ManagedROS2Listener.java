package us.ihmc.humanoidBehaviors.tools.ros2;

import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.SubscriptionMatchedListener;

public class ManagedROS2Listener implements NewMessageListener, SubscriptionMatchedListener
{
   private final NewMessageListener listener;
   private SubscriptionMatchedListener subscriptionMatchedListener;
   private boolean enabled = true;

   public ManagedROS2Listener(NewMessageListener listener)
   {
      this.listener = listener;
   }

   public ManagedROS2Listener(NewMessageListener listener, SubscriptionMatchedListener subscriptionMatchedListener)
   {
      this.listener = listener;
      this.subscriptionMatchedListener = subscriptionMatchedListener;
   }

   @Override
   public void onNewDataMessage(Subscriber subscriber)
   {
      if (enabled)
      {
         listener.onNewDataMessage(subscriber);
      }
   }

   @Override
   public void onSubscriptionMatched(Subscriber subscriber, MatchingInfo info)
   {
      if (enabled)
      {
         listener.onSubscriptionMatched(subscriber, info);

         if (subscriptionMatchedListener != null)
         {
            subscriptionMatchedListener.onSubscriptionMatched(subscriber, info);
         }
      }
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }
}
