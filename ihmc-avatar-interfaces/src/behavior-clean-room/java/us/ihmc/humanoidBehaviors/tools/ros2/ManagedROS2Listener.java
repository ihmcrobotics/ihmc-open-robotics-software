package us.ihmc.humanoidBehaviors.tools.ros2;

import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.SubscriptionMatchedListener;

public class ManagedROS2Listener<T> implements NewMessageListener<T>, SubscriptionMatchedListener<T>
{
   private final NewMessageListener<T> listener;
   private SubscriptionMatchedListener<T> subscriptionMatchedListener;
   private boolean enabled = true;

   public ManagedROS2Listener(NewMessageListener<T> listener)
   {
      this.listener = listener;
   }

   public ManagedROS2Listener(NewMessageListener<T> listener, SubscriptionMatchedListener<T> subscriptionMatchedListener)
   {
      this.listener = listener;
      this.subscriptionMatchedListener = subscriptionMatchedListener;
   }

   @Override
   public void onNewDataMessage(Subscriber<T> subscriber)
   {
      if (enabled)
      {
         listener.onNewDataMessage(subscriber);
      }
      else
      {
         subscriber.takeNextData();
      }
   }

   @Override
   public void onSubscriptionMatched(Subscriber<T> subscriber, MatchingInfo info)
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
