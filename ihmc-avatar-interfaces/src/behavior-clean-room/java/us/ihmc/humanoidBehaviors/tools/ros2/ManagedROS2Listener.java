package us.ihmc.humanoidBehaviors.tools.ros2;

import us.ihmc.pubsub.common.MatchingInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.SubscriptionMatchedListener;

import java.util.function.Supplier;

public class ManagedROS2Listener<T> implements NewMessageListener<T>, SubscriptionMatchedListener<T>
{
   private final NewMessageListener<T> listener;
   private final SubscriptionMatchedListener<T> subscriptionMatchedListener;
   private final Supplier<Boolean> enabled;

   public ManagedROS2Listener(NewMessageListener<T> listener, Supplier<Boolean> enabled)
   {
      this.listener = listener;
      this.subscriptionMatchedListener = null;
      this.enabled = enabled;
   }

   public ManagedROS2Listener(NewMessageListener<T> listener, SubscriptionMatchedListener<T> subscriptionMatchedListener, Supplier<Boolean> enabled)
   {
      this.listener = listener;
      this.subscriptionMatchedListener = subscriptionMatchedListener;
      this.enabled = enabled;
   }

   @Override
   public void onNewDataMessage(Subscriber<T> subscriber)
   {
      if (enabled.get())
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
      if (enabled.get())
      {
         listener.onSubscriptionMatched(subscriber, info);

         if (subscriptionMatchedListener != null)
         {
            subscriptionMatchedListener.onSubscriptionMatched(subscriber, info);
         }
      }
   }
}
