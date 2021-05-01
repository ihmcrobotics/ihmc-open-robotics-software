package us.ihmc.utilities.ros.subscriber;

import java.util.function.Consumer;

public class ROS1Subscriber<T>
{
   private final AbstractRosTopicSubscriber<T> subscriber;

   public ROS1Subscriber(String messageType, Consumer<T> callback)
   {
      subscriber = new AbstractRosTopicSubscriber<T>(messageType)
      {
         @Override
         public void onNewMessage(T message)
         {
            callback.accept(message);
         }
      };
   }

   public AbstractRosTopicSubscriber<T> getSubscriber()
   {
      return subscriber;
   }
}
