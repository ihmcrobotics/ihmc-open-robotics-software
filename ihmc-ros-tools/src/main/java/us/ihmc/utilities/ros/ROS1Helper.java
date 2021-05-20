package us.ihmc.utilities.ros;

import org.ros.internal.message.Message;
import org.ros.message.Time;
import org.ros.node.parameter.ParameterListener;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

/**
 * This class should help the user
 * - Easily publish and subscribe
 * - Reconnect
 * - Remove and add topic subscriptions
 */
public class ROS1Helper implements RosNodeInterface
{
   private final String nodeName;
   private RosMainNode ros1Node;

   private final HashMap<RosTopicPublisher<? extends Message>, String> publishers = new HashMap<>();
   private final HashMap<RosTopicSubscriberInterface<? extends Message>, String> subscribers = new HashMap<>();

   private boolean hasBeenExecuted = false;
   private boolean needsReconnect = true;

   public ROS1Helper(String nodeName)
   {
      this.nodeName = nodeName;
   }

   public void ensureConnected()
   {
      if (needsReconnect)
      {
         hasBeenExecuted = false;
         LogTools.info("Reconnecting ROS 1 node...");
         if (ros1Node != null)
            ros1Node.shutdown();

         ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), nodeName);

         for (Map.Entry<RosTopicPublisher<? extends Message>, String> publisher : publishers.entrySet())
         {
            ros1Node.attachPublisher(publisher.getValue(), publisher.getKey());
         }
         for (Map.Entry<RosTopicSubscriberInterface<? extends Message>, String> subscriber : subscribers.entrySet())
         {
            ros1Node.attachSubscriber(subscriber.getValue(), subscriber.getKey());
         }

         ros1Node.execute();
         hasBeenExecuted = true;
         needsReconnect = false;
      }
   }

   @Override
   public void attachPublisher(String topicName, RosTopicPublisher<? extends Message> publisher)
   {
      publishers.put(publisher, topicName);

      if (hasBeenExecuted)
      {
         needsReconnect = true;
      }
      else
      {
         ros1Node.attachPublisher(topicName, publisher);
      }
   }

   @Override
   public void attachSubscriber(String topicName, RosTopicSubscriberInterface<? extends Message> subscriber)
   {
      subscribers.put(subscriber, topicName);

      if (hasBeenExecuted)
      {
         needsReconnect = true;
      }
      else
      {
         ros1Node.attachSubscriber(topicName, subscriber);
      }
   }

   @Override
   public void removeSubscriber(RosTopicSubscriberInterface<? extends Message> subscriber)
   {
      if (ros1Node != null)
         ros1Node.removeSubscriber(subscriber);

      subscribers.remove(subscriber);
   }

   @Override
   public boolean isStarted()
   {
      return hasBeenExecuted && ros1Node.isStarted();
   }

   @Override
   public Time getCurrentTime()
   {
      if (isStarted())
         return ros1Node.getCurrentTime();
      else
         return null;
   }

   public RosMainNode getROS1Node()
   {
      return ros1Node;
   }

   public void destroy()
   {
      ros1Node.shutdown();
   }

   @Override
   public void attachParameterListener(String topicName, ParameterListener listener)
   {
      throw new RuntimeException("Not implemented yet.");
   }

   @Override
   public void attachServiceServer(String topicName, RosServiceServer<? extends Message, ? extends Message> server)
   {
      throw new RuntimeException("Not implemented yet.");
   }

   @Override
   public void attachServiceClient(String topicName, RosServiceClient<? extends Message, ? extends Message> client)
   {
      throw new RuntimeException("Not implemented yet.");
   }

   @Override
   public <T extends Message> void attachSubscriber(String topicName, Class<T> type, Consumer<T> callback)
   {
      throw new RuntimeException("Not implemented yet.");
   }
}
