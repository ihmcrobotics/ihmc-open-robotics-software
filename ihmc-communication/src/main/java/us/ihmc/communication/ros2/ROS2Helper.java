package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Bool;
import std_msgs.msg.dds.Empty;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.SwapReference;

import java.io.IOException;
import java.util.function.Consumer;

/**
 * Supports:
 * - Publishing on the fly without having to first create publishers
 * - Disabling and enabling all the publishers and subscribers created here
 */
public class ROS2Helper implements ROS2PublishSubscribeAPI
{
   protected final ROS2NodeInterface ros2NodeInterface;
   protected final ROS2PublisherMap ros2PublisherMap;

   public ROS2Helper(PubSubImplementation pubSubImplementation, String nodeName)
   {
      this(ROS2Tools.createROS2Node(pubSubImplementation, nodeName));
   }

   public ROS2Helper(ROS2NodeInterface ros2Node)
   {
      this.ros2NodeInterface = ros2Node;
      ros2PublisherMap = new ROS2PublisherMap(ros2Node);
   }

   @Override
   public <T> void subscribeViaCallback(ROS2Topic<T> topic, Consumer<T> callback)
   {
      ROS2Tools.createCallbackSubscription2(ros2NodeInterface, topic, callback);
   }

   @Override
   public <T> void subscribeViaVolatileCallback(ROS2Topic<T> topic, Consumer<T> callback)
   {
      ROS2Tools.createVolatileCallbackSubscription(ros2NodeInterface, topic, callback);
   }

   @Override
   public <T> SwapReference<T> subscribeViaSwapReference(ROS2Topic<T> topic, Notification callback)
   {
      return ROS2Tools.createSwapReferenceSubscription(ros2NodeInterface, topic, callback);
   }

   @Override
   public <T> ConcurrentRingBuffer<T> subscribeViaQueue(ROS2Topic<T> topic)
   {
      try
      {
         TopicDataType<T> topicDataType = IHMCROS2Callback.newMessageTopicDataTypeInstance(topic.getType());
         int queueSize = 16;
         ConcurrentRingBuffer<T> concurrentQueue = new ConcurrentRingBuffer<>(topicDataType::createData, queueSize);
         ros2NodeInterface.createSubscription(topicDataType, subscriber ->
         {
            T nextData = concurrentQueue.next();
            if (nextData != null)
            {
               if (subscriber.takeNextData(nextData, null))
               {
                  concurrentQueue.commit();
               }
            }
            else
            {
               LogTools.warn("Concurrent ring buffer is full! Queue size: {}", queueSize);
            }
         }, topic.getName(), ROS2QosProfile.DEFAULT());
         return concurrentQueue;
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   @Override
   public void subscribeViaCallback(ROS2Topic<Empty> topic, Runnable callback)
   {
      ROS2Tools.createCallbackSubscription2(ros2NodeInterface, topic, message -> callback.run());
   }

   @Override
   public <T> IHMCROS2Input<T> subscribe(ROS2Topic<T> topic)
   {
      return new IHMCROS2Input<>(ros2NodeInterface, topic.getType(), topic);
   }

   @Override
   public <T> IHMCROS2Input<T> subscribe(ROS2Topic<T> topic, IHMCROS2Input.MessageFilter<T> messageFilter)
   {
      return new IHMCROS2Input<>(ros2NodeInterface, topic.getType(), topic, messageFilter);
   }

   @Override
   public ROS2TypelessInput subscribeTypeless(ROS2Topic<Empty> topic)
   {
      return new ROS2TypelessInput(ros2NodeInterface, topic);
   }

   @Override
   public Notification subscribeViaNotification(ROS2Topic<Empty> topic)
   {
      Notification notification = new Notification();
      new ROS2Callback<>(ros2NodeInterface, Empty.class, topic, message -> notification.set());
      return notification;
   }

   @Override
   public <T> TypedNotification<T> subscribeViaTypedNotification(ROS2Topic<T> topic)
   {
      TypedNotification<T> typedNotification = new TypedNotification<>();
      ROS2Tools.createCallbackSubscription2(ros2NodeInterface, topic, typedNotification::set);
      return typedNotification;
   }

   @Override
   public TypedNotification<Boolean> subscribeViaBooleanNotification(ROS2Topic<Bool> topic)
   {
      TypedNotification<Boolean> typedNotification = new TypedNotification<>();
      ROS2Tools.createCallbackSubscription2(ros2NodeInterface, topic, message -> typedNotification.set(message.getData()));
      return typedNotification;
   }

   @Override
   public <T> void createPublisher(ROS2Topic<T> topic)
   {
      ros2PublisherMap.getOrCreatePublisher(topic);
   }

   @Override
   public <T> void publish(ROS2Topic<T> topic, T message)
   {
      ros2PublisherMap.publish(topic, message);
   }

   @Override
   public void publish(ROS2Topic<std_msgs.msg.dds.String> topic, String message)
   {
      std_msgs.msg.dds.String stringMessage = new std_msgs.msg.dds.String();
      stringMessage.setData(message);
      ros2PublisherMap.publish(topic, stringMessage);
   }

   @Override
   public void publish(ROS2Topic<Pose3D> topic, Pose3D message)
   {
      ros2PublisherMap.publish(topic, message);
   }

   @Override
   public void publish(ROS2Topic<Empty> topic)
   {
      ros2PublisherMap.publish(topic);
   }

   @Override
   public void publish(ROS2Topic<Bool> topic, boolean message)
   {
      ros2PublisherMap.publish(topic, message);
   }

   public ROS2NodeInterface getROS2NodeInterface()
   {
      return ros2NodeInterface;
   }
}
