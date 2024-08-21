package us.ihmc.utilities.ros;

import org.ros.internal.message.Message;
import org.ros.message.Time;
import org.ros.node.parameter.ParameterListener;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.utilities.ros.publisher.RosTopicPublisher;
import us.ihmc.utilities.ros.subscriber.RosTopicSubscriberInterface;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

/**
 * This class should help the user
 * - Easily publish and subscribe
 * - Reconnect
 * - Remove and add topic subscriptions
 */
public class ROS1Helper implements RosNodeInterface
{
   private static final boolean ROS1_ENABLED = Boolean.parseBoolean(System.getProperty("ros1.enabled", "true"));

   private final String nodeName;
   private RosMainNode ros1Node;

   private final HashMap<RosTopicPublisher<? extends Message>, String> publishers = new HashMap<>();
   private final HashMap<RosTopicSubscriberInterface<? extends Message>, String> subscribers = new HashMap<>();

   private boolean needsReconnect = true;
   private final ScheduledExecutorService scheduler;
   private ScheduledFuture<?> scheduledFuture;

   public ROS1Helper(String nodeName)
   {
      this.nodeName = nodeName;
      scheduler = ThreadTools.newSingleDaemonThreadScheduledExecutor("ROS1HelperMaintenance");
   }

   // TODO: Automate what ImGuiGDXROS1Visualizer is doing without putting the burden on the user

   private void ensureConnected()
   {
      scheduledFuture = null;
      if (needsReconnect)
      {
         needsReconnect = false;
         reconnectEverything();
      }
   }

   /**
    * Automatically tries to reconnect after new publishers are added,
    * but only 1/3 of a second after nothing new has been attached.
    */
   private void scheduleTentativeReconnect()
   {
      if (scheduledFuture != null)
      {
         scheduledFuture.cancel(false);
      }
      scheduledFuture = scheduler.schedule(() -> ExceptionTools.handle(this::ensureConnected, e -> LogTools.error(e.getMessage())),
                                           333, TimeUnit.MILLISECONDS);
   }

   public void reconnectEverything()
   {
      if (ROS1_ENABLED)
      {
         LogTools.info("Reconnecting {} ROS 1 node...", nodeName);
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
      }
      else
      {
         LogTools.warn("ROS 1 is disabled. Not connecting ROS 1 node.", nodeName);
      }
   }

   @Override
   public void attachPublisher(String topicName, RosTopicPublisher<? extends Message> publisher)
   {
      publishers.put(publisher, topicName);
      needsReconnect = true;
      scheduleTentativeReconnect();
   }

//   public RosTopicPublisher<PoseStamped> publishPose(String topicName)
//   {
//      boolean latched = false; // TODO: What does this mean?
//      RosTopicPublisher<PoseStamped> publisher = new RosTopicPublisher<PoseStamped>(PoseStamped._TYPE, latched)
//      {
//         // ???
//      };
//      attachPublisher(topicName, publisher);
//      return publisher;
//   }

   @Override
   public void attachSubscriber(String topicName, RosTopicSubscriberInterface<? extends Message> subscriber)
   {
      subscribers.put(subscriber, topicName);
      needsReconnect = true;
      scheduleTentativeReconnect();
   }

//   public AbstractRosTopicSubscriber<PoseStamped> subscribeToPoseViaCallback(String topicName, Consumer<PoseStamped> callback)
//   {
//      AbstractRosTopicSubscriber<PoseStamped> subscriber = new AbstractRosTopicSubscriber<PoseStamped>(PoseStamped._TYPE)
//      {
//         @Override
//         public void onNewMessage(PoseStamped poseStamped)
//         {
//            callback.accept(poseStamped);
//         }
//      };
//      attachSubscriber(topicName, subscriber);
//      return subscriber;
//   }

//   public AbstractRosTopicSubscriber<PointCloud2> subscribeToPointCloud2ViaCallback(String topicName, Consumer<PointCloud2> callback)
//   {
//      AbstractRosTopicSubscriber<PointCloud2> subscriber = new AbstractRosTopicSubscriber<PointCloud2>(PointCloud2._TYPE)
//      {
//         @Override
//         public void onNewMessage(PointCloud2 pointCloud2)
//         {
//            callback.accept(pointCloud2);
//         }
//      };
//      attachSubscriber(topicName, subscriber);
//      return subscriber;
//   }

//   public AbstractRosTopicSubscriber<Image> subscribeToImageViaCallback(String topicName, Consumer<Image> callback)
//   {
//      AbstractRosTopicSubscriber<Image> subscriber = new AbstractRosTopicSubscriber<Image>(sensor_msgs.Image._TYPE)
//      {
//         @Override
//         public void onNewMessage(Image image)
//         {
//            callback.accept(image);
//         }
//      };
//      attachSubscriber(topicName, subscriber);
//      return subscriber;
//   }

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
      return ros1Node != null && ros1Node.isStarted();
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
      scheduler.shutdown();
      if (ros1Node != null)
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
