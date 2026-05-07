package us.ihmc.communication.ros2.tf2;

import geometry_msgs.TransformStamped;
import tf2_msgs.msg.dds.TFMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Subscription;
import us.ihmc.jros2.ROS2Topic;

import java.util.Map;
import java.util.concurrent.ConcurrentSkipListMap;

@SuppressWarnings("ForLoopReplaceableByForEach")
public class ROS2TFTree
{
   public static final ROS2Topic<TFMessage> TF_TOPIC = new ROS2Topic<>().appendedWith("tf").withType(TFMessage.class);
   public static final ROS2Topic<TFMessage> TF_STATIC_TOPIC = new ROS2Topic<>().appendedWith("tf_static")
                                                                               
                                                                               .withType(TFMessage.class);

   private static ROS2TFTree instance = null;

   public static synchronized ROS2TFTree getInstance()
   {
      if (instance == null)
         instance = new ROS2TFTree();

      return instance;
   }

   /** Contains all transforms known to this process */
   private final Map<CharSequence, TransformStamped> transforms;

   private final ROS2Node ros2Node;

   private final ROS2Subscription<TFMessage> tfSubscription;
   private final TFMessage tfMessage;

   private final ROS2Subscription<TFMessage> tfStaticSubscription;
   private final TFMessage tfStaticMessage;

   private ROS2TFTree()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::close, getClass().getSimpleName() + "Shutdown"));

      transforms = new ConcurrentSkipListMap<>(CharSequence::compare);

      ros2Node = new ROS2Node("TFNode");

      tfMessage = new TFMessage();
      tfSubscription = ros2Node.createSubscription(TF_TOPIC, subscriber -> receiveTFMessage(subscriber, tfMessage));

      tfStaticMessage = new TFMessage();
      tfStaticSubscription = ros2Node.createSubscription(TF_STATIC_TOPIC, subscriber -> receiveTFMessage(subscriber, tfStaticMessage));
   }

   public Map<CharSequence, TransformStamped> getTransforms()
   {
      return transforms;
   }

   public ROS2Node getTFNode()
   {
      return ros2Node;
   }

   private void receiveTFMessage(@SuppressWarnings("deprecation") Subscriber<TFMessage> subscriber, TFMessage tfMessage)
   {
      // Read the new message
      subscriber.takeNextData(tfMessage, null);

      // Ignore null or empty messages
      if (tfMessage == null || tfMessage.getTransforms().isEmpty())
         return;

      // Update the transforms
      RecyclingArrayList<TransformStamped> receivedTransforms = tfMessage.getTransforms();
      for (int i = 0; i < receivedTransforms.size(); ++i)
      {
         TransformStamped receivedMessage = receivedTransforms.get(i);
         TransformStamped recordedTransform = transforms.get(receivedMessage.getChildFrameId());
         if (recordedTransform == null)
         {
            recordedTransform = new TransformStamped(receivedMessage);
            transforms.put(recordedTransform.getChildFrameId(), recordedTransform);
         }
         else if (MessageTools.compareTime(recordedTransform.getHeader().getStamp(), receivedMessage.getHeader().getStamp()) < 0)
         {
            recordedTransform.set(receivedMessage);
         }
      }
   }

   private void close()
   {
      if (tfSubscription != null)
         tfSubscription.remove();
      if (tfStaticSubscription != null)
         tfStaticSubscription.remove();
      if (ros2Node != null)
         ros2Node.destroy();
   }
}
