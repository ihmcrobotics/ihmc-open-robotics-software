package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import tf2_msgs.msg.dds.TFMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;

import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentSkipListMap;

@SuppressWarnings("ForLoopReplaceableByForEach")
public class ROS2TFTree
{
   public static final ROS2Topic<TFMessage> TF_TOPIC = new ROS2Topic<>().withModule("tf").withQoS(ROS2QosProfile.RELIABLE()).withType(TFMessage.class);
   public static final ROS2Topic<TFMessage> TF_STATIC_TOPIC = new ROS2Topic<>().withModule("tf_static")
                                                                               .withQoS(ROS2QosProfile.KEEP_HISTORY(1))
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
      Runtime.getRuntime().addShutdownHook(new Thread(this::close));

      transforms = new ConcurrentSkipListMap<>(CharSequence::compare);

      ros2Node = new ROS2NodeBuilder().build("TFNode");

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
         TransformStamped transformMessage = receivedTransforms.get(i);
         TransformStamped recordedTransform = transforms.get(transformMessage.getChildFrameId());
         if (recordedTransform != null)
            recordedTransform.set(transformMessage);
         else
         {
            recordedTransform = new TransformStamped(transformMessage);
            transforms.put(recordedTransform.getChildFrameId(), recordedTransform);
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
