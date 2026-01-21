package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import tf2_msgs.msg.dds.TFMessage;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Consumer;

@SuppressWarnings("ForLoopReplaceableByForEach")
public final class TFTree
{
   public static final ROS2Topic<TFMessage> TF_TOPIC = new ROS2Topic<>().withModule("tf").withQoS(ROS2QosProfile.RELIABLE()).withType(TFMessage.class);
   public static final ROS2Topic<TFMessage> TF_STATIC_TOPIC = new ROS2Topic<>().withModule("tf_static")
                                                                               .withQoS(ROS2QosProfile.KEEP_HISTORY(1))
                                                                               .withType(TFMessage.class);

   private static TFTree instance = null;

   public static synchronized TFTree getInstance()
   {
      if (instance == null)
         instance = new TFTree();

      return instance;
   }

   /** Contains all ROS2Frames known to this process */
   private final Map<CharSequence, ROS2Frame> frames;
   /** List of consumers that will be triggered when a new frame is created due to a TFMessage containing a frame unknown to this process */
   private final List<Consumer<ROS2Frame>> newFrameConsumers;

   private final ROS2Node ros2Node;

   private final ROS2Subscription<TFMessage> tfSubscription;
   private final TFMessage tfMessage;

   private final ROS2Subscription<TFMessage> tfStaticSubscription;
   private final TFMessage tfStaticMessage;

   private TFTree()
   {
      Runtime.getRuntime().addShutdownHook(new Thread(this::close));

      frames = new TreeMap<>(CharSequence::compare);
      newFrameConsumers = new ArrayList<>();

      ros2Node = new ROS2NodeBuilder().build("TFNode");

      tfSubscription = ros2Node.createSubscription(TF_TOPIC, this::receiveTFMessage);
      tfMessage = new TFMessage();

      tfStaticSubscription = ros2Node.createSubscription(TF_STATIC_TOPIC, this::receiveStaticTFMessage);
      tfStaticMessage = new TFMessage();
   }

   public void registerNewFrameConsumer(Consumer<ROS2Frame> newFrameConsumer)
   {
      newFrameConsumers.add(newFrameConsumer);
   }

   public Map<CharSequence, ROS2Frame> getFrames()
   {
      return frames;
   }

   synchronized void registerFrame(ROS2Frame frame)
   {
      frames.put(frame.getName(), frame);
   }

   ROS2Node getTFNode()
   {
      return ros2Node;
   }

   private void receiveTFMessage(@SuppressWarnings("deprecation") Subscriber<TFMessage> subscriber)
   {
      // Read the new message
      subscriber.takeNextData(tfMessage, null);

      // Ignore null or empty messages
      if (tfMessage == null || tfMessage.getTransforms().isEmpty())
         return;

      // Get the transforms contained in the message
      List<TransformStamped> transforms = tfMessage.getTransforms();

      // Process the transforms
      processTransforms(transforms, false);
   }

   private void receiveStaticTFMessage(@SuppressWarnings("deprecation") Subscriber<TFMessage> subscriber)
   {
      // Read the new message
      subscriber.takeNextData(tfStaticMessage, null);

      // Ignore null or empty messages
      if (tfStaticMessage == null || tfStaticMessage.getTransforms().isEmpty())
         return;

      // Get the transforms contained in the message
      List<TransformStamped> transforms = tfStaticMessage.getTransforms();

      // Process the transforms
      processTransforms(transforms, true);
   }

   private void processTransforms(List<TransformStamped> transforms, boolean isStatic)
   {
      for (int i = 0; i < transforms.size(); ++i)
      {
         TransformStamped transform = transforms.get(i);
         ROS2Frame childFrame = frames.get(transform.getChildFrameId());
         ROS2Frame parentFrame = frames.get(transform.getHeader().getFrameId());

         // We can't deal with this transform if the parent frame doesn't exist
         if (parentFrame == null)
            continue;

         // If the child frame is part of this tree, update its transform to parent
         if (childFrame != null)
         {
            childFrame.onTransformReceived(transform);
         }
         // Otherwise we create a new ROS2Frame and let consumers know
         else
         {
            childFrame = isStatic ?
                  new ROS2StaticFrame(transform.getChildFrameIdAsString(), parentFrame, transform.getTransform()) :
                  new ROS2MutableFrame(transform.getChildFrameIdAsString(), parentFrame, transform.getTransform());
            registerFrame(childFrame);
            for (int j = 0; j < newFrameConsumers.size(); ++j)
               newFrameConsumers.get(j).accept(childFrame);
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

      frames.values().forEach(ROS2Frame::remove);
   }
}
