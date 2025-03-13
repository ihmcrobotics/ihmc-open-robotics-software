package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import tf2_msgs.msg.dds.TFMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;

import java.time.Instant;
import java.util.List;

import static us.ihmc.communication.ros2.tf2.ROS2FrameTools.TF_STATIC_TOPIC;
import static us.ihmc.communication.ros2.tf2.ROS2FrameTools.TF_TOPIC;

public abstract class ROS2Frame extends ReferenceFrame
{
   private final ROS2Publisher<TFMessage> transformPublisher;
   private final ROS2Subscription<TFMessage> tfSubscription;
   private final TFMessage tfMessageToPublish;
   private final TFMessage tfMessageToReceive;

   protected int lastUpdateTimestampSeconds;
   protected int lastUpdateTimestampNanos;

   protected ROS2Frame(ROS2Node ros2Node,
                       String id,
                       ROS2Frame parentFrame,
                       RigidBodyTransformReadOnly transformToParent,
                       int transformTimestampSeconds,
                       int transformTimestampNanos,
                       boolean isAStationaryFrame,
                       boolean isZUpFrame,
                       boolean isStatic)
   {
      super(id, parentFrame, transformToParent, isAStationaryFrame, isZUpFrame, isStatic);

      lastUpdateTimestampSeconds = transformTimestampSeconds;
      lastUpdateTimestampNanos = transformTimestampNanos;

      if (ros2Node != null)
      {
         ROS2Topic<TFMessage> tfTopic = isStatic ? TF_STATIC_TOPIC : TF_TOPIC;

         tfMessageToPublish = new TFMessage();
         transformPublisher = ros2Node.createPublisher(tfTopic);

         tfMessageToReceive = new TFMessage();
         tfSubscription = ros2Node.createSubscription(tfTopic, this::receiveTFMessage);
      }
      else
      {
         tfMessageToPublish = null;
         transformPublisher = null;

         tfMessageToReceive = null;
         tfSubscription = null;
      }
   }

   private void receiveTFMessage(@SuppressWarnings("deprecation") Subscriber<TFMessage> subscriber)
   {
      // Read the new message
      subscriber.takeNextData(tfMessageToReceive, null);

      // Ignore null or empty messages
      if (tfMessageToReceive == null || tfMessageToReceive.getTransforms().isEmpty())
         return;

      // Find the matching transform (if it exists)
      List<TransformStamped> transforms = tfMessageToReceive.getTransforms();
      TransformStamped matchingTransform = null;
      //noinspection ForLoopReplaceableByForEach
      for (int i = 0; i < transforms.size(); ++i)
      {
         TransformStamped transform = transforms.get(i);
         if (transform.getChildFrameIdAsString().equals(getFrameId()))
         {
            matchingTransform = transform;
            break;
         }
      }

      // Do nothing if the message doesn't contain a matching transform
      if (matchingTransform == null)
         return;

      // Check whether the matching transform is newer than our transform. If it is, trigger onNewTransformReceived method.
      int matchingTimestampSeconds = matchingTransform.getHeader().getStamp().getSec();
      int matchingTimestampNanos = (int) matchingTransform.getHeader().getStamp().getNanosec();
      if (matchingTimestampSeconds > lastUpdateTimestampSeconds || (matchingTimestampSeconds == lastUpdateTimestampSeconds
                                                                    && matchingTimestampNanos > lastUpdateTimestampNanos))
      {
         onNewTransformReceived(matchingTransform);
      }
   }

   protected abstract void onNewTransformReceived(TransformStamped newTransform);

   /**
    * Get this frame's id string.
    * <p>
    * Same as {@link #getName()}.
    *
    * @return This frame's id string.
    */
   public String getFrameId()
   {
      return getName();
   }

   public Instant getLastUpdateTimestamp()
   {
      return Instant.ofEpochSecond(lastUpdateTimestampSeconds, lastUpdateTimestampNanos);
   }

   public int getLastUpdateTimestampSeconds()
   {
      return lastUpdateTimestampSeconds;
   }

   public int getLastUpdateTimestampNanos()
   {
      return lastUpdateTimestampNanos;
   }

   public void publish()
   {
      if (transformPublisher == null || getParent() == null)
         return;

      tfMessageToPublish.getTransforms().clear();
      TransformStamped transformMessage = tfMessageToPublish.getTransforms().add();

      transformMessage.getHeader().getStamp().setSec(lastUpdateTimestampSeconds);
      transformMessage.getHeader().getStamp().setNanosec(lastUpdateTimestampNanos);
      transformMessage.getHeader().setFrameId(getParent().getFrameId());
      transformMessage.getTransform().set(getTransformToParent());
      transformMessage.setChildFrameId(getFrameId());

      transformPublisher.publish(tfMessageToPublish);
   }

   @Override
   public boolean isWorldFrame()
   {
      return this == ROS2FrameTools.getWorldFrame();
   }

   @Override
   public ROS2Frame getParent()
   {
      return (ROS2Frame) super.getParent();
   }

   @Override
   public RigidBodyTransform getTransformToWorldFrame()
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      getTransformToDesiredFrame(ret, ROS2FrameTools.getWorldFrame());
      return ret;
   }

   @Override
   public void remove()
   {
      super.remove();
      if (transformPublisher != null)
         transformPublisher.remove();
      if (tfSubscription != null)
         tfSubscription.remove();
   }
}