package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import tf2_msgs.msg.dds.TFMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Subscription;

import java.util.List;

import static us.ihmc.communication.ros2.tf2.ROS2FrameTools.*;

public abstract class ROS2Frame extends ReferenceFrame
{
   private final TFMessage tfMessageToPublish;
   private final ROS2Publisher<TFMessage> tfPublisher;

   private final TFMessage tfStaticMessageToPublish;
   private final ROS2Publisher<TFMessage> tfStaticPublisher;

   private final TFMessage tfMessageToReceive;
   private final ROS2Subscription<TFMessage> tfSubscription;

   private int lastPublishTimestampSeconds;
   private int lastPublishTimestampNanos;

   protected ROS2Frame(ROS2Node ros2Node,
                       String id,
                       ReferenceFrame parentFrame,
                       RigidBodyTransformReadOnly transformToParent,
                       boolean isAStationaryFrame,
                       boolean isZUpFrame,
                       boolean isStatic)
   {
      super(id, parentFrame, transformToParent, isAStationaryFrame, isZUpFrame, isStatic);

      if (ros2Node != null)
      {
         tfMessageToPublish = new TFMessage();
         tfPublisher = ros2Node.createPublisher(TF_TOPIC);

         tfStaticMessageToPublish = new TFMessage();
         tfStaticPublisher = ros2Node.createPublisher(TF_STATIC_TOPIC);

         tfMessageToReceive = new TFMessage();
         tfSubscription = ros2Node.createSubscription(isStatic ? TF_STATIC_TOPIC : TF_TOPIC, this::receiveTFMessage);
      }
      else
      {
         tfPublisher = null;
         tfMessageToPublish = null;

         tfStaticPublisher = null;
         tfStaticMessageToPublish = null;

         tfSubscription = null;
         tfMessageToReceive = null;
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
      if (matchingTimestampSeconds > lastPublishTimestampSeconds || (matchingTimestampSeconds == lastPublishTimestampSeconds
                                                                     && matchingTimestampNanos > lastPublishTimestampNanos))
      {
         onNewTransformReceived(matchingTransform);
      }
   }

   protected abstract void onNewTransformReceived(TransformStamped newTransform);

   protected void publishTFMessages()
   {
      long currentTimeMillis = System.currentTimeMillis();
      lastPublishTimestampSeconds = (int) (currentTimeMillis / 1000);
      lastPublishTimestampNanos = (int) (currentTimeMillis % 1000) * 1000000;

      tfMessageToPublish.getTransforms().clear();
      tfStaticMessageToPublish.getTransforms().clear();

      packTFMessages(this, lastPublishTimestampSeconds, lastPublishTimestampNanos, tfMessageToPublish, tfStaticMessageToPublish);

      if (!tfMessageToPublish.getTransforms().isEmpty())
         tfPublisher.publish(tfMessageToPublish);
      if (!tfStaticMessageToPublish.getTransforms().isEmpty())
         tfStaticPublisher.publish(tfStaticMessageToPublish);
   }

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

   @Override
   public void remove()
   {
      super.remove();
      if (tfPublisher != null)
         tfPublisher.remove();
      if (tfStaticPublisher != null)
         tfStaticPublisher.remove();
      if (tfSubscription != null)
         tfSubscription.remove();
   }
}