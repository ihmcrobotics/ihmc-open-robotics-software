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

   private int lastPublishTimestampSeconds;
   private int lastPublishTimestampNanos;

   private volatile boolean hasBeenRemoved = false;

   protected ROS2Frame(String id,
                       ReferenceFrame parentFrame,
                       RigidBodyTransformReadOnly transformToParent,
                       boolean isAStationaryFrame,
                       boolean isZUpFrame,
                       boolean isStatic)
   {
      super(id, parentFrame, transformToParent, isAStationaryFrame, isZUpFrame, isStatic);

      ROS2Node ros2Node = TFTree.getInstance().getTFNode();
      TFTree.getInstance().registerFrame(this);

      tfMessageToPublish = new TFMessage();
      tfPublisher = ros2Node.createPublisher(TF_TOPIC);

      tfStaticMessageToPublish = new TFMessage();
      tfStaticPublisher = ros2Node.createPublisher(TF_STATIC_TOPIC);

      publishTFMessages();
   }

   void onTransformReceived(TransformStamped transform)
   {
      // Check whether the matching transform is newer than our transform. If it is, trigger onNewTransformReceived method.
      int matchingTimestampSeconds = transform.getHeader().getStamp().getSec();
      int matchingTimestampNanos = (int) transform.getHeader().getStamp().getNanosec();
      if (matchingTimestampSeconds > lastPublishTimestampSeconds || (matchingTimestampSeconds == lastPublishTimestampSeconds
                                                                           && matchingTimestampNanos > lastPublishTimestampNanos))
      {
         onNewTransformReceived(transform);
      }
   }

   protected abstract void onNewTransformReceived(TransformStamped newTransform);

   protected void publishTFMessages()
   {
      if (hasBeenRemoved)
         return;

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
      if (hasBeenRemoved)
         return;
      hasBeenRemoved = true;

      super.remove();
      if (tfPublisher != null)
         tfPublisher.remove();
      if (tfStaticPublisher != null)
         tfStaticPublisher.remove();
   }
}