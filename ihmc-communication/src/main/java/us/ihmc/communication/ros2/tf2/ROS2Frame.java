package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import tf2_msgs.msg.dds.TFMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import java.time.Instant;

import static us.ihmc.communication.ros2.tf2.ROS2FrameTools.TF_STATIC_TOPIC;
import static us.ihmc.communication.ros2.tf2.ROS2FrameTools.TF_TOPIC;

public abstract class ROS2Frame extends ReferenceFrame
{
   private final ROS2Publisher<TFMessage> transformPublisher;
   private final TFMessage tfMessage;

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
         transformPublisher = ros2Node.createPublisher(isStatic ? TF_STATIC_TOPIC : TF_TOPIC);
         tfMessage = new TFMessage();
      }
      else
      {
         transformPublisher = null;
         tfMessage = null;
      }
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
      if (transformPublisher == null)
         return;

      tfMessage.getTransforms().clear();
      TransformStamped transformMessage = tfMessage.getTransforms().add();

      transformMessage.getHeader().getStamp().setSec(lastUpdateTimestampSeconds);
      transformMessage.getHeader().getStamp().setNanosec(lastUpdateTimestampNanos);
      transformMessage.getHeader().setFrameId(getFrameId());
      transformMessage.getTransform().set(getTransformToParent());

      transformPublisher.publish(tfMessage);
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
   }
}