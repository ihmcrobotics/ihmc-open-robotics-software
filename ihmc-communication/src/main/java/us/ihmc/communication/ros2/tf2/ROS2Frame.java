package us.ihmc.communication.ros2.tf2;

import builtin_interfaces.msg.dds.Time;
import geometry_msgs.TransformStamped;
import tf2_msgs.msg.dds.TFMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;

import static us.ihmc.communication.ros2.tf2.ROS2FrameTools.*;

public abstract class ROS2Frame extends ReferenceFrame
{
   private final TFMessage tfMessageToPublish;
   private final ROS2Publisher<TFMessage> tfPublisher;

   private final TFMessage tfStaticMessageToPublish;
   private final ROS2Publisher<TFMessage> tfStaticPublisher;

   protected final Time updateTime;

   private volatile boolean hasBeenRemoved = false;

   protected ROS2Frame(String id,
                       ReferenceFrame parentFrame,
                       RigidBodyTransformReadOnly transformToParent,
                       boolean isAStationaryFrame,
                       boolean isZUpFrame,
                       boolean isStatic)
   {
      super(id, parentFrame, transformToParent, isAStationaryFrame, isZUpFrame, isStatic);

      updateTime = new Time();
      markUpdateTime();

      ROS2Node ros2Node = ROS2TFTree.getInstance().getTFNode();

      tfMessageToPublish = new TFMessage();
      tfPublisher = ros2Node.createPublisher(TF_TOPIC);

      tfStaticMessageToPublish = new TFMessage();
      tfStaticPublisher = ros2Node.createPublisher(TF_STATIC_TOPIC);
   }

   protected void markUpdateTime()
   {
      long currentTimeMillis = System.currentTimeMillis();
      updateTime.setSec((int) (currentTimeMillis / 1000));
      updateTime.setNanosec((currentTimeMillis % 1000) * 1000000);
   }

   protected void publishTFMessages()
   {
      if (hasBeenRemoved)
         return;

      tfMessageToPublish.getTransforms().clear();
      tfStaticMessageToPublish.getTransforms().clear();

      packTFMessages(this, updateTime, tfMessageToPublish, tfStaticMessageToPublish);

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
