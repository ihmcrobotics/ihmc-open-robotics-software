package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import tf2_msgs.msg.dds.TFMessage;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;

public class ROS2FrameTools
{
   public static final ROS2Topic<TFMessage> TF_TOPIC = new ROS2Topic<>().withModule("tf").withQoS(ROS2QosProfile.RELIABLE()).withType(TFMessage.class);
   public static final ROS2Topic<TFMessage> TF_STATIC_TOPIC = new ROS2Topic<>().withModule("tf_static")
                                                                               .withQoS(ROS2QosProfile.KEEP_HISTORY(1))
                                                                               .withType(TFMessage.class);

   public static void packTransformMessage(ReferenceFrame frame, int timestampSeconds, int timestampNanos, TransformStamped messageToPack)
   {
      if (frame.isRootFrame())
         throw new IllegalArgumentException("Cannot pack the transform message for a root frame");

      messageToPack.getHeader().getStamp().setSec(timestampSeconds);
      messageToPack.getHeader().getStamp().setNanosec(timestampNanos);
      messageToPack.getHeader().setFrameId(frame.getParent().getName());
      messageToPack.getTransform().set(frame.getTransformToParent());
      messageToPack.setChildFrameId(frame.getName());
   }

   public static void packTFMessages(ReferenceFrame frame, int timestampSeconds, int timestampNanos, TFMessage tfMessage, TFMessage tfStaticMessage)
   {
      // Once we have no parent (frame is root), we stop
      if (frame.isRootFrame())
         return;

      // Get the message we want to pack into
      TFMessage messageToPack = frame.isFixedInParent() ? tfStaticMessage : tfMessage;

      // Add a transform message and pack it
      ROS2FrameTools.packTransformMessage(frame, timestampSeconds, timestampNanos, messageToPack.getTransforms().add());

      // If the parent isn't a ROS2Frame, pack the parent into the TFMessage to ensure full TF tree
      if (!(frame.getParent() instanceof ROS2Frame))
         packTFMessages(frame.getParent(), timestampSeconds, timestampNanos, tfMessage, tfStaticMessage);
   }
}
