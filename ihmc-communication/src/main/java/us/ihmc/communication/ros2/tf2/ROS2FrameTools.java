package us.ihmc.communication.ros2.tf2;

import builtin_interfaces.Time;
import geometry_msgs.TransformStamped;
import tf2_msgs.TFMessage;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.jros2.ROS2Topic;

public class ROS2FrameTools
{
   public static final ROS2Topic<TFMessage> TF_TOPIC = new ROS2Topic<>().appendedWith("tf").withType(TFMessage.class);
   public static final ROS2Topic<TFMessage> TF_STATIC_TOPIC = new ROS2Topic<>().appendedWith("tf_static")

                                                                               .withType(TFMessage.class);

   // Read about optical frames here: https://ros.org/reps/rep-0103.html#suffix-frames
   public static final Orientation3DReadOnly CAMERA_TO_OPTICAL_ROTATION = new YawPitchRoll(-0.5 * Math.PI, 0.0, -0.5 * Math.PI);
   public static final RigidBodyTransformReadOnly CAMERA_TO_OPTICAL_TRANSFORM = new RigidBodyTransform(CAMERA_TO_OPTICAL_ROTATION, new Vector3D());

   public static void packTransformMessage(ReferenceFrame frame, Time timestamp, TransformStamped messageToPack)
   {
      if (frame.isRootFrame())
         throw new IllegalArgumentException("Cannot pack the transform message for a root frame");

      messageToPack.getHeader().getStamp().set(timestamp);
      messageToPack.getHeader().setFrameId(frame.getParent().getName());
      // TODO jros2
//      messageToPack.getTransform().set(frame.getTransformToParent());
      messageToPack.setChildFrameId(frame.getName());
   }

   public static void packTFMessages(ReferenceFrame frame, Time timestamp, TFMessage tfMessage, TFMessage tfStaticMessage)
   {
      // Once we have no parent (frame is root), we stop
      if (frame.isRootFrame())
         return;

      // Get the message we want to pack into
      TFMessage messageToPack = frame.isFixedInParent() ? tfStaticMessage : tfMessage;

      // Add a transform message and pack it
      ROS2FrameTools.packTransformMessage(frame, timestamp, messageToPack.getTransforms().add());

      // If the parent isn't a ROS2Frame, pack the parent into the TFMessage to ensure full TF tree
      if (!(frame.getParent() instanceof ROS2Frame))
         packTFMessages(frame.getParent(), timestamp, tfMessage, tfStaticMessage);
   }
}
