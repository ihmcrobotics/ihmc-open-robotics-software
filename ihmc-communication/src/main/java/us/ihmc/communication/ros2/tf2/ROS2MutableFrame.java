package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.ros2.ROS2Node;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

/**
 * A ROS 2 frame with a changing transform to parent.
 */
public class ROS2MutableFrame extends ROS2Frame
{
   private final RigidBodyTransform newestTransformToParent;
   private final AtomicBoolean hasNewTransform = new AtomicBoolean(false);

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node    ROS 2 node to publish the TFMessage on.
    * @param id          The frame's id.
    * @param parentFrame The parent frame.
    */
   public ROS2MutableFrame(ROS2Node ros2Node, String id, ReferenceFrame parentFrame)
   {
      this(ros2Node, id, parentFrame, null);
   }

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node                  ROS 2 node to publish the TFMessage on.
    * @param id                        The frame's id.
    * @param parentFrame               The parent frame.
    * @param transformToParent         Transform to the parent frame.
    */
   public ROS2MutableFrame(ROS2Node ros2Node,
                           String id,
                           ReferenceFrame parentFrame,
                           RigidBodyTransformReadOnly transformToParent)
   {
      this(ros2Node, id, parentFrame, transformToParent, false);
   }

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node                  ROS 2 node to publish the TFMessage on.
    * @param id                        The frame's id.
    * @param parentFrame               The parent frame.
    * @param transformToParent         Transform to the parent frame.
    * @param isZUpFrame                Whether this frame has its Z-axis aligned with root frame at all times.
    */
   public ROS2MutableFrame(ROS2Node ros2Node,
                           String id,
                           ReferenceFrame parentFrame,
                           RigidBodyTransformReadOnly transformToParent,
                           boolean isZUpFrame)
   {
      super(ros2Node, id, parentFrame, transformToParent, false, isZUpFrame, false);

      newestTransformToParent = new RigidBodyTransform();
      getTransformToParent(newestTransformToParent);
   }

   /**
    * Set this frame's transform to parent.
    * Will be applied on the next call to {@link #update()}.
    *
    * @param newTransformToParent The new transform to parent for this frame.
    */
   public void setNewTransformToParent(RigidBodyTransformReadOnly newTransformToParent)
   {
      newestTransformToParent.set(newTransformToParent);
      hasNewTransform.set(true);
   }

   /**
    * Set this frame's transform to parent.
    * Will be applied on the next call to {@link #update()}.
    *
    * @param transformUpdater Method applied to this frame's transform to parent.
    */
   public void setNewTransformToParent(Consumer<RigidBodyTransform> transformUpdater)
   {
      transformUpdater.accept(newestTransformToParent);
      hasNewTransform.set(true);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      if (hasNewTransform.getAndSet(false))
         transformToParent.set(newestTransformToParent);
   }

   @Override
   protected void onNewTransformReceived(TransformStamped newTransform)
   {
      setNewTransformToParent(newTransform.getTransform());
   }

   @Override
   public void update()
   {
      super.update();
      publishTFMessages();
   }
}
