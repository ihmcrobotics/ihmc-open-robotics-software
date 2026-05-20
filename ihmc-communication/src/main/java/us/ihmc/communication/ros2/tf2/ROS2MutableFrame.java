package us.ihmc.communication.ros2.tf2;

import geometry_msgs.TransformStamped;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

/**
 * A ROS 2 frame with a changing transform to parent.
 */
public class ROS2MutableFrame extends ROS2Frame
{
   private final RigidBodyTransform newestTransformToParent = new RigidBodyTransform();
   private final AtomicBoolean hasNewTransform = new AtomicBoolean(false);

   private TransformStamped remoteTransform;

   /**
    * Constructs a non-root frame.
    *
    * @param id          The frame's id.
    * @param parentFrame The parent frame.
    */
   public ROS2MutableFrame(String id, ReferenceFrame parentFrame)
   {
      this(id, parentFrame, null);
   }

   /**
    * Constructs a non-root frame.
    *
    * @param id                The frame's id.
    * @param parentFrame       The parent frame.
    * @param transformToParent Transform to the parent frame.
    */
   public ROS2MutableFrame(String id, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      this(id, parentFrame, transformToParent, false);
   }

   /**
    * Constructs a non-root frame.
    *
    * @param id                The frame's id.
    * @param parentFrame       The parent frame.
    * @param transformToParent Transform to the parent frame.
    * @param isZUpFrame        Whether this frame has its Z-axis aligned with root frame at all times.
    */
   public ROS2MutableFrame(String id, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent, boolean isZUpFrame)
   {
      super(id, parentFrame, transformToParent, false, isZUpFrame, false);
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
      // TODO jros2
      if (remoteTransform == null)
         remoteTransform = ROS2TFTree.getInstance().getTransforms().get(getFrameId());

      if (hasNewTransform.getAndSet(false))
         transformToParent.set(newestTransformToParent);
      else if (remoteTransform != null && MessageTools.compareTime(updateTime, remoteTransform.getHeader().getStamp()) < 0)
//         transformToParent.set(remoteTransform.getTransform());

      markUpdateTime();
      publishTFMessages();
   }
}
