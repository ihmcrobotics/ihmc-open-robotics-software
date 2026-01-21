package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * A ROS 2 frame with a static transform to parent.
 */
public class ROS2StaticFrame extends ROS2Frame
{
   private final boolean publishMessage;

   /**
    * Constructs a non-root frame.
    *
    * @param id                The frame's id.
    * @param parentFrame       The parent frame.
    * @param transformToParent Transform to the parent frame.
    */
   public ROS2StaticFrame(String id, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      this(id, parentFrame, transformToParent, false, false);
   }

   /**
    * Constructs a non-root frame.
    *
    * @param id                 The frame's id.
    * @param parentFrame        The parent frame.
    * @param transformToParent  Transform to the parent frame.
    * @param isAStationaryFrame Whether this frame is stationary with respect to root frame.
    * @param isZUpFrame         Whether this frame has its Z-axis aligned with root frame at all times.
    */
   public ROS2StaticFrame(String id, ReferenceFrame parentFrame, RigidBodyTransformReadOnly transformToParent, boolean isAStationaryFrame, boolean isZUpFrame)
   {
      super(id, parentFrame, transformToParent, isAStationaryFrame, isZUpFrame, true);

      publishMessage = shouldPublishMessage(parentFrame);
   }

   private ROS2StaticFrame(String id)
   {
      this(id, null, null, true, true);
   }

   /**
    * Constructs a root frame.
    *
    * @param id The frame's id.
    * @return The root frame.
    */
   public static ROS2StaticFrame constructARootFrame(String id)
   {
      return new ROS2StaticFrame(id);
   }

   private boolean shouldPublishMessage(ReferenceFrame parentFrame)
   {
      if (parentFrame instanceof ROS2Frame || parentFrame.isRootFrame())
         return false;

      if (!parentFrame.isFixedInParent())
         return true;

      return shouldPublishMessage(parentFrame.getParent());
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      // Transform to parent should not change in a static frame
   }

   @Override
   protected void onNewTransformReceived(TransformStamped newTransform)
   {
      if (isRootFrame())
         return;

      if (!newTransform.getTransform().geometricallyEquals(getTransformToParent(), 1E-4))
         throw new IllegalStateException("Transform of received message does not match static transform.");
   }

   @Override
   public void update()
   {
      super.update();

      if (publishMessage)
         publishTFMessages();
   }

   public boolean publishesMessageOnUpdate()
   {
      return publishMessage;
   }
}
