package us.ihmc.communication.ros2.tf2;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * A ROS 2 frame with a static transform to parent.
 */
public class ROS2StaticFrame extends ROS2Frame
{
   private final boolean publishMessage;
   private boolean firstPublish;

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
      firstPublish = true;

      postConstruction();
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
      if (firstPublish || publishMessage)
      {
         markUpdateTime();
         publishTFMessages();
         firstPublish = false;
      }
   }

   public boolean publishesMessageOnUpdate()
   {
      return publishMessage;
   }
}
