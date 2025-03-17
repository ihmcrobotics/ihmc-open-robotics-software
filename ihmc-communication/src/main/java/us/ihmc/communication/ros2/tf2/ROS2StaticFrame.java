package us.ihmc.communication.ros2.tf2;

import geometry_msgs.msg.dds.TransformStamped;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.ros2.ROS2Node;

/**
 * A ROS 2 frame with a static transform to parent.
 */
public class ROS2StaticFrame extends ROS2Frame
{
   private final RigidBodyTransform previousTransformToRoot;

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node                  ROS 2 node to publish the TFMessage on.
    * @param id                        The frame's id.
    * @param parentFrame               The parent frame.
    * @param transformToParent         Transform to the parent frame.
    */
   public ROS2StaticFrame(ROS2Node ros2Node,
                          String id,
                          ReferenceFrame parentFrame,
                          RigidBodyTransformReadOnly transformToParent)
   {
      this(ros2Node, id, parentFrame, transformToParent, false, false);
   }

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node                  ROS 2 node to publish the TFMessage on.
    * @param id                        The frame's id.
    * @param parentFrame               The parent frame.
    * @param transformToParent         Transform to the parent frame.
    * @param isAStationaryFrame        Whether this frame is stationary with respect to root frame.
    * @param isZUpFrame                Whether this frame has its Z-axis aligned with root frame at all times.
    */
   public ROS2StaticFrame(ROS2Node ros2Node,
                          String id,
                          ReferenceFrame parentFrame,
                          RigidBodyTransformReadOnly transformToParent,
                          boolean isAStationaryFrame,
                          boolean isZUpFrame)
   {
      super(ros2Node, id, parentFrame, transformToParent, isAStationaryFrame, isZUpFrame, true);

      previousTransformToRoot = new RigidBodyTransform(getTransformToRoot());

      publishTFMessages();
   }

   private ROS2StaticFrame(String id)
   {
      this(null, id, null, null, true, true);
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

      if (!previousTransformToRoot.geometricallyEquals(getTransformToRoot(), 1E-6))
         publishTFMessages();

      previousTransformToRoot.set(getTransformToRoot());
   }
}
