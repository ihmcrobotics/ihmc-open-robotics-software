package us.ihmc.communication.ros2.tf2;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.ros2.ROS2Node;

import java.time.Instant;

/**
 * A ROS 2 frame with a static transform to parent.
 */
public class ROS2StaticFrame extends ROS2Frame
{
   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node          ROS 2 node to publish the TFMessage on.
    * @param id                The frame's id.
    * @param parentFrame       The parent frame.
    * @param transformToParent Transform to the parent frame.
    */
   public ROS2StaticFrame(ROS2Node ros2Node, String id, ROS2Frame parentFrame, RigidBodyTransformReadOnly transformToParent)
   {
      this(ros2Node, id, parentFrame, transformToParent, Instant.now());
   }

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node           ROS 2 node to publish the TFMessage on.
    * @param id                 The frame's id.
    * @param parentFrame        The parent frame.
    * @param transformToParent  Transform to the parent frame.
    * @param transformTimestamp Timestamp of the transform.
    */
   public ROS2StaticFrame(ROS2Node ros2Node, String id, ROS2Frame parentFrame, RigidBodyTransformReadOnly transformToParent, Instant transformTimestamp)
   {
      this(ros2Node, id, parentFrame, transformToParent, (int) transformTimestamp.getEpochSecond(), transformTimestamp.getNano());
   }

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node                  ROS 2 node to publish the TFMessage on.
    * @param id                        The frame's id.
    * @param parentFrame               The parent frame.
    * @param transformToParent         Transform to the parent frame.
    * @param transformTimestampSeconds Timestamp of the transform, in seconds.
    * @param transformTimestampNanos   Additional nanoseconds to the timestamp seconds.
    */
   public ROS2StaticFrame(ROS2Node ros2Node,
                          String id,
                          ROS2Frame parentFrame,
                          RigidBodyTransformReadOnly transformToParent,
                          int transformTimestampSeconds,
                          int transformTimestampNanos)
   {
      this(ros2Node, id, parentFrame, transformToParent, transformTimestampSeconds, transformTimestampNanos, false, false);
   }

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node           ROS 2 node to publish the TFMessage on.
    * @param id                 The frame's id.
    * @param parentFrame        The parent frame.
    * @param transformToParent  Transform to the parent frame.
    * @param transformTimestamp Timestamp of the transform.
    * @param isAStationaryFrame Whether this frame is stationary with respect to root frame.
    * @param isZUpFrame         Whether this frame has its Z-axis aligned with root frame at all times.
    */
   public ROS2StaticFrame(ROS2Node ros2Node,
                          String id,
                          ROS2Frame parentFrame,
                          RigidBodyTransformReadOnly transformToParent,
                          Instant transformTimestamp,
                          boolean isAStationaryFrame,
                          boolean isZUpFrame)
   {
      this(ros2Node,
           id,
           parentFrame,
           transformToParent,
           (int) transformTimestamp.getEpochSecond(),
           transformTimestamp.getNano(),
           isAStationaryFrame,
           isZUpFrame);
   }

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node                  ROS 2 node to publish the TFMessage on.
    * @param id                        The frame's id.
    * @param parentFrame               The parent frame.
    * @param transformToParent         Transform to the parent frame.
    * @param transformTimestampSeconds Timestamp of the transform, in seconds.
    * @param transformTimestampNanos   Additional nanoseconds to the timestamp seconds.
    * @param isAStationaryFrame        Whether this frame is stationary with respect to root frame.
    * @param isZUpFrame                Whether this frame has its Z-axis aligned with root frame at all times.
    */
   public ROS2StaticFrame(ROS2Node ros2Node,
                          String id,
                          ROS2Frame parentFrame,
                          RigidBodyTransformReadOnly transformToParent,
                          int transformTimestampSeconds,
                          int transformTimestampNanos,
                          boolean isAStationaryFrame,
                          boolean isZUpFrame)
   {
      super(ros2Node, id, parentFrame, transformToParent, transformTimestampSeconds, transformTimestampNanos, isAStationaryFrame, isZUpFrame, true);
   }

   private ROS2StaticFrame(String id, Instant now)
   {
      this(null, id, null, null, (int) now.getEpochSecond(), now.getNano(), true, true);
   }

   /**
    * Constructs a root frame.
    *
    * @param id The frame's id.
    * @return The root frame.
    */
   public static ROS2StaticFrame constructARootFrame(String id)
   {
      return constructARootFrame(id, Instant.now());
   }

   /**
    * Constructs a root frame.
    *
    * @param id  The frame's id.
    * @param now Current time.
    * @return The root frame.
    */
   public static ROS2StaticFrame constructARootFrame(String id, Instant now)
   {
      return new ROS2StaticFrame(id, now);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
   }
}
