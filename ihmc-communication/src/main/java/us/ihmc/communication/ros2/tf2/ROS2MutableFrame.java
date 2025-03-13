package us.ihmc.communication.ros2.tf2;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.ros2.ROS2Node;

import java.time.Instant;

/**
 * A ROS 2 frame with a changing transform to parent.
 */
public class ROS2MutableFrame extends ROS2Frame
{
   private final RigidBodyTransform newestTransformToParent;

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node    ROS 2 node to publish the TFMessage on.
    * @param id          The frame's id.
    * @param parentFrame The parent frame.
    */
   public ROS2MutableFrame(ROS2Node ros2Node, String id, ROS2Frame parentFrame)
   {
      this(ros2Node, id, parentFrame, null);
   }

   /**
    * Constructs a non-root frame.
    *
    * @param ros2Node          ROS 2 node to publish the TFMessage on.
    * @param id                The frame's id.
    * @param parentFrame       The parent frame.
    * @param transformToParent Transform to the parent frame.
    */
   public ROS2MutableFrame(ROS2Node ros2Node, String id, ROS2Frame parentFrame, RigidBodyTransformReadOnly transformToParent)
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
   public ROS2MutableFrame(ROS2Node ros2Node, String id, ROS2Frame parentFrame, RigidBodyTransformReadOnly transformToParent, Instant transformTimestamp)
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
   public ROS2MutableFrame(ROS2Node ros2Node,
                           String id,
                           ROS2Frame parentFrame,
                           RigidBodyTransformReadOnly transformToParent,
                           int transformTimestampSeconds,
                           int transformTimestampNanos)
   {
      this(ros2Node, id, parentFrame, transformToParent, transformTimestampSeconds, transformTimestampNanos, false);
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
    * @param isZUpFrame                Whether this frame has its Z-axis aligned with root frame at all times.
    */
   public ROS2MutableFrame(ROS2Node ros2Node,
                           String id,
                           ROS2Frame parentFrame,
                           RigidBodyTransformReadOnly transformToParent,
                           int transformTimestampSeconds,
                           int transformTimestampNanos,
                           boolean isZUpFrame)
   {
      super(ros2Node, id, parentFrame, transformToParent, transformTimestampSeconds, transformTimestampNanos, false, isZUpFrame, false);

      newestTransformToParent = new RigidBodyTransform();
      getTransformToParent(newestTransformToParent);
   }

   /**
    * Update this frame's transform to parent with the transform timestamp set as now.
    *
    * @param newTransformToParent The new transform to parent for this frame.
    */
   public void updateTransform(RigidBodyTransformReadOnly newTransformToParent)
   {
      updateTransform(newTransformToParent, Instant.now());
   }

   /**
    * Update this frame's transform to parent.
    *
    * @param newTransformToParent The new transform to parent for this frame.
    * @param timestamp            Timestamp of the new transform.
    */
   public void updateTransform(RigidBodyTransformReadOnly newTransformToParent, Instant timestamp)
   {
      updateTransform(newTransformToParent, (int) timestamp.getEpochSecond(), timestamp.getNano());
   }

   /**
    * Update this frame's transform to parent.
    *
    * @param newTransformToParent The new transform to parent for this frame.
    * @param timestampSeconds     Seconds of the timestamp of the new transform.
    * @param timestampNanos       Nanoseconds of the timestamp of the new transform.
    */
   public void updateTransform(RigidBodyTransformReadOnly newTransformToParent, int timestampSeconds, int timestampNanos)
   {
      newestTransformToParent.set(newTransformToParent);
      lastUpdateTimestampSeconds = timestampSeconds;
      lastUpdateTimestampNanos = timestampNanos;
      update();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.set(newestTransformToParent);
   }
}
