package us.ihmc.euclid.jros2.messages;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.fastddsjava.cdr.CDRBuffer;
import us.ihmc.jros2.ROS2Message;

/**
 * Custom ROS2Message wrapper that adapts Euclid Pose3D to geometry_msgs/Pose.
 * This allows seamless use of Euclid geometry types with ROS2 messaging.
 *
 * Note: geometry_msgs/Pose contains:
 *   geometry_msgs/Point position
 *   geometry_msgs/Quaternion orientation
 */
public class EuclidPose3DMessage implements ROS2Message<EuclidPose3DMessage>
{
   /**
    * ROS 2 type name - REQUIRED by jros2
    */
   public static final String name = "geometry_msgs::msg::dds_::Pose_";

   private final Pose3D pose;

   /**
    * Default constructor - REQUIRED by jros2
    */
   public EuclidPose3DMessage()
   {
      this.pose = new Pose3D();
   }

   /**
    * Constructor that wraps an existing Pose3D.
    * Note: This does NOT copy the pose, it wraps the same instance.
    */
   public EuclidPose3DMessage(Pose3D pose)
   {
      this.pose = pose;
   }

   @Override
   public int calculateSizeBytes(int currentAlignment)
   {
      int initialAlignment = currentAlignment;

      // Position (Point3D): x, y, z
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // x
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // y
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // z

      // Orientation (Quaternion): x, y, z, w
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // x
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // y
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // z
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // w

      return currentAlignment - initialAlignment;
   }

   @Override
   public void serialize(CDRBuffer buffer)
   {
      // Serialize position
      buffer.writeDouble(pose.getX());
      buffer.writeDouble(pose.getY());
      buffer.writeDouble(pose.getZ());

      // Serialize orientation
      buffer.writeDouble(pose.getOrientation().getX());
      buffer.writeDouble(pose.getOrientation().getY());
      buffer.writeDouble(pose.getOrientation().getZ());
      buffer.writeDouble(pose.getOrientation().getS());
   }

   @Override
   public void deserialize(CDRBuffer buffer)
   {
      // Deserialize position
      pose.setX(buffer.readDouble());
      pose.setY(buffer.readDouble());
      pose.setZ(buffer.readDouble());

      // Deserialize orientation
      double qx = buffer.readDouble();
      double qy = buffer.readDouble();
      double qz = buffer.readDouble();
      double qs = buffer.readDouble();
      pose.getOrientation().set(qx, qy, qz, qs);
   }

   @Override
   public void set(EuclidPose3DMessage from)
   {
      this.pose.set(from.pose);
   }

   /**
    * Set from a Pose3D directly.
    */
   public void set(Pose3D from)
   {
      this.pose.set(from);
   }

   /**
    * Get the wrapped Euclid Pose3D object.
    * This returns the actual object, not a copy.
    */
   public Pose3D getPose()
   {
      return pose;
   }
}
