package us.ihmc.euclid.jros2.messages;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.fastddsjava.cdr.CDRBuffer;
import us.ihmc.jros2.ROS2Message;

/**
 * Custom ROS2Message wrapper that adapts Euclid RigidBodyTransform to geometry_msgs/Transform.
 * This allows seamless use of Euclid geometry types with ROS2 messaging.
 *
 * Note: geometry_msgs/Transform contains:
 *   geometry_msgs/Vector3 translation
 *   geometry_msgs/Quaternion rotation
 */
public class EuclidTransformMessage implements ROS2Message<EuclidTransformMessage>
{
   /**
    * ROS 2 type name - REQUIRED by jros2
    */
   public static final String name = "geometry_msgs::msg::dds_::Transform_";

   private final RigidBodyTransform transform;

   /**
    * Default constructor - REQUIRED by jros2
    */
   public EuclidTransformMessage()
   {
      this.transform = new RigidBodyTransform();
   }

   /**
    * Constructor that wraps an existing RigidBodyTransform.
    * Note: This does NOT copy the transform, it wraps the same instance.
    */
   public EuclidTransformMessage(RigidBodyTransform transform)
   {
      this.transform = transform;
   }

   @Override
   public int calculateSizeBytes(int currentAlignment)
   {
      int initialAlignment = currentAlignment;

      // Translation (Vector3): x, y, z
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // x
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // y
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // z

      // Rotation (Quaternion): x, y, z, w
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // x
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // y
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // z
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // w

      return currentAlignment - initialAlignment;
   }

   @Override
   public void serialize(CDRBuffer buffer)
   {
      // Serialize translation
      buffer.writeDouble(transform.getTranslationX());
      buffer.writeDouble(transform.getTranslationY());
      buffer.writeDouble(transform.getTranslationZ());

      // Serialize rotation as quaternion
      Quaternion quat = new Quaternion();
      quat.set(transform.getRotation());
      buffer.writeDouble(quat.getX());
      buffer.writeDouble(quat.getY());
      buffer.writeDouble(quat.getZ());
      buffer.writeDouble(quat.getS());
   }

   @Override
   public void deserialize(CDRBuffer buffer)
   {
      // Deserialize translation
      transform.getTranslation().setX(buffer.readDouble());
      transform.getTranslation().setY(buffer.readDouble());
      transform.getTranslation().setZ(buffer.readDouble());

      // Deserialize rotation as quaternion
      Quaternion quat = new Quaternion();
      double qx = buffer.readDouble();
      double qy = buffer.readDouble();
      double qz = buffer.readDouble();
      double qs = buffer.readDouble();
      quat.set(qx, qy, qz, qs);
      transform.getRotation().set(quat);
   }

   @Override
   public void set(EuclidTransformMessage from)
   {
      this.transform.set(from.transform);
   }

   /**
    * Set from a RigidBodyTransform directly.
    */
   public void set(RigidBodyTransform from)
   {
      this.transform.set(from);
   }

   /**
    * Get the wrapped Euclid RigidBodyTransform object.
    * This returns the actual object, not a copy.
    */
   public RigidBodyTransform getTransform()
   {
      return transform;
   }
}
