package us.ihmc.euclid.jros2.messages;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.fastddsjava.cdr.CDRBuffer;
import us.ihmc.jros2.ROS2Message;

/**
 * Custom ROS2Message wrapper that adapts Euclid Quaternion to geometry_msgs/Quaternion.
 * This allows seamless use of Euclid geometry types with ROS2 messaging.
 */
public class EuclidQuaternionMessage implements ROS2Message<EuclidQuaternionMessage>
{
   /**
    * ROS 2 type name - REQUIRED by jros2
    */
   public static final String name = "geometry_msgs::msg::dds_::Quaternion_";

   private final Quaternion quaternion;

   /**
    * Default constructor - REQUIRED by jros2
    */
   public EuclidQuaternionMessage()
   {
      this.quaternion = new Quaternion();
   }

   /**
    * Constructor that wraps an existing Quaternion.
    * Note: This does NOT copy the quaternion, it wraps the same instance.
    */
   public EuclidQuaternionMessage(Quaternion quaternion)
   {
      this.quaternion = quaternion;
   }

   @Override
   public int calculateSizeBytes(int currentAlignment)
   {
      int initialAlignment = currentAlignment;

      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // x
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // y
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // z
      currentAlignment += 8 + CDRBuffer.alignment(currentAlignment, 8); // w

      return currentAlignment - initialAlignment;
   }

   @Override
   public void serialize(CDRBuffer buffer)
   {
      buffer.writeDouble(quaternion.getX());
      buffer.writeDouble(quaternion.getY());
      buffer.writeDouble(quaternion.getZ());
      buffer.writeDouble(quaternion.getS());
   }

   @Override
   public void deserialize(CDRBuffer buffer)
   {
      double x = buffer.readDouble();
      double y = buffer.readDouble();
      double z = buffer.readDouble();
      double s = buffer.readDouble();
      quaternion.set(x, y, z, s);
   }

   @Override
   public void set(EuclidQuaternionMessage from)
   {
      this.quaternion.set(from.quaternion);
   }

   /**
    * Set from a Quaternion directly.
    */
   public void set(Quaternion from)
   {
      this.quaternion.set(from);
   }

   /**
    * Set from a QuaternionBasics directly.
    */
   public void set(QuaternionBasics from)
   {
      this.quaternion.set(from);
   }

   /**
    * Set from a QuaternionReadOnly directly.
    */
   public void set(QuaternionReadOnly from)
   {
      this.quaternion.set(from);
   }

   /**
    * Set from a RotationMatrix directly.
    */
   public void set(RotationMatrix from)
   {
      this.quaternion.set(from);
   }

   /**
    * Set from a RotationMatrixBasics directly.
    */
   public void set(RotationMatrixBasics from)
   {
      this.quaternion.set(from);
   }

   /**
    * Set from a RotationMatrixReadOnly directly.
    */
   public void set(RotationMatrixReadOnly from)
   {
      this.quaternion.set(from);
   }

   /**
    * Get the wrapped Euclid Quaternion object.
    * This returns the actual object, not a copy.
    */
   public Quaternion getQuaternion()
   {
      return quaternion;
   }
}
